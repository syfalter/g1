#include "g1_tcp_diagnostics/tcp_a_panel.hpp"
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QStyle>
#include <QTableWidget>
#include <QTabWidget>
#include <QTimer>
#include <QUuid>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <cmath>

namespace g1_tcp_diagnostics {
namespace {
QLabel *paragraph(const QString &text) {
  auto *label=new QLabel(text); label->setWordWrap(true);
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  return label;
}
QWidget *wrappedCheck(QCheckBox *box) {
  const auto text=box->text(); box->setText(""); box->setAccessibleName(text);
  auto *row=new QWidget; auto *layout=new QHBoxLayout(row); layout->setContentsMargins(0,0,0,0);
  layout->addWidget(box,0,Qt::AlignTop); layout->addWidget(paragraph(text),1); return row;
}
QPlainTextEdit *readout(const QString &name) {
  auto *out=new QPlainTextEdit; out->setReadOnly(true); out->setObjectName(name);
  out->setLineWrapMode(QPlainTextEdit::WidgetWidth); return out;
}
QString vectorText(const QJsonArray &values, double scale=1000.) {
  QStringList pieces; for (const auto &v:values) pieces << QString::number(v.toDouble()*scale,'f',3);
  return "["+pieces.join(", ")+"]";
}
QString millimeters(const QJsonObject &values, const QString &key) {
  return values.contains(key)?QString::number(values[key].toDouble()*1000.,'f',3):"--";
}
}

TcpAPanel::TcpAPanel(QWidget *parent):rviz_common::Panel(parent) {
  setObjectName("tcpAPanel"); setMinimumWidth(430);
  auto *root=new QVBoxLayout(this); root->setContentsMargins(10,10,10,10); root->setSpacing(7);
  auto *title=paragraph("<b>实验 A · 食指同点一致性 / TCP</b>"); root->addWidget(title);
  connection_=paragraph("等待诊断后端 · 采样只读 · pelvis / 胯部"); connection_->setObjectName("connection");
  root->addWidget(connection_);
  identity_=paragraph("未连接会话"); root->addWidget(identity_);
  auto *files=new QHBoxLayout;
  side_=new QComboBox; side_->setObjectName("side"); side_->addItem("左手","left"); side_->addItem("右手","right");
  files->addWidget(side_);
  auto *newButton=new QPushButton("新会话"); newButton->setIcon(style()->standardIcon(QStyle::SP_FileIcon)); files->addWidget(newButton);
  auto *load=new QPushButton; load->setIcon(style()->standardIcon(QStyle::SP_DialogOpenButton)); load->setToolTip("加载实验A会话"); files->addWidget(load);
  auto *save=new QPushButton; save->setIcon(style()->standardIcon(QStyle::SP_DialogSaveButton)); save->setToolTip("另存会话快照，不覆盖旧文件"); files->addWidget(save);
  root->addLayout(files);
  connect(newButton,&QPushButton::clicked,this,[this]{
    if (QMessageBox::question(this,"新建实验会话","当前样本会先保存，原始记录不会删除。确认开始新的安装/手型会话？") == QMessageBox::Yes) {
      source_->setChecked(false); pelvis_->setChecked(false); hand_->setChecked(false); contact_->setChecked(false); light_->setChecked(false);
      send("new",{{"side",side_->currentData().toString()}});
    }
  });
  connect(load,&QPushButton::clicked,this,[this]{
    const auto file=QFileDialog::getOpenFileName(this,"加载实验A原始样本","/home/syf/g1_upper_body_ws/data/tcp_a","JSON (*.json)");
    if (!file.isEmpty()) {
      source_->setChecked(false); pelvis_->setChecked(false); hand_->setChecked(false); contact_->setChecked(false); light_->setChecked(false);
      send("load",{{"path",file}});
    }
  });
  connect(save,&QPushButton::clicked,this,[this]{send("save");});
  tabs_=new QTabWidget; tabs_->setObjectName("tabs"); root->addWidget(tabs_,1);

  auto *guide=new QWidget; auto *guideLayout=new QVBoxLayout(guide);
  controlStatus_=paragraph("等待原任务控制进程；启动本面板不会设置手型或切换模式。");
  controlStatus_->setObjectName("controlStatus"); guideLayout->addWidget(controlStatus_);
  index_=new QPushButton("1  设置所选手伸食指（真机）"); index_->setObjectName("indexOnly");
  waistAdmittance_=new QCheckBox("导纳包含腰 yaw（须确认躯干净空与机器人支撑）");
  waistAdmittance_->setObjectName("waistAdmittance"); guideLayout->addWidget(wrappedCheck(waistAdmittance_));
  connect(waistAdmittance_,&QCheckBox::toggled,this,[this]{refreshControls();});
  admittance_=new QPushButton("2  STOP任务并进入双臂导纳（真机）"); admittance_->setObjectName("admittance");
  hold_=new QPushButton("离开接触后：恢复当前姿态保持（真机）"); hold_->setObjectName("hold");
  for(auto *button:{index_,admittance_,hold_}) {button->setEnabled(false); button->setMinimumHeight(34); guideLayout->addWidget(button);}
  connect(index_,&QPushButton::clicked,this,[this]{control("index_only");});
  connect(admittance_,&QPushButton::clicked,this,[this]{control("admittance");});
  connect(hold_,&QPushButton::clicked,this,[this]{control("hold");});
  guideLayout->addWidget(paragraph(
    "<b>检查什么</b><br>不使用相机，检查同一个实体触点在不同实测腕姿下是否得到一致坐标。"
    "低残差不等于整条机械链绝对准确。<br><br>"
    "<b>1 · 固定基准</b><br>固定桌面参考块，胯部相对它不挪动、不明显摆动。"
    "不要把RViz的固定pelvis或world TF当作实体固定证据。<br><br>"
    "<b>2 · 指定接触点</b><br>食指最末端正前方选一个可辨识的小点，"
    "每次用这个实体位置轻触参考块上的同一个十字/定位点，不用整片指腹滚动接触。"
    "标定板可以放在背景，但本实验不需要相机，也不使用板的视觉坐标。<br><br>"
    "<b>3 · 设置伸指，再进入导纳</b><br>先在无接触、双臂静止保持时点击上方伸指按钮，"
    "仅所选手食指伸出、其余手指收拢；等待反馈完成并目视核对。然后确认进入双臂导纳，"
    "等原网关显示TASK_ADMITTANCE后才手动引导。勾选包含腰yaw时，还需确认腰导纳已启用；未勾选则腰保持，不可强扭。"
    "启动不会自动ARM或设置手型。"
    "同一会话中保持手型、安装和负载条件不变；执行器反馈不等于被动指骨实测角度。<br><br>"
    "<b>4 · 统一腰臂采样</b><br>先同腕姿重复轻触3次，再用不同方向的腕姿轻触同一点，"
    "建议8~12组。不能只平移或只绕一根轴；先离开接触面再改变腕姿，不顶着参考块扭腕。"
    "轻触后稳定至少1秒，去接触采样页确认本次接触再采样。导纳中可采样，不能用力顶住来制造稳定。"
    "若要恢复保持，先离开实体点，保持静止，再点恢复按钮；它不是返回旧姿态。<br><br>"
    "<b>5 · 独立验证</b><br>额外留5个新姿态，不参与拟合。两组均可包含转腰，不再区分腰部批次。"
    "腰yaw与手臂一起使用实测FK，所有点统一到pelvis；胯部相对参考点仍必须固定。腰roll/pitch不开放导纳。"
    "更换实体点时使用新的触点ID。<br><br>"
    "<b>无有效固定触点时</b><br>若指尖滑动、软胶压缩、手指下垂或关节被动变化不能控制，"
    "只记录重复性，不宣称有效TCP标定。可另做刚性探针对照，不能把探针TCP套给食指。"));
  guideLayout->addStretch(); auto *guideScroll=new QScrollArea; guideScroll->setWidgetResizable(true); guideScroll->setWidget(guide);
  tabs_->addTab(guideScroll,"布置引导");

  auto *capturePage=new QWidget; auto *captureLayout=new QVBoxLayout(capturePage);
  auto *form=new QFormLayout; form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
  target_=new QLineEdit("point_01"); target_->setObjectName("targetId"); target_->setMaxLength(80);
  form->addRow("实体触点 ID",target_);
  group_=new QComboBox; group_->addItem("拟合组","fit"); group_->addItem("独立验证组","validation"); group_->setObjectName("group"); form->addRow("样本用途",group_);
  note_=new QLineEdit; note_->setMaxLength(500); form->addRow("接触/姿态备注",note_);
  captureLayout->addLayout(form);
  source_=new QCheckBox("已核对实际LowState来源，非演示/目标状态"); source_->setObjectName("sourceCheck");
  pelvis_=new QCheckBox("胯部与实体参考点相对不动，腰roll/pitch条件已核实"); pelvis_->setObjectName("pelvisCheck");
  hand_=new QCheckBox("人工确认固定index_only，接触点相对腕部不变"); hand_->setObjectName("handCheck");
  contact_=new QCheckBox("本次：指定食指小点已真实接触指定实体点"); contact_->setObjectName("contactCheck");
  light_=new QCheckBox("本次：轻触，无滑动、无明显压缩或指节弯曲"); light_->setObjectName("lightCheck");
  for (auto *box:{source_,pelvis_,hand_,contact_,light_}) {
    captureLayout->addWidget(wrappedCheck(box)); connect(box,&QCheckBox::toggled,this,[this]{updateCapture();});
  }
  readHand_=new QCheckBox("采样时只读手部DDS，附加执行器一致性检查"); readHand_->setObjectName("readHand");
  captureLayout->addWidget(wrappedCheck(readHand_));
  captureLayout->addWidget(paragraph("未勾选DDS时，样本明确记录人工固定手型假设，不把模型角度标成实测。手部读取期间保持当前轻触；失败不会发送hold或动作。"));
  quality_=paragraph("等待实际状态"); quality_->setObjectName("quality"); captureLayout->addWidget(quality_);
  capture_=new QPushButton("确认轻触，采样1秒取平均"); capture_->setObjectName("capture"); capture_->setMinimumHeight(42); capture_->setEnabled(false);
  capture_->setIcon(style()->standardIcon(QStyle::SP_DialogApplyButton));
  auto *undo=new QPushButton("撤销上一条（保留原始数据）"); undo->setIcon(style()->standardIcon(QStyle::SP_ArrowBack));
  captureLayout->addWidget(paragraph("采样只读。真机伸指/导纳/保持仅在布置页单独确认，并交给原任务进程执行。未到命令目标但实测稳定时可记录；漂移时拒绝。"));
  captureLayout->addStretch();
  auto *captureScroll=new QScrollArea; captureScroll->setWidgetResizable(true); captureScroll->setWidget(capturePage);
  auto *captureTab=new QWidget; auto *captureTabLayout=new QVBoxLayout(captureTab);
  captureTabLayout->setContentsMargins(0,0,0,0); captureTabLayout->addWidget(captureScroll,1);
  captureTabLayout->addWidget(capture_); captureTabLayout->addWidget(undo); tabs_->addTab(captureTab,"接触采样");
  connect(target_,&QLineEdit::textChanged,this,[this]{updateCapture();});
  connect(side_,qOverload<int>(&QComboBox::currentIndexChanged),this,[this]{updateCapture();});
  connect(capture_,&QPushButton::clicked,this,&TcpAPanel::capture);
  connect(undo,&QPushButton::clicked,this,[this]{send("undo");});

  auto *samplePage=new QWidget; auto *sampleLayout=new QVBoxLayout(samplePage);
  samples_=new QTableWidget(0,6); samples_->setObjectName("samples");
  samples_->setHorizontalHeaderLabels({"样本","触点","用途","状态","跟踪mm","腰deg"});
  samples_->setSelectionBehavior(QAbstractItemView::SelectRows); samples_->setSelectionMode(QAbstractItemView::SingleSelection);
  samples_->setEditTriggers(QAbstractItemView::NoEditTriggers); samples_->verticalHeader()->hide();
  samples_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  samples_->horizontalHeader()->setStretchLastSection(true); samples_->setMinimumHeight(160);
  sampleLayout->addWidget(samples_,1);
  auto *sampleButtons=new QHBoxLayout;
  auto *exclude=new QPushButton("排除 / 恢复"); sampleButtons->addWidget(exclude);
  auto *changeGroup=new QPushButton("拟合 / 验证"); sampleButtons->addWidget(changeGroup); sampleLayout->addLayout(sampleButtons);
  sampleDetails_=readout("sampleDetails"); sampleDetails_->setMinimumHeight(120); sampleLayout->addWidget(sampleDetails_,1);
  sampleLayout->addWidget(paragraph("蓝点：原模型；绿点：候选TCP；紫点：估计固定触点（非真值）；红线：候选残差；黄框/线：控制参考及跟踪差。方块为验证样本，灰色为排除样本，黄色光圈为选中样本。"));
  tabs_->addTab(samplePage,"样本残差");
  connect(exclude,&QPushButton::clicked,this,[this]{send("exclude",{{"sample_id",selectedId()}});});
  connect(changeGroup,&QPushButton::clicked,this,[this]{send("group",{{"sample_id",selectedId()}});});
  connect(samples_,&QTableWidget::itemSelectionChanged,this,[this]{
    if (rebuilding_)return;
    const auto id=selectedId();
    for(const auto &v:status_["samples"].toArray()) if(v.toObject()["id"].toString()==id)
      sampleDetails_->setPlainText(QString::fromUtf8(QJsonDocument(v.toObject()).toJson(QJsonDocument::Indented)));
    send("select",{{"sample_id",id}});
  });

  auto *solvePage=new QWidget; auto *solveLayout=new QVBoxLayout(solvePage);
  solveSummary_=paragraph("尚未求解。至少6个有效拟合样本，并需多轴腕姿覆盖。"); solveLayout->addWidget(solveSummary_);
  solve_=new QPushButton("求解固定手型 TCP 并验证"); solve_->setObjectName("solve"); solve_->setMinimumHeight(38); solveLayout->addWidget(solve_);
  candidate_=new QCheckBox("显示候选点与残差（不影响采样基线）"); candidate_->setChecked(true); solveLayout->addWidget(wrappedCheck(candidate_));
  results_=readout("results"); solveLayout->addWidget(results_,1);
  export_=new QPushButton("导出 CSV / JSON / YAML / 中文报告"); export_->setObjectName("export"); solveLayout->addWidget(export_);
  solveLayout->addWidget(paragraph("只生成新文件，不覆盖在线配置，不发布候选TF。新点验证只评价同点一致性；没有独立机械基准就不能证明绝对精度。"));
  tabs_->addTab(solvePage,"求解导出");
  connect(solve_,&QPushButton::clicked,this,[this]{send("solve");});
  connect(export_,&QPushButton::clicked,this,[this]{send("export");});
  connect(candidate_,&QCheckBox::toggled,this,[this](bool on){send("preview",{{"candidate",on}});});

  auto *systemPage=new QWidget; auto *systemLayout=new QVBoxLayout(systemPage);
  system_=readout("system"); systemLayout->addWidget(system_,1);
  demo_=new QPushButton("加载合成示例（仅离线）"); demo_->setObjectName("demo"); demo_->setEnabled(false); systemLayout->addWidget(demo_);
  connect(demo_,&QPushButton::clicked,this,[this]{send("demo");}); tabs_->addTab(systemPage,"跟踪系统");
  message_=paragraph("未发送任何请求"); message_->setObjectName("message");
  message_->setMaximumHeight(108); root->addWidget(message_);
  timer_=new QTimer(this); timer_->setInterval(150); connect(timer_,&QTimer::timeout,this,&TcpAPanel::refresh); timer_->start();
}

void TcpAPanel::onInitialize() {
  auto abstraction=getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) return;
  node_=abstraction->get_raw_node();
  controlPub_=node_->create_publisher<std_msgs::msg::String>("/g1/task/tcp_a_control_request",rclcpp::QoS(10));
  ownerSub_=node_->create_subscription<std_msgs::msg::String>("/g1/task/console_status",rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::String::SharedPtr msg){
      const auto doc=QJsonDocument::fromJson(QByteArray::fromStdString(msg->data));
      if(doc.isObject())setOwnerStatus(doc.object());
    });
  requestPub_=node_->create_publisher<std_msgs::msg::String>("/g1/tcp_a/request",rclcpp::QoS(10));
  statusSub_=node_->create_subscription<std_msgs::msg::String>("/g1/tcp_a/status",rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::String::SharedPtr msg){
      const auto doc=QJsonDocument::fromJson(QByteArray::fromStdString(msg->data));
      if(doc.isObject())setStatus(doc.object());
    });
}

void TcpAPanel::setStatus(const QJsonObject &status) {
  std::lock_guard<std::mutex> lock(mutex_); pending_=status; dirty_=true; receivedMs_=QDateTime::currentMSecsSinceEpoch();
}

void TcpAPanel::setOwnerStatus(const QJsonObject &status) {
  std::lock_guard<std::mutex> lock(mutex_); pendingOwner_=status; ownerReceivedMs_=QDateTime::currentMSecsSinceEpoch();
}

void TcpAPanel::refreshControls() {
  const auto proof=owner_["tcp_a_controls"].toObject();
  const auto now=QDateTime::currentMSecsSinceEpoch();
  const bool fresh=now-ownerReceivedMs_<1800 && std::abs(now/1000.-proof["stamp"].toDouble())<1.8;
  // console_status.gateway is the phase string, not the raw diagnostic map.
  const auto phase=owner_["gateway"].toString();
  const bool armedPhase=phase=="ARMED_HOLD" || phase=="RUNNING" || phase=="TASK_ADMITTANCE";
  const bool usable=connected_ && status_["mode"].toString()=="live" && fresh && proof["protocol"].toString()=="tcp_a_owner_v1"
    && proof["hardware"].toBool() && proof["gateway_fresh"].toBool() && armedPhase;
  const bool pending=!controlRequest_.isEmpty() && now-controlSentMs_<12000;
  const bool idle=usable && !owner_["worker_busy"].toBool() && !busy_ && !pending;
  index_->setEnabled(idle && phase=="ARMED_HOLD" && proof["hand_enabled"].toBool() &&
    status_["samples"].toArray().isEmpty() && side_->currentData().toString()==status_["side"].toString());
  admittance_->setEnabled(usable && (!pending || owner_["worker_busy"].toBool()) && (phase=="ARMED_HOLD" || phase=="RUNNING"));
  hold_->setEnabled(idle && phase=="TASK_ADMITTANCE");
  if(waistAdmittance_->isChecked())
    admittance_->setEnabled(idle && phase=="ARMED_HOLD" && proof["waist_admittance_supported"].toBool());
  const auto last=proof["last"].toObject();
  if(!controlRequest_.isEmpty() && last["request_id"].toString()==controlRequest_ &&
     (last["state"].toString()!="accepted" || (last["op"].toString()=="admittance" && phase=="TASK_ADMITTANCE") ||
      (last["op"].toString()=="hold" && phase=="ARMED_HOLD"))) controlRequest_.clear();
  QString text;
  if(status_["mode"].toString()=="offline") text="离线模式：所有真机操作禁用。";
  else if(proof.isEmpty()) text="原任务面板尚未加载实验控制接口；由操作者正常更新/重开原面板后连接。不会另起控制器或自动重启。";
  else if(!fresh) text="原任务控制进程断流：真机操作禁用，请查看原面板/物理停止装置。";
  else text="原任务进程 "+QString::number(proof["console_pid"].toInt())+" · "+phase+"\n"+last["message"].toString();
  if(pending) text+="\n请求已发出，等待原进程/网关确认；不要重复点击。";
  else if(!controlRequest_.isEmpty()) text+="\n操作结果未确认，不自动重试。检查原面板实际状态。";
  text += !fresh?"\n腰 yaw 导纳：状态未确认，勿拖动":
    proof["waist_admittance_active"].toBool() && phase=="TASK_ADMITTANCE"?"\n腰 yaw 导纳：已启用":"\n腰 yaw 导纳：尚未就绪，勿拖动";
  controlStatus_->setText(text);
  index_->setToolTip("需要：原控制器已ARM并静止保持、空实验会话、所选手及另一臂周围净空；仅所选手运动。");
  admittance_->setToolTip("复用原控制器；腰导纳需勾选且从静止保持进入。不是物理急停。");
  hold_->setToolTip("无接触且静止后恢复当前实测姿态保持，不返回旧姿态、不执行规划。");
}

void TcpAPanel::control(const QString &operation) {
  refreshControls();
  auto *button=operation=="index_only"?index_:operation=="admittance"?admittance_:hold_;
  if(!button->isEnabled())return;
  const auto proof=owner_["tcp_a_controls"].toObject();
  const auto session=status_["session_id"].toString(); const auto side=side_->currentData().toString();
  const QString detail=operation=="index_only"?
    "将设置"+side_->currentText()+"食指伸出、其余手指收拢。确认手指没有接触物体/人体，双臂静止、机器人稳固、周围净空且能立即停止？仅所选手动作。":
    operation=="admittance"?(waistAdmittance_->isChecked()?"将进入双臂＋腰yaw导纳。确认机器人支撑可靠、躯干及双臂净空，无实体接触且静止保持？首次转腰轻推、小角度观察，异常立即停止。等待网关确认后才拖动。这不是急停。":"将中止原任务并使双臂进入导纳，腰仍保持。确认双臂均有支撑、活动空间充足？等待TASK_ADMITTANCE后才拖动。这不是急停。"):
    "确认已离开实体触点、双臂静止且姿态可保持？将退出导纳并保持当前实测姿态，不返回旧目标。";
  if(QMessageBox::question(this,"确认真机操作",detail,QMessageBox::Yes|QMessageBox::No,QMessageBox::No)!=QMessageBox::Yes)return;
  refreshControls();
  if(!button->isEnabled() || session!=status_["session_id"].toString() || side!=side_->currentData().toString())return;
  if(node_ && node_->count_publishers("/g1/task/console_status")!=1) {
    controlStatus_->setText("原任务状态发布者不是唯一，拒绝操作。"); return;
  }
  QJsonObject request=proof;
  request.remove("last"); request["op"]=operation; request["side"]=side;
  request["request_id"]=QUuid::createUuid().toString(QUuid::WithoutBraces);
  request["sent_at"]=QDateTime::currentMSecsSinceEpoch()/1000.;
  request["confirmed"]=true; request["no_contact"]=operation!="admittance";
  request["both_arms_clear"]=true; request["sample_count"]=status_["samples"].toArray().size();
  request["include_waist"]=operation=="admittance" && waistAdmittance_->isChecked();
  request["waist_clear"]=request["include_waist"];
  request["diagnostic_session_id"]=session;
  if(!controlPub_)return;
  std_msgs::msg::String msg; msg.data=QJsonDocument(request).toJson(QJsonDocument::Compact).toStdString();
  controlPub_->publish(msg); controlRequest_=request["request_id"].toString(); controlSentMs_=QDateTime::currentMSecsSinceEpoch();
  contact_->setChecked(false); light_->setChecked(false); if(operation=="index_only")hand_->setChecked(false);
  refreshControls();
}

void TcpAPanel::send(const QString &operation, QJsonObject fields) {
  if (!connected_ || (busy_ && operation!="select" && operation!="preview"))return;
  fields["op"]=operation; fields["request_id"]=QUuid::createUuid().toString(QUuid::WithoutBraces);
  fields["session_id"]=status_["session_id"];
  fields["sent_at"]=QDateTime::currentMSecsSinceEpoch()/1000.;
  Q_EMIT requestBuilt(fields);
  if(requestPub_) {std_msgs::msg::String msg; msg.data=QJsonDocument(fields).toJson(QJsonDocument::Compact).toStdString(); requestPub_->publish(msg);}
  if(operation!="select" && operation!="preview") {
    busy_=true; updateCapture(); message_->setText("处理中："+operation+"。机器人仍由原控制界面管理。");
  }
}

void TcpAPanel::capture() {
  if(!capture_->isEnabled())return;
  QJsonObject confirmations{{"source_actual",source_->isChecked()},{"pelvis_fixed",pelvis_->isChecked()},
    {"hand_rigid",hand_->isChecked()},{"contact_fixed",contact_->isChecked()},{"light_contact",light_->isChecked()}};
  send("capture",{{"confirmations",confirmations},{"target_id",target_->text().trimmed()},
    {"phase","unified"},{"group",group_->currentData().toString()},
    {"note",note_->text()},{"read_hand",readHand_->isChecked()}});
  contact_->setChecked(false); light_->setChecked(false);
}

QString TcpAPanel::selectedId() const {
  const int row=samples_->currentRow();
  return row>=0 && samples_->item(row,0)?samples_->item(row,0)->data(Qt::UserRole).toString():QString();
}

void TcpAPanel::updateCapture() {
  capture_->setEnabled(connected_ && !busy_ && !owner_["worker_busy"].toBool() && controlRequest_.isEmpty() && backendReady_ && status_["mode"].toString()=="live" &&
    side_->currentData().toString()==status_["side"].toString() && !target_->text().trimmed().isEmpty() &&
    source_->isChecked() && pelvis_->isChecked() && hand_->isChecked() && contact_->isChecked() && light_->isChecked());
  QStringList reasons;
  if(!connected_)reasons<<"诊断后端未连接";
  if(busy_)reasons<<"正在处理，请等待完成";
  if(!backendReady_)reasons<<status_["reason"].toString();
  if(side_->currentData().toString()!=status_["side"].toString())reasons<<"手侧与会话不一致";
  if(!source_->isChecked() || !pelvis_->isChecked() || !hand_->isChecked())reasons<<"请确认实际来源、胯部固定、固定手型";
  if(!contact_->isChecked() || !light_->isChecked())reasons<<"请确认本次真实轻触且无滑动压缩";
  capture_->setToolTip(reasons.isEmpty()?"点击后继续保持轻触1秒，完成后保存一条平均样本":reasons.join("\n"));
}

void TcpAPanel::refresh() {
  qint64 received;
  {std::lock_guard<std::mutex> lock(mutex_); if(dirty_){status_=pending_; dirty_=false;} received=receivedMs_; owner_=pendingOwner_;}
  connected_=received>0 && QDateTime::currentMSecsSinceEpoch()-received<1800 &&
    std::abs(QDateTime::currentMSecsSinceEpoch()/1000.-status_["stamp"].toDouble())<1.8;
  refreshControls();
  if(!connected_) {connection_->setText("诊断后端断流 / 未启动 · 禁止采样"); backendReady_=false; updateCapture(); solve_->setEnabled(false); export_->setEnabled(false); return;}
  busy_=status_["busy"].toBool(); backendReady_=status_["ready"].toBool();
  if(activeSession_!=status_["session_id"].toString()) {
    activeSession_=status_["session_id"].toString(); side_->setCurrentIndex(status_["side"].toString()=="right"?1:0);
    for(auto *box:{source_,pelvis_,hand_,contact_,light_})box->setChecked(false);
    group_->setCurrentIndex(0);
  }
  const bool offline=status_["mode"].toString()=="offline";
  connection_->setText(offline?"离线回放 · 非本次真机测量 · pelvis / 胯部":
    (backendReady_?"实际反馈稳定 · 采样只读 · pelvis / 胯部":"采样只读 · 等待采样条件 · pelvis / 胯部"));
  identity_->setText("会话 "+status_["session_id"].toString().left(12)+" · "+
    (status_["side"].toString()=="left"?"左手":"右手")+" · 样本 "+QString::number(status_["samples"].toArray().size()));
  message_->setText(status_["message"].toString());
  demo_->setEnabled(offline && !busy_);
  solve_->setEnabled(!busy_ && status_["samples"].toArray().size()>=6);
  export_->setEnabled(!busy_ && status_["result"].isObject());
  const auto q=status_["quality"].toObject();
  const auto progress=status_["capture_progress"].toObject();
  capture_->setText(progress.isEmpty()?"确认轻触，采样1秒取平均":
    QString("正在采样 %1 / %2 秒，保持轻触").arg(progress["elapsed_s"].toDouble(),0,'f',1).arg(progress["duration_s"].toDouble(),0,'f',1));
  quality_->setText(q.isEmpty()?status_["reason"].toString():
    QString("稳定窗 %1 s / %2 帧；反馈年龄 %3 ms\nTCP漂移 %4 mm；原始dq峰值 %5 rad/s（仅告警）\n%6")
    .arg(q["window_s"].toDouble(),0,'f',2).arg(q["frames"].toInt()).arg(q["age_s"].toDouble()*1000.,0,'f',1)
    .arg(q["tip_drift_m"].toDouble()*1000.,0,'f',3).arg(q["max_speed_rad_s"].toDouble(),0,'f',4).arg(status_["reason"].toString()));
  updateCapture();
  QStringList sys;
  sys << "实际来源："+status_["source"].toString() << "模式："+status_["mode"].toString()
      << "网关阶段："+status_["gateway_phase"].toString()
      << "腕部："+status_["side"].toString()+"_wrist_yaw_link；基准：pelvis"
      << "腰roll/pitch：运行模型固定，非本工具实际测量"
      << "原TCP / 腕部系 (mm)："+vectorText(status_["s_model"].toArray())
      << "模型SHA："+status_["model"].toObject()["sha256"].toString()
      << "手型：index_only近似模型 + 人工刚性确认"
      << "时序：LowState tick推进；本机接收时刻，不宣称硬件时钟同步"
      << "\n跟踪：实际FK - 控制参考FK；不参与TCP拟合";
  const auto track=status_["tracking"].toObject();
  if(track.isEmpty())sys << status_["tracking_reason"].toString();
  else {
    sys << "TCP差 (mm)："+vectorText(track["actual_minus_reference_m"].toArray())
        << QString("TCP差范数 %1 mm；接收配对偏差 %2 ms").arg(track["norm_m"].toDouble()*1000.,0,'f',3).arg(track["skew_s"].toDouble()*1000.,0,'f',2);
    const auto errors=track["joint_actual_minus_reference_rad"].toObject();
    for(auto it=errors.begin();it!=errors.end();++it)sys << it.key()+": "+QString::number(it.value().toDouble()*180./M_PI,'f',3)+" deg";
    sys << QString("另一臂最大保持误差 %1 deg（不直接当作本侧TCP误差）").arg(track["other_arm_max_error_rad"].toDouble()*180./M_PI,0,'f',3);
  }
  sys << "\n采样门限："+QString::fromUtf8(QJsonDocument(status_["limits"].toObject()).toJson())
      << "数据目录："+status_["output_directory"].toString();
  system_->setPlainText(sys.join("\n"));
  const auto rows=status_["samples"].toArray();
  QString signature=status_["session_id"].toString()+status_["selected"].toString();
  for(const auto &v:rows){const auto r=v.toObject(); signature+=r["id"].toString()+r["group"].toString()+QString::number(r["excluded"].toBool());}
  if(signature!=sampleSignature_) {
    const auto oldId=selectedId(); rebuilding_=true; samples_->setRowCount(rows.size());
    int select=-1;
    for(int i=0;i<rows.size();++i) {
      const auto r=rows[i].toObject(); const auto t=r["tracking"].toObject();
      QStringList cells{r["id"].toString().left(8),r["target_id"].toString(),r["group"].toString()=="fit"?"拟合":"验证",
        r["excluded"].toBool()?"排除":"保留",t.isEmpty()?"缺失":millimeters(t,"norm_m"),
        r["joints"].toObject().contains("waist_yaw_joint")?QString::number(r["joints"].toObject()["waist_yaw_joint"].toDouble()*180./M_PI,'f',2):"--"};
      for(int j=0;j<cells.size();++j){auto *item=new QTableWidgetItem(cells[j]); item->setData(Qt::UserRole,r["id"].toString()); if(r["excluded"].toBool())item->setForeground(QColor("#888888")); samples_->setItem(i,j,item);}
      if(r["id"].toString()==(status_["selected"].toString().isEmpty()?oldId:status_["selected"].toString()))select=i;
    }
    if(select>=0) {samples_->selectRow(select); sampleDetails_->setPlainText(QString::fromUtf8(QJsonDocument(rows[select].toObject()).toJson(QJsonDocument::Indented)));}
    rebuilding_=false; sampleSignature_=signature;
  }
  const auto result=status_["result"].toObject();
  if(result.isEmpty()) {solveSummary_->setText("尚未求解或样本已改变；候选结果已失效。"); results_->clear();}
  else {
    const auto old=result["fit_old"].toObject(), corrected=result["fit_new"].toObject();
    solveSummary_->setText("拟合 RMS："+millimeters(old,"rms_m")+" → "+millimeters(corrected,"rms_m")+" mm；最大："+millimeters(corrected,"max_m")+" mm");
    QStringList text;
    text << "旧TCP / 腕部系 mm："+vectorText(result["s_model_m"].toArray())
         << "候选TCP / 腕部系 mm："+vectorText(result["s_candidate_m"].toArray())
         << "变化 mm："+vectorText(result["delta_tcp_m"].toArray())
         << QString("rank %1 / 6；condition %2").arg(result["rank"].toInt()).arg(result["condition"].toDouble(),0,'f',3)
         << "奇异值："+vectorText(result["singular_values"].toArray(),1.)
         << "原模型各轴标准差 mm："+vectorText(old["axis_std_m"].toArray()) << "\n独立验证：";
    const auto validation=result["validation"].toObject();
    for(auto it=validation.begin();it!=validation.end();++it) {
      const auto v=it.value().toObject(); text << it.key()+QString("，n=%1，RMS ").arg(v["n"].toInt())+
        millimeters(v["old"].toObject(),"rms_m")+" -> "+millimeters(v["new"].toObject(),"rms_m")+" mm\n"+v["interpretation"].toString();
    }
    for(const auto &v:result["warnings"].toArray())text << "注意："+v.toString();
    for(const auto &v:result["limitations"].toArray())text << "\n"+v.toString();
    results_->setPlainText(text.join("\n"));
  }
}
}
PLUGINLIB_EXPORT_CLASS(g1_tcp_diagnostics::TcpAPanel,rviz_common::Panel)
