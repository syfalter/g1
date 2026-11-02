#include "g1_tcp_diagnostics/camera_c_panel.hpp"
#include <QComboBox>
#include <QDateTime>
#include <QFileDialog>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGridLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSplitter>
#include <QStyle>
#include <QTableWidget>
#include <QTimer>
#include <QToolButton>
#include <QUuid>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace g1_tcp_diagnostics {
namespace {
QLabel *text(const QString &s) {
  auto *out=new QLabel(s); out->setWordWrap(true); out->setTextFormat(Qt::PlainText);
  out->setTextInteractionFlags(Qt::TextSelectableByMouse); return out;
}
}
CameraCPanel::CameraCPanel(QWidget *parent):rviz_common::Panel(parent) {
  setMinimumSize(760,600);
  auto *root=new QVBoxLayout(this);
  auto *title=text("实验 C · 相机 / 腕部运动学一致性");
  auto font=title->font();font.setPointSize(16);font.setBold(true);title->setFont(font);root->addWidget(title);
  connection_=text("等待 C 后端");connection_->setObjectName("cConnection");root->addWidget(connection_);
  auto *bar=new QGridLayout;
  auto button=[&](QString name,int column,QStyle::StandardPixmap icon) {
    auto *b=new QPushButton(name); b->setIcon(style()->standardIcon(icon));b->setMinimumHeight(32);
    bar->addWidget(b,0,column);actions_.append(b);return b;
  };
  auto *fresh=button("新安装",0,QStyle::SP_FileIcon);
  auto *load=button("加载",1,QStyle::SP_DialogOpenButton);
  auto *import=button("导入归档",2,QStyle::SP_DirOpenIcon);
  auto *save=button("保存",3,QStyle::SP_DialogSaveButton);
  auto *recompute=button("复算",4,QStyle::SP_BrowserReload);
  auto *solve=button("求解候选",5,QStyle::SP_DialogApplyButton);
  auto *exportButton=button("导出",6,QStyle::SP_ArrowDown);root->addLayout(bar);
  auto *form=new QHBoxLayout;
  side_=new QComboBox;side_->addItem("左腕","left");side_->addItem("右腕","right");form->addWidget(side_);
  mount_=new QLineEdit("offline-unconfirmed");mount_->setPlaceholderText("安装 ID");mount_->setMaxLength(100);form->addWidget(mount_);
  group_=new QComboBox;group_->addItem("诊断 fit","fit");group_->addItem("独立验证","validation");group_->addItem("预览","preview");form->addWidget(group_);
  note_=new QLineEdit;note_->setPlaceholderText("样本备注");note_->setMaxLength(500);form->addWidget(note_,1);root->addLayout(form);
  auto *live=new QHBoxLayout;
  check_=new QPushButton("连接检查");check_->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));live->addWidget(check_);
  snapshot_=new QPushButton("更新 RGB 快照");snapshot_->setIcon(style()->standardIcon(QStyle::SP_FileDialogContentsView));live->addWidget(snapshot_);
  capture_=new QPushButton("确认停稳采样");capture_->setEnabled(false);live->addWidget(capture_);
  auto *gap=text("新流取帧 + 全窗口静止检查 · 候选不自动应用");live->addWidget(gap,1);root->addLayout(live);
  auto *split=new QSplitter;root->addWidget(split,1);
  auto *visual=new QWidget;auto *vl=new QVBoxLayout(visual);vl->setContentsMargins(0,0,0,0);
  auto *tools=new QHBoxLayout;view_=new QComboBox;view_->addItem("角点","overlay");view_->addItem("原始 RGB","rgb");tools->addWidget(view_,1);
  auto tool=[&](QString theme,QString tip,QStyle::StandardPixmap fallback) {
    auto *b=new QToolButton;b->setIcon(QIcon::fromTheme(theme,style()->standardIcon(fallback)));b->setToolTip(tip);b->setFixedSize(30,30);tools->addWidget(b);return b;
  };
  auto *focus=tool("zoom-select","聚焦板面",QStyle::SP_FileDialogContentsView);
  auto *zoomIn=tool("zoom-in","放大",QStyle::SP_ArrowUp);
  auto *zoomOut=tool("zoom-out","缩小",QStyle::SP_ArrowDown);
  auto *fit=tool("zoom-fit-best","完整图像",QStyle::SP_TitleBarMaxButton);vl->addLayout(tools);
  scene_=new QGraphicsScene(this);image_=new QGraphicsView(scene_);image_->setObjectName("cImage");
  image_->setBackgroundBrush(QColor(29,31,33));image_->setDragMode(QGraphicsView::ScrollHandDrag);
  image_->setRenderHint(QPainter::SmoothPixmapTransform);image_->setMinimumSize(300,180);vl->addWidget(image_,1);
  caption_=text("尚无图像");vl->addWidget(caption_);split->addWidget(visual);
  details_=new QPlainTextEdit;details_->setReadOnly(true);details_->setMinimumWidth(240);details_->setMaximumWidth(460);
  split->addWidget(details_);split->setStretchFactor(0,3);split->setStretchFactor(1,1);split->setSizes({850,340});
  samples_=new QTableWidget(0,7);samples_->setObjectName("cSamples");
  samples_->setHorizontalHeaderLabels({"样本","用途","RGB","角点","PnP px","配对","排除"});
  samples_->setSelectionBehavior(QAbstractItemView::SelectRows);samples_->setSelectionMode(QAbstractItemView::SingleSelection);
  samples_->setEditTriggers(QAbstractItemView::NoEditTriggers);samples_->verticalHeader()->hide();
  samples_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);samples_->horizontalHeader()->setStretchLastSection(true);
  samples_->horizontalHeader()->setMinimumSectionSize(85);
  samples_->setMinimumHeight(120);samples_->setMaximumHeight(200);root->addWidget(samples_);
  auto *audit=new QHBoxLayout;auto *exclude=new QPushButton("排除 / 恢复");auto *regroup=new QPushButton("设为所选用途");
  audit->addWidget(exclude);audit->addWidget(regroup);audit->addStretch();root->addLayout(audit);actions_<<exclude<<regroup;
  message_=text("启动不采图、不 ARM、不改手型；关闭仅退出 C");message_->setMaximumHeight(85);root->addWidget(message_);
  connect(fresh,&QPushButton::clicked,this,[this]{
    if(mount_->text().trimmed().isEmpty())return;
    if(QMessageBox::question(this,"新安装","保存当前会话并新建所选腕侧 / 安装 ID？不会改变机器人状态。") == QMessageBox::Yes)
      send("new",{{"side",side_->currentData().toString()},{"mount_id",mount_->text().trimmed()}});
  });
  connect(load,&QPushButton::clicked,this,[this]{auto p=QFileDialog::getOpenFileName(this,"加载 C 会话",{},"JSON (*.json)");if(!p.isEmpty())send("load",{{"path",p}});});
  connect(import,&QPushButton::clicked,this,[this]{auto p=QFileDialog::getExistingDirectory(this,"选择完整 RGB-D 归档目录");if(!p.isEmpty())send("import",{{"path",p},{"group",group_->currentData().toString()},{"note",note_->text()}});});
  connect(save,&QPushButton::clicked,this,[this]{send("save");});
  connect(recompute,&QPushButton::clicked,this,[this]{send("recompute");});
  connect(solve,&QPushButton::clicked,this,[this]{send("solve");});
  connect(exportButton,&QPushButton::clicked,this,[this]{send("export");});
  connect(check_,&QPushButton::clicked,this,[this]{send("check_camera");});
  connect(snapshot_,&QPushButton::clicked,this,[this]{send("snapshot",{{"note",note_->text()}});});
  connect(capture_,&QPushButton::clicked,this,[this]{
    if(group_->currentData().toString()=="preview") {
      QMessageBox::information(this,"采样用途","请选择诊断 fit 或独立验证；预览使用更新 RGB 快照。");return;
    }
    if(QMessageBox::question(this,"本次停稳确认",
      "确认板安装牢固，且手臂和板从现在起直到本次采样完成都保持静止？允许在导纳状态采集，不发送运动命令。\n原相机服务会重开同一图像流并预热；新帧和实际姿态检查均通过后才进入求解。") == QMessageBox::Yes)
      send("capture",{{"note",note_->text()},{"group",group_->currentData().toString()},
                      {"stationary_confirmed",true},{"rigid_mount_confirmed",true}});
  });
  connect(view_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this]{showSelected();});
  connect(zoomIn,&QToolButton::clicked,this,[this]{image_->scale(1.25,1.25);});
  connect(zoomOut,&QToolButton::clicked,this,[this]{image_->scale(.8,.8);});
  connect(fit,&QToolButton::clicked,this,[this]{image_->fitInView(scene_->sceneRect(),Qt::KeepAspectRatio);});
  connect(focus,&QToolButton::clicked,this,[this]{
    for(const auto &v:status_["samples"].toArray()) {auto s=v.toObject();if(s["id"]!=status_["selected"])continue;
      QRectF area;for(const auto &p:s["result"].toObject()["detection"].toObject()["image_points_px"].toArray()) {
        auto a=p.toArray();QRectF dot(a[0].toDouble(),a[1].toDouble(),1,1);area=area.isNull()?dot:area.united(dot);
      }if(!area.isNull())image_->fitInView(area.adjusted(-35,-35,35,35),Qt::KeepAspectRatio);
    }
  });
  connect(samples_,&QTableWidget::itemSelectionChanged,this,[this]{
    if(rebuilding_ || samples_->currentRow()<0)return;
    const auto id=samples_->item(samples_->currentRow(),0)->data(Qt::UserRole).toString();
    if(id!=status_["selected"].toString())send("select",{{"sample_id",id}});
  });
  connect(exclude,&QPushButton::clicked,this,[this]{bool ok=false;auto reason=QInputDialog::getText(this,"排除审计","原因",QLineEdit::Normal,{},&ok);
    if(ok&&!reason.trimmed().isEmpty())send("exclude",{{"sample_id",status_["selected"]},{"reason",reason}});
  });
  connect(regroup,&QPushButton::clicked,this,[this]{
    if(QMessageBox::question(this,"用途审计","明确改变所选样本用途？不可依据留出表现挑样本。") == QMessageBox::Yes)
      send("group",{{"sample_id",status_["selected"]},{"group",group_->currentData().toString()}});
  });
  auto *timer=new QTimer(this);connect(timer,&QTimer::timeout,this,&CameraCPanel::refresh);timer->start(100);
}
void CameraCPanel::onInitialize(){attachNode(getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node());}
void CameraCPanel::attachNode(rclcpp::Node::SharedPtr node) {
  node_=node;pub_=node_->create_publisher<std_msgs::msg::String>("/g1/camera_c/request",10);
  sub_=node_->create_subscription<std_msgs::msg::String>("/g1/camera_c/status",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr msg){
    const auto data=QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();
    if(data["protocol"].toString()!="camera_c_v1")return;
    std::lock_guard<std::mutex> lock(mutex_);pending_=data;dirty_=true;
  });
}
void CameraCPanel::send(const QString &op,QJsonObject fields) {
  if(!pub_||!pendingRequest_.isEmpty()||status_["busy"].toBool()||QDateTime::currentMSecsSinceEpoch()-receivedMs_>2000)return;
  pendingRequest_=QUuid::createUuid().toString(QUuid::WithoutBraces);sentMs_=QDateTime::currentMSecsSinceEpoch();
  fields["op"]=op;fields["protocol"]="camera_c_v1";fields["request_id"]=pendingRequest_;
  fields["session_id"]=status_["session_id"];fields["stamp"]=sentMs_/1000.;
  std_msgs::msg::String msg;msg.data=QJsonDocument(fields).toJson(QJsonDocument::Compact).toStdString();pub_->publish(msg);
}
void CameraCPanel::refresh() {
  bool changed=false;
  {std::lock_guard<std::mutex> lock(mutex_);if(dirty_){status_=pending_;dirty_=false;changed=true;receivedMs_=QDateTime::currentMSecsSinceEpoch();}}
  const auto now=QDateTime::currentMSecsSinceEpoch();
  const bool stale=now-receivedMs_>2000 || qAbs(now-qint64(status_["stamp"].toDouble()*1000))>3000;
  auto ack=status_["ack"].toObject();
  if(ack["request_id"].toString()==pendingRequest_&&ack["state"].toString()!="accepted")pendingRequest_.clear();
  if(!pendingRequest_.isEmpty()&&now-sentMs_>10000&&!status_["busy"].toBool()) {
    message_->setText("请求结果未确认；未自动重试");pendingRequest_.clear();
  }
  const bool enabled=!stale&&!status_["busy"].toBool()&&pendingRequest_.isEmpty();
  for(auto *b:actions_)b->setEnabled(enabled);
  check_->setEnabled(enabled&&status_["mode"].toString()=="live");snapshot_->setEnabled(check_->isEnabled());
  capture_->setEnabled(enabled&&status_["diagnostic_capture_enabled"].toBool());
  if(!changed&&!stale)return;
  const auto actual=status_["actual"].toObject();const auto owner=status_["owner"].toObject();
  connection_->setText(QString("%1 | %2 | 腕侧 %3 | 安装 %4\n相机 %5 | actual %6 帧 / %7 来源 | 模型 %8 | owner %9")
    .arg(stale?"后端过期":"C 已连接",status_["mode"].toString(),status_["side"].toString(),status_["mount_id"].toString(),
      status_["camera"].toObject()["message"].toString()).arg(actual["count"].toInt()).arg(actual["publishers"].toInt())
    .arg(status_["model"].toObject()["received"].toBool()?"已接收（未归档配对）":"未接收",owner["available"].toBool()?"新鲜":"不可用"));
  message_->setText(status_["message"].toString());
  const auto rows=status_["samples"].toArray();
  const QString signature=QString::fromUtf8(QJsonDocument(rows).toJson(QJsonDocument::Compact))+status_["selected"].toString();
  if(signature!=signature_) {
    signature_=signature;rebuilding_=true;samples_->setRowCount(rows.size());
    for(int i=0;i<rows.size();++i) {
      auto s=rows[i].toObject();auto r=s["result"].toObject();auto pose=r["pose"].toObject();
      QStringList cells={s["id"].toString().left(11),s["group"].toString(),r["rgb_valid"].toBool()?"可计算":"无效",
        QString::number(r["detection"].toObject()["corner_count"].toInt()),pose.contains("reprojection_rms_px")?QString::number(pose["reprojection_rms_px"].toDouble(),'f',3):"--",
        r["pairing_valid"].toBool()?"合格":"拒绝",s["excluded"].toBool()?"是":"否"};
      for(int j=0;j<cells.size();++j){auto *item=new QTableWidgetItem(cells[j]);item->setData(Qt::UserRole,s["id"].toString());samples_->setItem(i,j,item);}
      if(s["id"]==status_["selected"])samples_->selectRow(i);
    }rebuilding_=false;showSelected();
  }
}
void CameraCPanel::showSelected() {
  for(const auto &v:status_["samples"].toArray()) {
    const auto s=v.toObject();if(s["id"]!=status_["selected"])continue;
    const auto path=status_["root"].toString()+"/"+s["views"].toObject()[view_->currentData().toString()].toString();
    if(path!=imagePath_) {imagePath_=path;scene_->clear();QPixmap pix(path);scene_->addPixmap(pix);scene_->setSceneRect(pix.rect());image_->fitInView(scene_->sceneRect(),Qt::KeepAspectRatio);}
    caption_->setText(s["id"].toString()+" | "+s["group"].toString()+" | "+s["note"].toString());
    QJsonObject detail{{"result",s["result"]},{"source",s["source"]},{"acquisition_audit",s["source_audit"]},
                       {"candidate",status_["candidate"]},{"saved",status_["last_snapshot"]}};
    details_->setPlainText(QString::fromUtf8(QJsonDocument(detail).toJson(QJsonDocument::Indented)));return;
  }
  imagePath_.clear();scene_->clear();caption_->setText("尚无图像");details_->clear();
}
}
PLUGINLIB_EXPORT_CLASS(g1_tcp_diagnostics::CameraCPanel,rviz_common::Panel)
