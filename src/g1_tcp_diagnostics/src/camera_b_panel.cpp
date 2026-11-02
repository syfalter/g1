#include "g1_tcp_diagnostics/camera_b_panel.hpp"
#include <QComboBox>
#include <QDateTime>
#include <QFileDialog>
#include <QGridLayout>
#include <QHeaderView>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QStyle>
#include <QTableWidget>
#include <QTimer>
#include <QToolButton>
#include <QWheelEvent>
#include <QUuid>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <cmath>

namespace g1_tcp_diagnostics {
class BoardImage : public QWidget {
public:
  explicit BoardImage(QWidget *parent=nullptr):QWidget(parent) {
    setObjectName("boardImage"); setMinimumSize(240,135);
    setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
  }
  void load(const QString &path) { image_=QImage(path); fit(); }
  void fit() { region_=image_.rect(); update(); }
  void focus(const QRectF &region) { if(!image_.isNull()){region_=region.intersected(image_.rect()); update();} }
  void zoom(double factor) {
    if(image_.isNull() || region_.width()<=0)return;
    const auto center=region_.center();
    const double width=std::min(double(image_.width()),std::max(30.,region_.width()*factor));
    const double height=std::min(double(image_.height()),width*region_.height()/region_.width());
    region_=QRectF(center-QPointF(width/2,height/2),QSizeF(width,height)); bound(); update();
  }
  QSize sizeHint() const override { return QSize(720,405); }
protected:
  void paintEvent(QPaintEvent *) override {
    QPainter painter(this); painter.fillRect(rect(),QColor(29,31,33));
    if(image_.isNull()) {
      painter.setPen(QColor(185,190,193)); painter.drawText(rect(),Qt::AlignCenter,"尚未导入图像"); return;
    }
    const QSize size=region_.size().toSize().scaled(this->size(),Qt::KeepAspectRatio);
    const QRect target(QPoint((width()-size.width())/2,(height()-size.height())/2),size);
    painter.setRenderHint(QPainter::SmoothPixmapTransform); painter.drawImage(target,image_,region_);
  }
  void wheelEvent(QWheelEvent *event) override {zoom(event->angleDelta().y()>0?.8:1.25);event->accept();}
  void mousePressEvent(QMouseEvent *event) override {last_=event->pos();}
  void mouseMoveEvent(QMouseEvent *event) override {
    if(!(event->buttons() & Qt::LeftButton) || image_.isNull())return;
    const auto delta=event->pos()-last_;last_=event->pos();
    const auto rendered=region_.size().toSize().scaled(size(),Qt::KeepAspectRatio);
    region_.translate(-delta.x()*region_.width()/rendered.width(),-delta.y()*region_.height()/rendered.height());
    bound();update();
  }
private:
  void bound() {
    if(region_.left()<0)region_.moveLeft(0);
    if(region_.top()<0)region_.moveTop(0);
    if(region_.right()>image_.width())region_.moveRight(image_.width());
    if(region_.bottom()>image_.height())region_.moveBottom(image_.height());
  }
  QImage image_;
  QRectF region_;
  QPoint last_;
};
namespace {
QLabel *label(const QString &text) {
  auto *out=new QLabel(text); out->setWordWrap(true); out->setTextFormat(Qt::PlainText);
  out->setTextInteractionFlags(Qt::TextSelectableByMouse); return out;
}
QString value(const QJsonObject &m,const QString &key,int places=3) {
  return m.contains(key)?QString::number(m[key].toDouble(),'f',places):"--";
}
}

CameraBPanel::CameraBPanel(QWidget *parent):rviz_common::Panel(parent) {
  setObjectName("cameraBPanel"); setMinimumSize(760,600);
  auto *root=new QVBoxLayout(this); root->setContentsMargins(12,10,12,10); root->setSpacing(8);
  auto *title=label("实验 B · RGB / 深度一致性");
  auto font=title->font(); font.setPointSize(16); font.setBold(true); title->setFont(font); root->addWidget(title);
  connection_=label("等待离线后端"); connection_->setObjectName("bConnection"); root->addWidget(connection_);
  board_=label("charuco-200 · 9 × 12 · 方格 15 mm · 标记 11.25 mm · 5 × 5 编码"); root->addWidget(board_);
  auto *toolbar=new QGridLayout; toolbar->setHorizontalSpacing(6);
  auto button=[&](QString text,QString name,QStyle::StandardPixmap icon,int col) {
    auto *b=new QPushButton(text); b->setObjectName(name); b->setIcon(style()->standardIcon(icon));
    b->setMinimumHeight(32); toolbar->addWidget(b,0,col); actions_.append(b); return b;
  };
  auto *fresh=button("新会话","bNew",QStyle::SP_FileIcon,0);
  auto *photo=button("导入 RGB","bImportRgb",QStyle::SP_DirOpenIcon,1);
  auto *bundle=button("导入 RGB-D","bImportBundle",QStyle::SP_DirOpenIcon,2);
  auto *load=button("加载","bLoad",QStyle::SP_DialogOpenButton,3);
  auto *save=button("保存","bSave",QStyle::SP_DialogSaveButton,4);
  auto *recompute=button("复算","bRecompute",QStyle::SP_BrowserReload,5);
  auto *exportButton=button("导出","bExport",QStyle::SP_ArrowDown,6);
  root->addLayout(toolbar);
  auto *form=new QHBoxLayout;
  form->addWidget(label("用途")); group_=new QComboBox; group_->setObjectName("bGroup");
  group_->addItem("诊断组（fit）","fit"); group_->addItem("独立验证组","validation"); form->addWidget(group_);
  note_=new QLineEdit; note_->setObjectName("bNote"); note_->setMaxLength(500); note_->setPlaceholderText("样本备注"); form->addWidget(note_,1);
  root->addLayout(form);
  liveBar_=new QWidget; auto *liveLayout=new QHBoxLayout(liveBar_); liveLayout->setContentsMargins(0,0,0,0);
  checkCamera_=new QPushButton("连接检查"); checkCamera_->setObjectName("bCheckCamera");
  checkCamera_->setIcon(style()->standardIcon(QStyle::SP_BrowserReload)); liveLayout->addWidget(checkCamera_);
  snapshot_=new QPushButton("更新快照"); snapshot_->setObjectName("bSnapshot");
  snapshot_->setIcon(style()->standardIcon(QStyle::SP_FileDialogContentsView)); liveLayout->addWidget(snapshot_);
  snapshot_->setToolTip("请求一张新的RGB-D，仅预览、不参加正式统计；不会控制机器人");
  capture_=new QPushButton("确认停稳并采样"); capture_->setObjectName("bCapture"); capture_->setMinimumHeight(34);
  capture_->setIcon(style()->standardIcon(QStyle::SP_DialogApplyButton)); liveLayout->addWidget(capture_);
  capture_->setToolTip("点击即确认板和相机已停稳；取帧期间保持静止。只采图，不发运动命令。");
  cameraStatus_=label("尚未检查相机"); liveLayout->addWidget(cameraStatus_,1); root->addWidget(liveBar_);liveBar_->hide();
  checkCamera_->setEnabled(false);snapshot_->setEnabled(false);capture_->setEnabled(false);
  connect(checkCamera_,&QPushButton::clicked,this,[this]{send("check_camera");});
  connect(snapshot_,&QPushButton::clicked,this,[this]{send("snapshot");});
  connect(capture_,&QPushButton::clicked,this,[this]{send("capture",{{"group",group_->currentData().toString()},
    {"note",note_->text()},{"stationary_confirmed",true}});});
  auto *content=new QSplitter(Qt::Horizontal); root->addWidget(content,1);
  auto *visual=new QWidget; auto *visualLayout=new QVBoxLayout(visual); visualLayout->setContentsMargins(0,0,0,0);
  view_=new QComboBox; view_->setObjectName("bView");
  view_->addItem("角点 / 板坐标轴","overlay"); view_->addItem("原始 RGB","rgb");
  view_->addItem("深度预览 · 0–3 m","depth"); view_->addItem("板面误差 · 蓝 −20 / 白 0 / 红 +20 mm","heatmap");
  view_->addItem("距离 / 倾角趋势","trend");
  auto *imageTools=new QHBoxLayout; imageTools->addWidget(view_,1);
  auto imageTool=[&](const QString &theme,const QString &tip,const QString &name,QStyle::StandardPixmap fallback){
    auto *b=new QToolButton; b->setObjectName(name); b->setIcon(QIcon::fromTheme(theme,style()->standardIcon(fallback)));
    b->setToolTip(tip); b->setFixedSize(30,30);imageTools->addWidget(b);return b;
  };
  auto *focus=imageTool("zoom-select","放大到板面","bFocus",QStyle::SP_FileDialogContentsView);
  auto *zoomIn=imageTool("zoom-in","放大","bZoomIn",QStyle::SP_ArrowUp);
  auto *zoomOut=imageTool("zoom-out","缩小","bZoomOut",QStyle::SP_ArrowDown);
  auto *fit=imageTool("zoom-fit-best","显示完整图像","bFit",QStyle::SP_TitleBarMaxButton);
  visualLayout->addLayout(imageTools);
  image_=new BoardImage; visualLayout->addWidget(image_,1);
  connect(zoomIn,&QToolButton::clicked,this,[this]{image_->zoom(.8);});
  connect(zoomOut,&QToolButton::clicked,this,[this]{image_->zoom(1.25);});
  connect(fit,&QToolButton::clicked,this,[this]{image_->fit();});
  connect(focus,&QToolButton::clicked,this,[this]{
    if(view_->currentData().toString()=="trend")return;
    for(const auto &item:status_["samples"].toArray()) {
      const auto s=item.toObject(); if(s["id"]!=status_["selected"])continue;
      const auto points=s["result"].toObject()["detection"].toObject()["image_points_px"].toArray();
      if(points.isEmpty())return;
      QRectF area;
      for(const auto &v:points) {const auto p=v.toArray();const QRectF dot(p[0].toDouble(),p[1].toDouble(),1,1);area=area.isNull()?dot:area.united(dot);}
      image_->focus(area.adjusted(-35,-35,35,35));
    }
  });
  caption_=label("无图像"); caption_->setObjectName("bCaption"); visualLayout->addWidget(caption_); content->addWidget(visual);
  auto *side=new QWidget; side->setMinimumWidth(235); auto *sideLayout=new QVBoxLayout(side);
  metrics_=label("尚无 RGB-D 指标"); metrics_->setObjectName("bMetrics"); sideLayout->addWidget(metrics_);
  details_=new QPlainTextEdit; details_->setReadOnly(true); details_->setObjectName("bDetails");
  details_->setLineWrapMode(QPlainTextEdit::WidgetWidth); sideLayout->addWidget(details_,1);
  auto *sideScroll=new QScrollArea; sideScroll->setWidgetResizable(true); sideScroll->setMinimumWidth(255); sideScroll->setWidget(side);
  content->addWidget(sideScroll); content->setStretchFactor(0,3); content->setStretchFactor(1,1); content->setSizes({780,280});
  samples_=new QTableWidget(0,8); samples_->setObjectName("bSamples");
  samples_->setHorizontalHeaderLabels({"样本","用途","状态","距离 m","倾角 °","RMS mm","P95 mm","来源"});
  samples_->setSelectionBehavior(QAbstractItemView::SelectRows); samples_->setSelectionMode(QAbstractItemView::SingleSelection);
  samples_->setEditTriggers(QAbstractItemView::NoEditTriggers); samples_->verticalHeader()->hide();
  samples_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents); samples_->horizontalHeader()->setStretchLastSection(true);
  samples_->setMinimumHeight(115); samples_->setMaximumHeight(195); root->addWidget(samples_);
  auto *row=new QHBoxLayout;
  exclude_=new QPushButton("排除 / 恢复"); exclude_->setObjectName("bExclude"); row->addWidget(exclude_);
  changeGroup_=new QPushButton("诊断 / 验证"); changeGroup_->setObjectName("bChangeGroup"); row->addWidget(changeGroup_);
  row->addStretch(); root->addLayout(row);
  message_=label("离线模式"); message_->setObjectName("bMessage"); message_->setMaximumHeight(70); root->addWidget(message_);
  connect(fresh,&QPushButton::clicked,this,[this]{
    if(QMessageBox::question(this,"新建 B 会话","保存当前会话并开始新会话？原始文件保留。") == QMessageBox::Yes)send("new");
  });
  connect(photo,&QPushButton::clicked,this,[this]{
    const auto path=QFileDialog::getOpenFileName(this,"导入历史 RGB","/home/syf/g1_upper_body_ws/data","Images (*.png *.jpg *.jpeg)");
    if(!path.isEmpty())send("import",{{"path",path},{"group",group_->currentData().toString()},{"note",note_->text()}});
  });
  connect(bundle,&QPushButton::clicked,this,[this]{
    const auto path=QFileDialog::getExistingDirectory(this,"导入已有配对 RGB-D 归档","/home/syf/g1_upper_body_ws/data");
    if(!path.isEmpty())send("import",{{"path",path},{"group",group_->currentData().toString()},{"note",note_->text()}});
  });
  connect(load,&QPushButton::clicked,this,[this]{
    const auto path=QFileDialog::getOpenFileName(this,"加载实验 B 快照","/home/syf/g1_upper_body_ws/data/camera_b","JSON (*.json)");
    if(!path.isEmpty())send("load",{{"path",path}});
  });
  connect(save,&QPushButton::clicked,this,[this]{send("save");});
  connect(exportButton,&QPushButton::clicked,this,[this]{send("export");});
  connect(recompute,&QPushButton::clicked,this,[this]{send("recompute");});
  connect(exclude_,&QPushButton::clicked,this,[this]{send("exclude",{{"sample_id",selectedId()}});});
  connect(changeGroup_,&QPushButton::clicked,this,[this]{send("group",{{"sample_id",selectedId()}});});
  connect(view_,qOverload<int>(&QComboBox::currentIndexChanged),this,[this]{showSelected();});
  connect(samples_,&QTableWidget::itemSelectionChanged,this,[this]{
    if(!rebuilding_ && !selectedId().isEmpty() && selectedId()!=status_["selected"].toString())send("select",{{"sample_id",selectedId()}});
  });
  auto *timer=new QTimer(this); timer->setInterval(150); connect(timer,&QTimer::timeout,this,&CameraBPanel::refresh); timer->start();
  for(auto *b:actions_)b->setEnabled(false); exclude_->setEnabled(false); changeGroup_->setEnabled(false);
}

void CameraBPanel::onInitialize() {
  auto abstraction=getDisplayContext()->getRosNodeAbstraction().lock();
  if(abstraction)attachNode(abstraction->get_raw_node());
}
void CameraBPanel::attachNode(rclcpp::Node::SharedPtr node) {
  if(qEnvironmentVariable("ROS_DOMAIN_ID")!="96" || qEnvironmentVariable("ROS_LOCALHOST_ONLY")!="1") {
    connection_->setText("B界面仅允许 localhost / domain 96；相机由独立RPC客户端访问");return;
  }
  node_=node;
  pub_=node_->create_publisher<std_msgs::msg::String>("/g1/camera_b/request",10);
  sub_=node_->create_subscription<std_msgs::msg::String>("/g1/camera_b/status",rclcpp::QoS(1).transient_local(),
    [this](std_msgs::msg::String::SharedPtr message){
      const auto doc=QJsonDocument::fromJson(QByteArray::fromStdString(message->data)); if(doc.isObject())setStatus(doc.object());
    });
}
void CameraBPanel::setStatus(const QJsonObject &status) {
  const auto mode=status["mode"].toString();
  if(status["protocol"].toString()!="camera_b_v1" || (mode!="offline" && mode!="live"))return;
  std::lock_guard<std::mutex> lock(mutex_); pending_=status; dirty_=true; receivedMs_=QDateTime::currentMSecsSinceEpoch();
}
QString CameraBPanel::selectedId() const {
  const auto items=samples_->selectedItems(); return items.isEmpty()?QString():samples_->item(items[0]->row(),0)->data(Qt::UserRole).toString();
}
void CameraBPanel::send(const QString &operation,QJsonObject fields) {
  const auto now=QDateTime::currentMSecsSinceEpoch();
  qint64 received;{std::lock_guard<std::mutex> lock(mutex_);received=receivedMs_;}
  if(now-received>1800 || status_["busy"].toBool() || !pendingRequest_.isEmpty())return;
  if((operation=="capture" || operation=="snapshot" || operation=="check_camera") && status_["mode"].toString()!="live")return;
  pendingRequest_=QUuid::createUuid().toString(QUuid::WithoutBraces); sentMs_=now;
  fields["protocol"]="camera_b_v1"; fields["op"]=operation; fields["request_id"]=pendingRequest_;
  fields["session_id"]=status_["session_id"]; fields["stamp"]=now/1000.;
  Q_EMIT requestBuilt(fields);
  if(pub_) {std_msgs::msg::String msg; msg.data=QJsonDocument(fields).toJson(QJsonDocument::Compact).toStdString(); pub_->publish(msg);}
  message_->setText("正在处理："+operation); refresh();
}
void CameraBPanel::refresh() {
  bool changed=false;qint64 received;
  {std::lock_guard<std::mutex> lock(mutex_); received=receivedMs_;if(dirty_){status_=pending_; dirty_=false; changed=true;}}
  const auto now=QDateTime::currentMSecsSinceEpoch();
  const bool fresh=now-received<1800 && std::abs(now/1000.-status_["stamp"].toDouble())<1.8;
  if(status_["last_request_id"].toString()==pendingRequest_)pendingRequest_.clear();
  if(!pendingRequest_.isEmpty() && now-sentMs_>300000) {pendingRequest_.clear(); message_->setText("请求结果未确认；请检查后端状态，不自动重试。");}
  const bool enabled=fresh && !status_["busy"].toBool() && pendingRequest_.isEmpty();
  for(auto *b:actions_)b->setEnabled(enabled);
  group_->setEnabled(enabled); note_->setEnabled(enabled); samples_->setEnabled(enabled);
  exclude_->setEnabled(enabled && !selectedId().isEmpty()); changeGroup_->setEnabled(enabled && !selectedId().isEmpty());
  const bool live=status_["mode"].toString()=="live";
  const auto camera=status_["camera"].toObject();
  liveBar_->setVisible(live);checkCamera_->setEnabled(enabled && live);
  snapshot_->setEnabled(enabled && live && camera["ready"].toBool());capture_->setEnabled(snapshot_->isEnabled());
  cameraStatus_->setText(camera["message"].toString());
  int formal=0,previews=0;
  for(const auto &item:status_["samples"].toArray()) {
    const auto s=item.toObject();
    if(s["group"].toString()=="preview")++previews;
    else if(s["result"].toObject()["valid"].toBool() && !s["excluded"].toBool())++formal;
    if(s["id"].toString()==selectedId() && s["group"].toString()=="preview")changeGroup_->setEnabled(false);
  }
  connection_->setText(fresh?(live?QString("在线相机 · "):QString("离线 · "))+"localhost / domain 96 · "+status_["session_id"].toString()+
    QString(" · 有效正式 %1 / 预览 %2").arg(formal).arg(previews):"B后端未连接 / 状态过期");
  window()->setWindowTitle(live?"实验 B · RGB / 深度一致性 · 在线相机（无运动控制）":"实验 B · RGB / 深度一致性 · 离线");
  if(!changed)return;
  const auto layout=status_["layout"].toObject();
  board_->setText("charuco-200 · 标签格长 15 mm / 标记 11.25 mm · "+
    (layout.isEmpty()?QString("9 × 12 布局待图像核对"):QString("已匹配 %1 × %2 · 5×5 编码布局已核对").arg(layout["cols"].toInt()).arg(layout["rows"].toInt())));
  if(pendingRequest_.isEmpty() || status_["busy"].toBool())message_->setText(status_["message"].toString());
  const auto array=status_["samples"].toArray();
  const auto signature=QString::fromUtf8(QJsonDocument(array).toJson(QJsonDocument::Compact))+status_["selected"].toString();
  if(signature!=signature_) {
    rebuilding_=true; samples_->setRowCount(array.size());
    for(int row=0;row<array.size();++row) {
      const auto s=array[row].toObject(), result=s["result"].toObject(), m=result["metrics"].toObject();
      const auto id=s["id"].toString();
      QStringList cells{id.section('_',0,1),s["group"].toString()=="fit"?"诊断":s["group"].toString()=="preview"?"预览":"验证",
        s["excluded"].toBool()?"排除":result["valid"].toBool()?"有效":"仅预览 / 无效",
        value(m,"board_center_z_m"),value(m,"tilt_deg",1),value(m,"rms_mm"),value(m,"p95_absolute_mm"),
        s["source_kind"].toString().startsWith("live_")?"现场 RGB-D":s["source_kind"].toString()=="paired_archive"?"历史 RGB-D":s["source_kind"].toString()=="rgb_only"?"仅 RGB":"合成测试"};
      for(int col=0;col<cells.size();++col) {auto *item=new QTableWidgetItem(cells[col]); item->setToolTip(result["reason"].toString()); item->setData(Qt::UserRole,id); samples_->setItem(row,col,item);}
      if(id==status_["selected"].toString())samples_->selectRow(row);
    }
    rebuilding_=false; signature_=signature;
  }
  showSelected();
}
void CameraBPanel::showSelected() {
  const auto views=status_["views"].toObject(); const auto key=view_->currentData().toString();
  const auto path=views[key].toString(); if(path!=imagePath_) {image_->load(path); imagePath_=path;}
  QJsonObject s;
  for(const auto &item:status_["samples"].toArray())if(item.toObject()["id"]==status_["selected"])s=item.toObject();
  if(s.isEmpty()) {metrics_->setText("尚无样本"); caption_->setText("无图像"); details_->clear(); return;}
  const auto result=s["result"].toObject(), m=result["metrics"].toObject(), detection=result["detection"].toObject();
  const auto acquisition=s["acquisition"].toObject();
  caption_->setText(path.isEmpty()?"该样本没有所选视图":s["id"].toString()+" · "+
    (s["source_kind"].toString().startsWith("live_")?"现场快照 · 请求于 "+acquisition["requested_at"].toString():QString("历史图像")));
  metrics_->setText(QString("角点 %1 · marker %2\n\n重投影 %3 px\n板中心 Z %4 m\n倾角 %5°\n\n有符号均值 %6 mm\n绝对误差中位数 %7 mm\nRMS %8 mm\n绝对 P95 %9 mm\n法向差 %10°\n深度有效率 %11\n\n%12")
    .arg(detection["corner_count"].toInt()).arg(detection["marker_count"].toInt())
    .arg(value(m,"reprojection_rms_px")).arg(value(m,"board_center_z_m")).arg(value(m,"tilt_deg",2))
    .arg(value(m,"signed_mean_mm")).arg(value(m,"median_absolute_mm")).arg(value(m,"rms_mm"))
    .arg(value(m,"p95_absolute_mm")).arg(value(m,"normal_difference_deg",2))
    .arg(m.contains("valid_fraction")?QString::number(m["valid_fraction"].toDouble()*100,'f',1)+"%":"--")
    .arg(result["valid"].toBool()?QString("RGB-D 指标可用"):result["reason"].toString()));
  details_->setPlainText(QString::fromUtf8(QJsonDocument(s).toJson(QJsonDocument::Indented)));
}
}
PLUGINLIB_EXPORT_CLASS(g1_tcp_diagnostics::CameraBPanel,rviz_common::Panel)
