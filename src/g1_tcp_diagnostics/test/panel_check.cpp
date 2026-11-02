#include "g1_tcp_diagnostics/tcp_a_panel.hpp"
#include "g1_tcp_diagnostics/camera_b_panel.hpp"
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDateTime>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QPushButton>
#include <QSignalSpy>
#include <QTabWidget>
#include <QTest>
#include <QTableWidget>
#include <QToolButton>
#include <iostream>

int main(int argc,char **argv) {
  QApplication app(argc,argv);
  if(argc<3){std::cerr<<"usage: panel_check STATUS_JSON OUTPUT_DIR\n";return 2;}
  QFile input(argv[1]); if(!input.open(QIODevice::ReadOnly))return 3;
  auto state=QJsonDocument::fromJson(input.readAll()).object();
  QDir().mkpath(argv[2]);
  if(state["protocol"].toString()=="camera_b_v1") {
    g1_tcp_diagnostics::CameraBPanel panel;
    QSignalSpy spy(&panel,&g1_tcp_diagnostics::CameraBPanel::requestBuilt);
    panel.show();
    auto *save=panel.findChild<QPushButton*>("bSave");
    if(save->isEnabled())return 20;
    for(auto size:{QSize(1320,920),QSize(820,700)}) {
      panel.resize(size);
      auto *view=panel.findChild<QComboBox*>("bView");
      for(int i=0;i<view->count();++i) {
        state["stamp"]=QDateTime::currentMSecsSinceEpoch()/1000.; panel.setStatus(state); view->setCurrentIndex(i); QTest::qWait(200);
        if(!save->isEnabled() || save->mapTo(&panel,QPoint(save->width(),save->height())).x()>panel.width())return 21;
        auto *image=panel.findChild<QWidget*>("boardImage");
        if(!image || image->width()<200 || image->height()<100)return 22;
        panel.grab().save(QString(argv[2])+QString("/camera_b_%1_%2.png").arg(size.width()).arg(i));
      }
    }
    panel.findChild<QComboBox*>("bView")->setCurrentIndex(0);
    QTest::mouseClick(panel.findChild<QToolButton*>("bFocus"),Qt::LeftButton);QTest::qWait(200);
    panel.grab().save(QString(argv[2])+"/camera_b_focus.png");
    QTest::mouseClick(panel.findChild<QToolButton*>("bZoomIn"),Qt::LeftButton);
    QTest::mouseClick(panel.findChild<QToolButton*>("bZoomOut"),Qt::LeftButton);
    QTest::mouseClick(panel.findChild<QToolButton*>("bFit"),Qt::LeftButton);
    auto *group=panel.findChild<QComboBox*>("bGroup"); group->setCurrentIndex(1);
    if(group->currentData().toString()!="validation")return 23;
    QTest::mouseClick(save,Qt::LeftButton); QTest::qWait(200);
    if(spy.count()!=1 || save->isEnabled())return 24;
    const auto request=spy.at(0).at(0).toJsonObject();
    if(request["protocol"].toString()!="camera_b_v1" || request["op"].toString()!="save")return 25;
    state["last_request_id"]=request["request_id"]; state["stamp"]=QDateTime::currentMSecsSinceEpoch()/1000.; panel.setStatus(state); QTest::qWait(200);
    if(!save->isEnabled())return 26;
    state["busy"]=true; state["stamp"]=QDateTime::currentMSecsSinceEpoch()/1000.; panel.setStatus(state); QTest::qWait(200);
    if(save->isEnabled())return 27;
    state["busy"]=false; state["stamp"]=QDateTime::currentMSecsSinceEpoch()/1000.; panel.setStatus(state); QTest::qWait(2000);
    if(save->isEnabled())return 28;
    if(panel.findChild<QPushButton*>("indexOnly") || panel.findChild<QPushButton*>("admittance"))return 29;
    auto *liveCapture=panel.findChild<QPushButton*>("bCapture");
    if(liveCapture->isEnabled())return 30;
    state["mode"]="live";state["camera"]=QJsonObject{{"ready",false}};
    state["stamp"]=QDateTime::currentMSecsSinceEpoch()/1000.;panel.setStatus(state);QTest::qWait(250);
    if(liveCapture->isEnabled())return 31;
    state["camera"]=QJsonObject{{"ready",true},{"message","camera ready"}};
    state["stamp"]=QDateTime::currentMSecsSinceEpoch()/1000.;panel.setStatus(state);QTest::qWait(250);
    if(!liveCapture->isEnabled())return 32;
    panel.grab().save(QString(argv[2])+"/camera_b_live.png");
    QTest::mouseClick(liveCapture,Qt::LeftButton);
    auto liveRequest=spy.last().at(0).toJsonObject();
    if(liveRequest["op"]!="capture" || !liveRequest["stationary_confirmed"].toBool() || liveRequest["group"]!="validation")return 33;
    if(liveCapture->isEnabled())return 34;
    std::cout<<"PASS: B five image modes / two sizes / explicit group / request ACK / busy / stale / no motion controls\n";
    return 0;
  }
  g1_tcp_diagnostics::TcpAPanel panel;
  QSignalSpy spy(&panel,&g1_tcp_diagnostics::TcpAPanel::requestBuilt);
  state["stamp"]=QDateTime::currentMSecsSinceEpoch()/1000.;
  panel.resize(560,850); panel.show(); panel.setStatus(state); QTest::qWait(250);
  auto *tabs=panel.findChild<QTabWidget*>("tabs");
  auto *capture=panel.findChild<QPushButton*>("capture");
  if(capture->isEnabled())return 4;
  for(auto size:{QSize(560,850),QSize(460,720)}){
    panel.resize(size);
    for(int i=0;i<tabs->count();++i){state["stamp"]=QDateTime::currentMSecsSinceEpoch()/1000.;panel.setStatus(state);tabs->setCurrentIndex(i); QTest::qWait(180);
      if(i==1 && (!capture->isVisible() || capture->mapTo(&panel,QPoint(0,capture->height())).y()>panel.height()))return 10;
      panel.grab().save(QString(argv[2])+QString("/panel_%1_%2.png").arg(size.width()).arg(i));}
  }
  for(auto name:{"indexOnly","admittance","hold"})if(panel.findChild<QPushButton*>(name)->isEnabled())return 11;
  state["mode"]="live";state["ready"]=true;state["busy"]=false;
  state["stamp"]=QDateTime::currentMSecsSinceEpoch()/1000.;
  panel.setStatus(state);QTest::qWait(200);
  tabs->setCurrentIndex(1);
  for(auto name:{"sourceCheck","pelvisCheck","handCheck","contactCheck","lightCheck"})panel.findChild<QCheckBox*>(name)->setChecked(true);
  if(!capture->isEnabled())return 5;
  QTest::mouseClick(capture,Qt::LeftButton);
  if(spy.count()!=1 || spy.at(0).at(0).toJsonObject()["op"].toString()!="capture")return 6;
  if(panel.findChild<QCheckBox*>("contactCheck")->isChecked() || capture->isEnabled())return 7;
  if(panel.findChild<QComboBox*>("phase"))return 8;
  auto *group=panel.findChild<QComboBox*>("group");
  group->setCurrentIndex(1);
  if(!group->isEnabled() || group->currentData().toString()!="validation")return 8;
  state["samples"]=QJsonArray();state["busy"]=false;
  state["stamp"]=QDateTime::currentMSecsSinceEpoch()/1000.;panel.setStatus(state);
  QJsonObject proof{{"protocol","tcp_a_owner_v1"},{"hardware",true},{"hand_enabled",true},
    {"gateway_fresh",true},{"stamp",QDateTime::currentMSecsSinceEpoch()/1000.}};
  QJsonObject owner{{"tcp_a_controls",proof},{"worker_busy",false},
    {"gateway","ARMED_HOLD"}};
  panel.setOwnerStatus(owner);QTest::qWait(400);
  if(!panel.findChild<QPushButton*>("indexOnly")->isEnabled() || !panel.findChild<QPushButton*>("admittance")->isEnabled() || panel.findChild<QPushButton*>("hold")->isEnabled())return 12;
  owner["gateway"]="TASK_ADMITTANCE";
  panel.setOwnerStatus(owner);QTest::qWait(250);
  if(panel.findChild<QPushButton*>("indexOnly")->isEnabled() || !panel.findChild<QPushButton*>("hold")->isEnabled())return 13;
  QTest::qWait(1900);
  if(capture->isEnabled())return 9;
  for(auto name:{"indexOnly","admittance","hold"})if(panel.findChild<QPushButton*>(name)->isEnabled())return 14;
  std::cout<<"PASS: five tabs / two sizes / pinned capture / explicit contact / unified waist / owner controls gated / stale backend\n";
  return 0;
}
