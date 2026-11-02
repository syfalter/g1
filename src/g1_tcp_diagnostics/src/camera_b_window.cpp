#include "g1_tcp_diagnostics/camera_b_panel.hpp"
#include <QApplication>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QLabel>
#include <QTableWidget>
#include <QTimer>
#include <iostream>
int main(int argc,char **argv) {
  if(qEnvironmentVariable("ROS_DOMAIN_ID")!="96" || qEnvironmentVariable("ROS_LOCALHOST_ONLY")!="1") {
    std::cerr<<"Experiment B UI requires localhost / domain 96\n"; return 2;
  }
  rclcpp::init(argc,argv); QApplication app(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("g1_camera_b_window");
  g1_tcp_diagnostics::CameraBPanel panel; panel.attachNode(node);
  panel.setWindowTitle("实验 B · RGB / 深度一致性"); panel.resize(1320,920); panel.show();
  QTimer timer; timer.setInterval(20);
  QObject::connect(&timer,&QTimer::timeout,[&]{if(rclcpp::ok())rclcpp::spin_some(node);else app.quit();}); timer.start();
  // In-process capture also works when the desktop compositor returns black
  // pixels for external X11 screenshots. Optional offline UI smoke check only.
  const auto check=qEnvironmentVariable("G1_CAMERA_B_UI_CHECK_DIR");
  if(!check.isEmpty())QTimer::singleShot(5000,[&]{
    QDir().mkpath(check);
    const auto connection=panel.findChild<QLabel*>("bConnection")->text();
    const int rows=panel.findChild<QTableWidget*>("bSamples")->rowCount();
    const auto image=panel.findChild<QWidget*>("boardImage")->grab().toImage();int lit=0;
    for(int y=0;y<image.height();++y)for(int x=0;x<image.width();++x)
      if(image.pixelColor(x,y).lightness()>80)++lit;
    const bool rendered=panel.grab().save(check+"/connected_window.png");
    const bool passed=rendered && connection.contains("domain 96") && rows>0 && lit>10000;
    QFile output(check+"/connected_window.json");output.open(QIODevice::WriteOnly);
    output.write(QJsonDocument(QJsonObject{{"passed",passed},{"connection",connection},{"rows",rows},{"image_lit_pixels",lit}}).toJson());
    std::cout<<"B actual backend UI: rows="<<rows<<" image_lit="<<lit<<" passed="<<passed<<std::endl;
    app.exit(passed?0:3);
  });
  const int result=app.exec(); if(rclcpp::ok())rclcpp::shutdown(); return result;
}
