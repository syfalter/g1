#include "g1_tcp_diagnostics/camera_c_panel.hpp"
#include <QApplication>
#include <QDir>
#include <QFile>
#include <QGraphicsView>
#include <QJsonDocument>
#include <QLabel>
#include <QTableWidget>
#include <QTimer>
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);QApplication app(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("g1_camera_c_window");
  g1_tcp_diagnostics::CameraCPanel panel;panel.attachNode(node);panel.setWindowTitle("实验 C · 相机 / 腕部一致性");panel.resize(1320,920);panel.show();
  QTimer timer;timer.setInterval(20);QObject::connect(&timer,&QTimer::timeout,[&]{if(rclcpp::ok())rclcpp::spin_some(node);else app.quit();});timer.start();
  const auto check=qEnvironmentVariable("G1_CAMERA_C_UI_CHECK_DIR");
  if(!check.isEmpty())QTimer::singleShot(6000,[&]{
    QDir().mkpath(check);const auto connection=panel.findChild<QLabel*>("cConnection")->text();
    const int rows=panel.findChild<QTableWidget*>("cSamples")->rowCount();
    const auto image=panel.findChild<QGraphicsView*>("cImage")->grab().toImage();int lit=0;
    for(int y=0;y<image.height();++y)for(int x=0;x<image.width();++x)if(image.pixelColor(x,y).lightness()>80)++lit;
    const bool rendered=panel.grab().save(check+"/connected_window.png");
    const bool passed=rendered&&connection.contains("C 已连接")&&rows>0&&lit>10000;
    QFile out(check+"/connected_window.json");out.open(QIODevice::WriteOnly);
    out.write(QJsonDocument(QJsonObject{{"passed",passed},{"rows",rows},{"image_lit_pixels",lit},{"connection",connection}}).toJson());
    app.exit(passed?0:3);
  });
  int result=app.exec();if(rclcpp::ok())rclcpp::shutdown();return result;
}
