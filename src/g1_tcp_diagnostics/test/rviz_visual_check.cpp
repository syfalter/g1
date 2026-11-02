#include <cstdlib>
#include <iostream>
#include <QApplication>
#include <QDir>
#include <QFileInfo>
#include <QImage>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_rendering/render_window.hpp>

int main(int argc,char **argv) {
  if(argc!=3 || qEnvironmentVariable("ROS_DOMAIN_ID")!="96" || qEnvironmentVariable("ROS_LOCALHOST_ONLY")!="1") {
    std::cerr<<"usage (isolated domain 96): rviz_visual_check CONFIG OUTPUT_DIR\n"; return 2;
  }
  const QString config=argv[1], directory=argv[2]; QDir().mkpath(directory);
  rclcpp::init(argc,argv); QApplication app(argc,argv);
  auto node=std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("tcp_a_visual_check");
  auto frame=std::make_unique<rviz_common::VisualizationFrame>(node);
  frame->setApp(&app); frame->setSplashPath(""); frame->initialize(node,config); frame->show();
  int result=3;
  QTimer::singleShot(9000,[&]{
    frame->getRenderWindow()->renderNow();
    frame->getRenderWindow()->captureScreenShot((directory+"/rviz_canvas.png").toStdString());
    frame->grab().save(directory+"/rviz_widget.png");
    QImage image(directory+"/rviz_canvas.png"); int colored=0, lit=0;
    for(int y=0;y<image.height();++y)for(int x=0;x<image.width();++x) {
      const QColor c=image.pixelColor(x,y);
      if(std::max({c.red(),c.green(),c.blue()})>75)++lit;
      if(std::max({c.red(),c.green(),c.blue()})-std::min({c.red(),c.green(),c.blue()})>60)++colored;
    }
    result=!image.isNull() && lit>500 && colored>200?0:4;
    std::cout<<"Ogre framebuffer "<<image.width()<<"x"<<image.height()<<" lit="<<lit<<" colored="<<colored
             <<" result="<<result<<std::endl;
    app.quit();
  });
  app.exec(); frame.reset(); node.reset(); rclcpp::shutdown(); return result;
}
