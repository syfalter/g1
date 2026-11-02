// Test-only RViz host: native framebuffer capture works when X11 readback is black.
#include <QApplication>
#include <QTimer>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <rviz_common/ros_integration/ros_client_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_rendering/render_window.hpp>

int main(int argc, char **argv) {
  const char *domain = std::getenv("ROS_DOMAIN_ID");
  const char *local = std::getenv("ROS_LOCALHOST_ONLY");
  if (argc != 3 || !domain || std::stoi(domain) < 93 || std::stoi(domain) > 99 ||
      !local || std::string(local) != "1") {
    std::cerr << "Requires CONFIG OUTPUT and isolated localhost domain 93..99\n";
    return 2;
  }
  const QString config = argv[1];
  const std::filesystem::path output = argv[2];
  QApplication app(argc, argv);
  rviz_common::ros_integration::RosClientAbstraction ros;
  auto node = ros.init(argc, argv, "task_rviz_smoke_renderer", false);
  int result = 0;
  {
    rviz_common::VisualizationFrame frame(node);
    frame.setApp(&app);
    frame.setSplashPath("");
    frame.initialize(node, config);
    frame.show();
    int previous = 0;
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]() {
      if (!ros.ok()) { app.quit(); return; }
      int requested = 0;
      std::ifstream(output / "native_request.txt") >> requested;
      if (requested <= previous) return;
      try {
        frame.getRenderWindow()->render();
        frame.getRenderWindow()->captureScreenShot((output / "native_frame.png").string());
        std::ofstream(output / "native_ready.txt") << requested;
        previous = requested;
      } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        result = 1;
        app.quit();
      }
    });
    timer.start(50);
    app.exec();
  }
  ros.shutdown();
  return result;
}
