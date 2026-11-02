#ifndef G1_TCP_DIAGNOSTICS__CAMERA_C_PANEL_HPP_
#define G1_TCP_DIAGNOSTICS__CAMERA_C_PANEL_HPP_
#include <mutex>
#include <QJsonObject>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
class QLabel;
class QComboBox;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QTableWidget;
class QGraphicsView;
class QGraphicsScene;
namespace g1_tcp_diagnostics {
class CameraCPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit CameraCPanel(QWidget *parent=nullptr);
  void onInitialize() override;
  void attachNode(rclcpp::Node::SharedPtr node);
private:
  void send(const QString &op, QJsonObject fields={});
  void refresh();
  void showSelected();
  QLabel *connection_, *message_, *caption_;
  QComboBox *side_, *group_, *view_;
  QLineEdit *mount_, *note_;
  QPlainTextEdit *details_;
  QTableWidget *samples_;
  QGraphicsView *image_;
  QGraphicsScene *scene_;
  QList<QPushButton *> actions_;
  QPushButton *check_, *snapshot_, *capture_;
  QJsonObject status_, pending_;
  std::mutex mutex_;
  bool dirty_{false}, rebuilding_{false};
  qint64 receivedMs_{0}, sentMs_{0};
  QString pendingRequest_, imagePath_, signature_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};
}
#endif
