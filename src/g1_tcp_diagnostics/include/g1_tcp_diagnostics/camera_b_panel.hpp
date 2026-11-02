#ifndef G1_TCP_DIAGNOSTICS__CAMERA_B_PANEL_HPP_
#define G1_TCP_DIAGNOSTICS__CAMERA_B_PANEL_HPP_
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
namespace g1_tcp_diagnostics {
class BoardImage;
class CameraBPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit CameraBPanel(QWidget *parent=nullptr);
  void onInitialize() override;
  void attachNode(rclcpp::Node::SharedPtr node);
  void setStatus(const QJsonObject &status);
Q_SIGNALS:
  void requestBuilt(const QJsonObject &request);
private:
  void send(const QString &op, QJsonObject fields={});
  void refresh();
  void showSelected();
  QString selectedId() const;
  QLabel *connection_, *board_, *message_, *caption_, *metrics_, *cameraStatus_;
  QComboBox *group_, *view_;
  QLineEdit *note_;
  QPlainTextEdit *details_;
  QTableWidget *samples_;
  BoardImage *image_;
  QList<QPushButton *> actions_;
  QPushButton *exclude_, *changeGroup_, *checkCamera_, *snapshot_, *capture_;
  QWidget *liveBar_;
  QJsonObject status_, pending_;
  std::mutex mutex_;
  bool dirty_{false}, rebuilding_{false};
  qint64 receivedMs_{0}, sentMs_{0};
  QString signature_, pendingRequest_, imagePath_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};
}
#endif
