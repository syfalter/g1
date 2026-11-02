#ifndef G1_TCP_DIAGNOSTICS__TCP_A_PANEL_HPP_
#define G1_TCP_DIAGNOSTICS__TCP_A_PANEL_HPP_
#include <mutex>
#include <QJsonObject>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
class QLabel;
class QComboBox;
class QCheckBox;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QTableWidget;
class QTabWidget;
class QTimer;
namespace g1_tcp_diagnostics {
class TcpAPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit TcpAPanel(QWidget *parent=nullptr);
  void onInitialize() override;
  void setStatus(const QJsonObject &status);
  void setOwnerStatus(const QJsonObject &status);
Q_SIGNALS:
  void requestBuilt(const QJsonObject &request);
private:
  void send(const QString &operation, QJsonObject fields={});
  void refresh();
  void updateCapture();
  void capture();
  void control(const QString &operation);
  void refreshControls();
  QString selectedId() const;
  QLabel *connection_, *message_, *quality_, *identity_, *solveSummary_;
  QLabel *controlStatus_;
  QComboBox *side_, *group_;
  QCheckBox *waistAdmittance_;
  QLineEdit *target_, *note_;
  QCheckBox *source_, *pelvis_, *hand_, *contact_, *light_, *readHand_, *candidate_;
  QPlainTextEdit *system_, *sampleDetails_, *results_;
  QTableWidget *samples_;
  QTabWidget *tabs_;
  QPushButton *capture_, *solve_, *export_, *demo_;
  QPushButton *index_, *admittance_, *hold_;
  QTimer *timer_;
  QJsonObject status_, pending_;
  QJsonObject owner_, pendingOwner_;
  std::mutex mutex_;
  bool dirty_{false}, connected_{false}, backendReady_{false}, busy_{false}, rebuilding_{false};
  qint64 receivedMs_{0};
  qint64 ownerReceivedMs_{0}, controlSentMs_{0};
  QString sampleSignature_, activeSession_, controlRequest_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr requestPub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr statusSub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controlPub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ownerSub_;
};
}
#endif
