// include/my_rviz_plugins/simple_panel.h
#ifndef CREATE_AREA_PANEL_H
#define CREATE_AREA_PANEL_H

// Qt implementation >>>>
#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
#endif
// Qt implementation <<<<

#include <QPushButton>
#include <QVBoxLayout>
#include <QLineEdit>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

namespace my_rviz_plugins
{
class CreateAreaPanel : public rviz::Panel
{
  Q_OBJECT
public:
  CreateAreaPanel(QWidget* parent = 0);
  ~CreateAreaPanel();

protected Q_SLOTS:
  void updateName();
  void addPoint();
  void saveFile();
  void deleteAll();
  void pointCallback(const geometry_msgs::PointStamped::ConstPtr & point_stamped_msg);

protected:
  QLineEdit*   save_file_name_;
  QLineEdit*   area_name_;
  QPushButton* add_button_;
  QPushButton* save_button_;
  QPushButton* delete_button_;

  geometry_msgs::Point point_;
  geometry_msgs::Point* points_ = nullptr;
  int max_points_ = 100; // Maximum number of points (adjust as needed)
  int num_points_ = 0;   // Number of points currently stored

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
};
} // end namespace my_rviz_plugins

#endif // CREATE_AREA_PANEL_H
