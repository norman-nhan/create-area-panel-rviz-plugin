// include/my_rviz_plugins/simple_panel.h
#ifndef CREATE_AREA_PANEL_H
#define CREATE_AREA_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include <QPushButton>
#include <QVBoxLayout>
#include <QLineEdit>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include <visualization_msgs/Marker.h>

#include <vector> 

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
  void saveFile();
  void deleteAll();

  void callback(const geometry_msgs::PointStamped::ConstPtr& msg);

protected:
  QLineEdit*   save_file_name_;
  QLineEdit*   area_name_;
  QPushButton* save_button_;
  QPushButton* delete_button_;

  std::vector<geometry_msgs::Point> points_;
  geometry_msgs::Point point_;

  // visualization_msgs::Marker polygon_marker_;
  std::vector<visualization_msgs::Marker> markers_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};
} // end namespace my_rviz_plugins

#endif // CREATE_AREA_PANEL_H