#include <fstream>

#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

#include <geometry_msgs/Point.h>

#include <ros/package.h>

#include "create_area_panel.h"


namespace my_rviz_plugins
{
CreateAreaPanel::CreateAreaPanel(QWidget* parent) : rviz::Panel(parent)
{
  ////////////////
  //// LAYOUT ////
  ////////////////

  // MAIN LAYOUT BEGIN //
  QVBoxLayout* main_layout = new QVBoxLayout;
  
  // input layout begin //
  QHBoxLayout* sfn_input = new QHBoxLayout;
  sfn_input->addWidget(new QLabel("Save File Name: "));
  save_file_name_ = new QLineEdit("area_list");
  sfn_input->addWidget(save_file_name_);

  QHBoxLayout* an_input  = new QHBoxLayout;
  an_input->addWidget(new QLabel("Area Name: "));
  area_name_ = new QLineEdit("some_area");
  an_input->addWidget(area_name_);

  main_layout->addLayout(sfn_input);
  main_layout->addLayout(an_input);
  // input layout end //

  // button layout begin //
  QHBoxLayout* button_layout = new QHBoxLayout;

  save_button_   = new QPushButton("Save");
  delete_button_ = new QPushButton("Delete All");

  button_layout->addWidget(save_button_);
  button_layout->addWidget(delete_button_);

  main_layout->addLayout(button_layout);
  // button layout end //

  setLayout(main_layout);
  // MAIN LAYOUT END //

  //////////////////
  ///// CONNECT ////
  //////////////////

  connect(save_file_name_, SIGNAL(returnPressed()), this, SLOT(updateName()));
  connect(area_name_,      SIGNAL(returnPressed()), this, SLOT(updateName()));
  connect(save_button_,   SIGNAL(clicked()), this, SLOT(saveFile()));
  connect(delete_button_, SIGNAL(clicked()), this, SLOT(deleteAll()));

  ///////////////
  ///// ROS /////
  ///////////////

  sub_ = nh_.subscribe("clicked_point", 1, &CreateAreaPanel::callback, this);
  pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_markers", 1);

  // initialize point_
  point_.x = point_.y = point_.z = 0;

  // // Initialize polygon_marker_ for the polygon
  // polygon_marker_.header.frame_id = "map"; // Adjust frame_id as per your requirement
  // polygon_marker_.ns = "create_area_polygon";
  // polygon_marker_.id = 0;
  // polygon_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  // polygon_marker_.action = visualization_msgs::Marker::ADD;
  // polygon_marker_.pose.orientation.w = 1.0;
  // polygon_marker_.scale.x = 0.1; // Line width
  // polygon_marker_.color.r = 1.0;
  // polygon_marker_.color.a = 1.0;
}

CreateAreaPanel::~CreateAreaPanel() = default;
  
void CreateAreaPanel::updateName()
{
  QLineEdit* senderLineEdit = qobject_cast<QLineEdit*>(sender());
  std::string n_name = senderLineEdit->text().toStdString();
  if (senderLineEdit == save_file_name_)
  {
    ROS_INFO("Save File Name has been changed to: %s", n_name.c_str());
  }
  if (senderLineEdit == area_name_)
  {
    ROS_INFO("Area Name has been changed to: %s", n_name.c_str());
  }
}

void CreateAreaPanel::saveFile()
{
  if (points_.empty()) 
  {
      ROS_WARN("No points to save.");
      return;
  }

  std::string file_name = ros::package::getPath("my_rviz_plugins") + "/" + save_file_name_->text().toStdString() + ".yaml";
  ROS_INFO("FIle name: %s", file_name.c_str());
  std::ofstream outputFile(file_name, std::ios::app);
  if (outputFile.is_open())
  {
    ROS_INFO("Start writing file!");
    outputFile << area_name_->text().toStdString() << ": [" << std::endl;
    for (const auto& point : points_)
    {
      outputFile << "\t[" << point.x << ", " << point.y << "],\n";
    }
    outputFile << "]" << std::endl;
    outputFile.close();
    ROS_INFO("File was saved!");
  }
  else 
  {
    ROS_WARN("Unable to open file!");
  }
}

void CreateAreaPanel::deleteAll()
{
  points_.clear();
  markers_.clear();
  // Publish markers with action DELETE to remove them from RViz
  visualization_msgs::Marker marker_delete;
  marker_delete.action = visualization_msgs::Marker::DELETEALL;
  pub_.publish(marker_delete);
  ROS_INFO("Deleted all registed points!");
}

void CreateAreaPanel::callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  geometry_msgs::Point pre_point = point_;
  if (msg->point != pre_point)
  {
    point_ = msg->point;
    points_.push_back(point_);

    //////////////////////////////////
    ///// VISUALIZE WITH MARKERS  ////
    //////////////////////////////////

    // prepare marker
    visualization_msgs::Marker marker;

    marker.header = msg->header;
    marker.ns = "create_area_marker";
    marker.id = markers_.size();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = point_;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();  // Marker remains until explicitly deleted

    markers_.push_back(marker); // Add the new marker to the list

    // Publish all markers
    for (const auto& m : markers_)
    {
        pub_.publish(m);
    }

    //////////////////////////////////
    ///// VISUALIZE WITH POLYGON  ////
    //////////////////////////////////

    // // update polygon
    // polygon_marker_.points.clear();
    // for (const auto& p: points_)
    // {
    //   geometry_msgs::Point pt;
    //   pt.x = p.x;
    //   pt.y = p.y;
    //   pt.z = 0;
    //   polygon_marker_.points.push_back(pt);
    // }

    // // Ensure closed loop by adding the first point at the end
    // if (!points_.empty())
    // {
    //     geometry_msgs::Point first_point = points_.front();
    //     polygon_marker_.points.push_back(first_point);
    // }

    // // Publish polygon_marker_
    // pub_.publish(polygon_marker_);
  }
}
} // end namespace my_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_rviz_plugins::CreateAreaPanel, rviz::Panel)
