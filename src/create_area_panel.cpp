#include <fstream>

#include <ros/ros.h>
#include <rviz/panel.h>

#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

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

  add_button_    = new QPushButton("Add Point");
  save_button_   = new QPushButton("Save");
  delete_button_ = new QPushButton("Delete All");

  button_layout->addWidget(add_button_);
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
  connect(add_button_,    SIGNAL(clicked()), this, SLOT(addPoint()));
  connect(save_button_,   SIGNAL(clicked()), this, SLOT(saveFile()));
  connect(delete_button_, SIGNAL(clicked()), this, SLOT(deleteAll()));

  /////////
  // ROS //
  /////////

  // subscriber
  sub_ = nh_.subscribe("clicked_point", 10, &CreateAreaPanel::pointCallback, this);

  // initialize point array
  points_ = new geometry_msgs::Point[max_points_];
  num_points_ = 0;
}

CreateAreaPanel::~CreateAreaPanel()
{
  delete[] points_;
  num_points_ = 0;
}
  
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

void CreateAreaPanel::addPoint()
{
  ROS_INFO("Attempt to add point:\nx: %f,\ny: %f,\nz: %f\n", point_.x, point_.y, point_.z);
  if (num_points_ < max_points_)
  {
    points_[num_points_].x = point_.x;
    points_[num_points_].y = point_.y;
    points_[num_points_].z = point_.z;
    num_points_++;
    ROS_INFO("Point was added!");
  }
  else ROS_WARN("Maximum number of points reached (%d). Cannot add more points.", max_points_);
}

bool isNull(const geometry_msgs::Point& data)
{
    return (data.x == 0 && data.y == 0 && data.z == 0);
}

void CreateAreaPanel::saveFile()
{
  if (num_points_ == 0)
  {
      ROS_WARN("No points to save.");
      return;
  }
  std::string file_name = "~/" + save_file_name_->text().toStdString() + ".yaml";
  std::ofstream outputFile(file_name, std::ios::app);
  if (outputFile.is_open())
  {
    ROS_INFO("Start writing file!");
    // ROS_INFO("[%f, %f, %f]", points_[0].x, points_[0].y, points_[0].z);
    outputFile << area_name_->text().toStdString() << ": [" << std::endl;
    for (int i = 0; !isNull(points_[i]); i++)
    {
      outputFile << "\t[" << points_[i].x << ", " << points_[i].y << "],\n";
    }
    outputFile << "]" << std::endl;
    outputFile.close();
  }
  else 
  {
    ROS_WARN("Unable to open file!");
  }
  ROS_INFO("File was saved!");
}

void CreateAreaPanel::deleteAll()
{
  num_points_ = 0;
  ROS_INFO("Deleted all registed points!");
}

void CreateAreaPanel::pointCallback(const geometry_msgs::PointStamped::ConstPtr & point_stamped_msg)
{
  point_ = point_stamped_msg->point;
  
  ROS_INFO("Received point:\nx: %f,\ny: %f,\nz: %f\n", point_.x, point_.y, point_.z);
}

} // end namespace my_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_rviz_plugins::CreateAreaPanel, rviz::Panel)
