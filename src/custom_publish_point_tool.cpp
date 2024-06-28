#include "custom_publish_point_tool.h"

namespace my_rviz_plugins
{
CustomPublishPointTool::CustomPublishPointTool() : rviz::PublishPoint()
{
}

CustomPublishPointTool::~CustomPublishPointTool()
{
}

void CustomPublishPointTool::onInitialize()
{
  rviz::PublishPoint::onInitialize();
  setName("Custom Publish Point");
}
} // end namespace my_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_rviz_plugins::CustomPublishPointTool, rviz::Tool)
