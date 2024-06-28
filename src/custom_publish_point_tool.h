#ifndef CUSTOM_PUBLISH_POINT_TOOL_H
#define CUSTOM_PUBLISH_POINT_TOOL_H

#include <rviz/default_plugin/tools/publish_point.h>

namespace my_rviz_plugins
{
class CustomPublishPointTool : public rviz::PublishPoint
{
Q_OBJECT
public:
  CustomPublishPointTool();
  virtual ~CustomPublishPointTool() override;

  virtual void onInitialize() override;
};
} // end namespace my_rviz_plugins

#endif // CUSTOM_PUBLISH_POINT_TOOL_H
