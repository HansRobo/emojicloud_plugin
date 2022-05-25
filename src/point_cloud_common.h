/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef POINT_CLOUD_COMMON_H
#define POINT_CLOUD_COMMON_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <deque>
#include <queue>
#include <vector>

#include <QList>
#include <QObject>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_rendering/objects/point_cloud.hpp"

#include "rviz_default_plugins/displays/pointcloud/point_cloud_transformer.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_selection_handler.hpp"

#include "rviz_default_plugins/displays/pointcloud/point_cloud_selection_handler.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_transformer.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_transformer_factory.hpp"
#include "rviz_default_plugins/visibility_control.hpp"
#endif

namespace rviz_common {
class BoolProperty;
class Display;
class DisplayContext;
class EnumProperty;
class FloatProperty;
class PointCloudTransformer;
} // namespace rviz_common

namespace emojicloud_plugin {
class PointCloudSelectionHandler;
struct IndexAndMessage;
typedef std::shared_ptr<PointCloudSelectionHandler>
    PointCloudSelectionHandlerPtr;
typedef std::shared_ptr<rviz_common::PointCloudTransformer> PointCloudTransformerPtr;
typedef std::vector<std::string> V_string;

struct RVIZ_DEFAULT_PLUGINS_PUBLIC CloudInfo
{
  CloudInfo();
  ~CloudInfo();

  // clear the point cloud, but keep selection handler around
  void clear();

  void setSelectable(
    bool selectable, float selection_box_size, rviz_common::DisplayContext * context);

  rclcpp::Time receive_time_;

  Ogre::SceneManager * manager_;

  sensor_msgs::msg::PointCloud2::ConstSharedPtr message_;

  Ogre::SceneNode * scene_node_;
  std::shared_ptr<rviz_rendering::PointCloud> cloud_;
  PointCloudSelectionHandlerPtr selection_handler_;

  std::vector<rviz_rendering::PointCloud::Point> transformed_points_;

  Ogre::Quaternion orientation_;
  Ogre::Vector3 position_;
};

/**
 * \class PointCloudCommon
 * \brief Displays a point cloud of type sensor_msgs::PointCloud
 *
 * By default it will assume channel 0 of the cloud is an intensity value, and
 * will color them by intensity. If you set the channel's name to "rgb", it will
 * interpret the channel as an integer rgb value, with r, g and b all being 8
 * bits.
 */
class PointCloudCommon : public QObject {
  Q_OBJECT
public:
  typedef std::shared_ptr<CloudInfo> CloudInfoPtr;
  typedef std::deque<CloudInfoPtr> D_CloudInfo;
  typedef std::vector<CloudInfoPtr> V_CloudInfo;
  typedef std::queue<CloudInfoPtr> Q_CloudInfo;
  typedef std::list<CloudInfoPtr> L_CloudInfo;

  PointCloudCommon(rviz_common::Display *display);
  ~PointCloudCommon();

  void initialize(rviz_common::DisplayContext *context, Ogre::SceneNode *scene_node);

//  void fixedFrameChanged();
  void reset();
  void update(float wall_dt, float ros_dt);

  void addMessage(sensor_msgs::msg::PointCloud::ConstSharedPtr cloud);
  void addMessage(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

//  ros::CallbackQueueInterface *getCallbackQueue() { return &cbqueue_; }

  rviz_common::Display *getDisplay() { return display_; }

  bool auto_size_;

  rviz_common::properties::BoolProperty * selectable_property_;
  rviz_common::properties::FloatProperty * point_world_size_property_;
  rviz_common::properties::FloatProperty * point_pixel_size_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::EnumProperty * xyz_transformer_property_;
  rviz_common::properties::EnumProperty * color_transformer_property_;
  rviz_common::properties::EnumProperty * style_property_;
  rviz_common::properties::FloatProperty * decay_time_property_;

//  void setAutoSize(bool auto_size);

public Q_SLOTS:
  void causeRetransform();

private Q_SLOTS:
  void updateSelectable();
  void updateStyle();
  void updateBillboardSize();
  void updateAlpha();
  void updateXyzTransformer();
  void updateColorTransformer();
  void setXyzTransformerOptions(rviz_common::properties::EnumProperty * prop);
  void setColorTransformerOptions(rviz_common::properties::EnumProperty * prop);

private:
  /**
   * \brief Transforms the cloud into the correct frame, and sets up our
   * renderable cloud
   */
  bool transformCloud(const CloudInfoPtr & cloud,
                      bool fully_update_transformers);

  void processMessage(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
//  bool transformPoints(
//    const CloudInfoPtr & cloud_info, V_PointCloudPoint & cloud_points, bool update_transformers);
  void setProblematicPointsToInfinity(V_PointCloudPoint & cloud_points);
  void updateStatus();

  PointCloudTransformerPtr getXYZTransformer(
    ensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
  PointCloudTransformerPtr getColorTransformer(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
  void retransform();
//  void onTransformerOptions(V_string &ops, uint32_t mask);

  void loadTransformers();
  void loadTransformer(
    PointCloudTransformerPtr trans,
    std::string name,
    const std::string & lookup_name);

  float getSelectionBoxSize();
  void setPropertiesHidden(const QList<rviz_common::properties::Property *> & props, bool hide);
  void fillTransformerOptions(rviz_common::properties::EnumProperty * prop, uint32_t mask);

  void insertNewClouds(float point_decay_time, const rclcpp::Time & now);
  float getSizeForRenderMode(const rviz_rendering::PointCloud::RenderMode & mode);

  void updateTransformerProperties();

  /**
   * Instead of deleting obsolete cloud infos, we just clear them
   * and put them into obsolete_cloud_infos, so active selections are preserved
   *
   * If decay time == 0, clear the old cloud when we get a new one.
   * Otherwise, clear all the outdated ones
   */
  void collectObsoleteCloudInfos(float point_decay_time, const rclcpp::Time & now);

  /// Garbage-collect old point clouds that don't have an active selection
  void removeObsoleteCloudInfos();

  bool cloudInfoIsDecayed(
    CloudInfoPtr cloud_info, float point_decay_time, const rclcpp::Time & now);

  D_CloudInfo cloud_infos_;

  Ogre::SceneNode * scene_node_;

  V_CloudInfo new_cloud_infos_;
  std::mutex new_clouds_mutex_;

  L_CloudInfo obsolete_cloud_infos_;

  struct TransformerInfo
  {
    PointCloudTransformerPtr transformer;
    QList<rviz_common::properties::Property *> xyz_props;
    QList<rviz_common::properties::Property *> color_props;

    std::string readable_name;
    std::string lookup_name;
  };
  typedef std::map<std::string, TransformerInfo> M_TransformerInfo;

  std::recursive_mutex transformers_mutex_;
  M_TransformerInfo transformers_;
  bool new_xyz_transformer_;
  bool new_color_transformer_;
  bool needs_retransform_;

  pluginlib::ClassLoader<rviz::PointCloudTransformer>
      *transformer_class_loader_;

  rviz_common::Display * display_;
  rviz_common::DisplayContext * context_;
  rclcpp::Clock::SharedPtr clock_;

  static const std::string message_status_name_;

  friend class PointCloudSelectionHandler;
};

class PointCloudSelectionHandler : public rviz::SelectionHandler {
public:
  PointCloudSelectionHandler(float box_size,
                             PointCloudCommon::CloudInfo *cloud_info,
                             rviz_common::DisplayContext *context);
  virtual ~PointCloudSelectionHandler();

  virtual void createProperties(const rviz::Picked &obj,
                                rviz::Property *parent_property);
  virtual void destroyProperties(const rviz::Picked &obj,
                                 rviz::Property *parent_property);

  virtual bool needsAdditionalRenderPass(uint32_t pass) {
    if (pass < 2) {
      return true;
    }

    return false;
  }

  virtual void preRenderPass(uint32_t pass);
  virtual void postRenderPass(uint32_t pass);

  virtual void onSelect(const rviz::Picked &obj);
  virtual void onDeselect(const rviz::Picked &obj);

  virtual void getAABBs(const rviz::Picked &obj, rviz::V_AABB &aabbs);

  void setBoxSize(float size) { box_size_ = size; }

  bool hasSelections() { return !boxes_.empty(); }

private:
  PointCloudCommon::CloudInfo *cloud_info_;
  QHash<IndexAndMessage, rviz::Property *> property_hash_;
  float box_size_;
};

} // namespace emojicloud_plugin

#endif // RVIZ_POINT_CLOUD_COMMON_H
