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

#ifndef EMOJI_POINT_CLOUD2_DISPLAY_H
#define EMOJI_POINT_CLOUD2_DISPLAY_H

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define EMOJICLOUD_PLUGIN_EXPORT __attribute__ ((dllexport))
#define EMOJICLOUD_PLUGIN_IMPORT __attribute__ ((dllimport))
#else
#define EMOJICLOUD_PLUGIN_EXPORT __declspec(dllexport)
#define EMOJICLOUD_PLUGIN_IMPORT __declspec(dllimport)
#endif
#ifdef EMOJICLOUD_PLUGIN_BUILDING_LIBRARY
#define EMOJICLOUD_PLUGIN_PUBLIC EMOJICLOUD_PLUGIN_EXPORT
#else
#define EMOJICLOUD_PLUGIN_PUBLIC EMOJICLOUD_PLUGIN_IMPORT
#endif
#define EMOJICLOUD_PLUGIN_PUBLIC_TYPE EMOJICLOUD_PLUGIN_PUBLIC
#define EMOJICLOUD_PLUGIN_LOCAL
#else
#define EMOJICLOUD_PLUGIN_EXPORT __attribute__ ((visibility("default")))
#define EMOJICLOUD_PLUGIN_IMPORT
#if __GNUC__ >= 4
#define EMOJICLOUD_PLUGIN_PUBLIC __attribute__ ((visibility("default")))
#define EMOJICLOUD_PLUGIN_LOCAL  __attribute__ ((visibility("hidden")))
#else
#define EMOJICLOUD_PLUGIN_PUBLIC
#define EMOJICLOUD_PLUGIN_LOCAL
#endif
#define EMOJICLOUD_PLUGIN_PUBLIC_TYPE
#endif

#include <memory>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rviz_common/message_filter_display.hpp"

#include "rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"

namespace rviz_common
{
namespace properties
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}
}

namespace emojicloud_plugin {

struct Offsets
{
  uint32_t x, y, z;
};

class EMOJICLOUD_PLUGIN_PUBLIC EmojiPointCloud2Display : public
  rviz_common::MessageFilterDisplay<sensor_msgs::msg::PointCloud2>
{
Q_OBJECT
public:
  EmojiPointCloud2Display();
  ~EmojiPointCloud2Display();

  void reset() override;

  void update(float wall_dt, float ros_dt) override;

  void onDisable() override;

private Q_SLOTS:
  void updateQueueSize();

protected:
  /** @brief Do initialization. Overridden from MessageFilterDisplay. */
  void onInitialize() override;

  /** @brief Process a single message.  Overridden from MessageFilterDisplay. */
  void processMessage(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) override;

  bool hasXYZChannels(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) const;
  bool cloudDataMatchesDimensions(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) const;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr filterOutInvalidPoints(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) const;
  sensor_msgs::msg::PointCloud2::_data_type
  filterData(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) const;
  Offsets determineOffsets(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) const;
  bool validateFloatsAtPosition(
      sensor_msgs::msg::PointCloud2::_data_type::const_iterator position,
      const Offsets offsets) const;

  rviz_common::properties::IntProperty *queue_size_property_;

  std::unique_ptr<rviz_default_plugins::PointCloudCommon> point_cloud_common_;
};

} // namespace emojicloud_plugin

#endif  // EMOJI_POINT_CLOUD2_DISPLAY_H
