/**
 * @brief Camera IMU synchronisation plugin
 * @file cam_imu_sync.cpp
 * @author Mohammed Kabir < mhkabir98@gmail.com >
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Mohammed Kabir.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include "std_msgs/Int64.h"
#include <fstream>
#include <math.h>
#include <mavros/mavros_plugin.h>
#include <mavros_msgs/CamIMUStamp.h>
#include <mavros_msgs/CommandTriggerControl.h>
#include <mavros_msgs/CommandTriggerInterval.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Camera IMU synchronisation plugin
 *
 * This plugin publishes a timestamp for when a external camera system was
 * triggered by the FCU. Sequence ID from the message and the image sequence from
 * camera can be corellated to get the exact shutter trigger time.
 */
class CamIMUSyncPlugin : public plugin::PluginBase {
public:
    CamIMUSyncPlugin() : PluginBase(), cam_imu_sync_nh("~sync") {}

    void initialize(UAS &uas_) override {
        PluginBase::initialize(uas_);

        cam_imu_pub = cam_imu_sync_nh.advertise<mavros_msgs::CamIMUStamp>("cam_imu_stamp", 10);
        offset_pub  = cam_imu_sync_nh.advertise<std_msgs::Int64>("offset", 10);
    }

    Subscriptions get_subscriptions() override {
        return {make_handler(&CamIMUSyncPlugin::handle_cam_trig)};
    }

private:
    ros::NodeHandle cam_imu_sync_nh;
    ros::Publisher  cam_imu_pub;
    ros::Publisher  offset_pub;
    bool            first_sync_flag = false;
    ros::Time       last_stamp;
    int             last_id = 0;

    void handle_cam_trig(const mavlink::mavlink_message_t     *msg,
                         mavlink::common::msg::CAMERA_TRIGGER &ctrig) {
        auto    sync_msg = boost::make_shared<mavros_msgs::CamIMUStamp>();
        int64_t FCU_time = (int64_t)(ctrig.time_usec / 1000000UL) * 1000000000 +
                           (int64_t)(ctrig.time_usec % 1000000UL) * 1000;
        int64_t time_offset;

        if (m_uas->first_sync_flag == false) {
            auto first_stamp = ros::Time::now();
            int64_t curTime = (int64_t)first_stamp.sec * 1000000000 + (int64_t)first_stamp.nsec;
            m_uas->first_sync_flag  = true;
            time_offset             = curTime - FCU_time;
            m_uas->offset_time = time_offset;
        }
        int64_t correctTime   = m_uas->offset_time + FCU_time;
        sync_msg->frame_stamp = ros::Time((int32_t)(correctTime / 1000000000), correctTime % 1000000000);
        if (fabs((sync_msg->frame_stamp - last_stamp).toSec()) > 0.06) {
            ROS_DEBUG("WRONG");
            ROS_DEBUG("current stamp: %lf, id: %ld", sync_msg->frame_stamp.toSec(), ctrig.seq - 1);
            ROS_DEBUG("last stamp: %lf, id: %ld", last_stamp.toSec(), last_id);
        }
        last_stamp = sync_msg->frame_stamp;
        last_id    = ctrig.seq - 1;

        sync_msg->frame_seq_id = ctrig.seq - 1;
        // ROS_ERROR("%ld", bridge::timeOffset);
        cam_imu_pub.publish(sync_msg);
        std_msgs::Int64 offset_msg;
        offset_msg.data = m_uas->offset_time;
        offset_pub.publish(offset_msg);
    }
};
} // namespace extra_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::CamIMUSyncPlugin, mavros::plugin::PluginBase)

