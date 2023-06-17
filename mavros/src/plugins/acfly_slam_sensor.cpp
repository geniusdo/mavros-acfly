/**
 * @file acfly_slam_sensor.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief acfly slam sensor plugin.
 * @version 1.0
 * @date 2022-02-03
 *
 * @copyright Copyright (c) 2022 acfly
 * For commercial use, please contact acfly: https://www.acfly.cn
 *
 */

#include <eigen_conversions/eigen_msg.h>
#include <mavros/acfly_position_sensor_base.h>
#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>

#include "log.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
namespace mavros {
namespace std_plugins {
/**
 * @brief acfly slam sensor send plugin
 * @brief acfly位置传感器发送插件，专门用来发送slam信息
 * @warning acfly自定义的信息全部直接以ENU-FLU系发送，与PX4不同
 */
class AcflySlamSensorPlugin : public plugin::AcflyPositionSensorBase,
                              private plugin::TF2ListenerMixin<AcflySlamSensorPlugin> {
public:
    AcflySlamSensorPlugin()
        : AcflyPositionSensorBase(), ass_nh("~acfly_slam_sensor"), tf_rate(10.0) {}

    void initialize(UAS &uas_) override {
        PluginBase::initialize(uas_);

        // 获取传感器信息
        get_sensor_info(&ass_nh);

        // 传感器到飞控body系的三维变换

        std::vector<double> rot{}, trans{};
        std::string         sensor_id, body_id;
        ass_nh.param<std::string>("sensor_id", sensor_id, "camera");
        ass_nh.param<std::string>("body_id", body_id, "base_link");
        // 获取sensor->body的变换
        if (ass_nh.getParam("sensor_body_rotation", rot)) {
            tr_sensor_body = Eigen::Affine3d(ftf::quaternion_from_rpy(rot[0], rot[1], rot[2]));
        } else {
            tr_sensor_body = Eigen::Affine3d(ftf::quaternion_from_rpy(0, 0, 0));
            ROS_WARN_STREAM_NAMED(
                "acfly_slam_sensor",
                "ASS: No rotation parameter between sensor and body, set to default(0, 0, 0)");
        }
        if (ass_nh.getParam("sensor_body_translation", trans)) {
            tr_sensor_body.translation() = Eigen::Vector3d(trans[0], trans[1], trans[2]);
        } else {
            tr_sensor_body.translation() = Eigen::Vector3d(0, 0, 0);
            ROS_WARN_STREAM_NAMED(
                "acfly_slam_sensor",
                "ASS: No translation parameter between sensor and body, set to default(0, 0, 0)");
        }

        // tf参数
        bool tf_listen;
        ass_nh.param("tf/listen", tf_listen, true);
        ass_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
        ass_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "base_link");
        ass_nh.param("tf/rate_limit", tf_rate, 10.0);

        // 添加静态tf(SLAM传感器与飞控body系之间的变换)
        std::vector<geometry_msgs::TransformStamped> transform_vector;
        m_uas->add_static_transform(sensor_id, body_id, tr_sensor_body, transform_vector);
        m_uas->tf2_static_broadcaster.sendTransform(transform_vector);

        // 启动tf监听线程(详细请看TF2ListenerMixin)或订阅
        if (tf_listen) {
            ROS_INFO_STREAM_NAMED("acfly_slam_sensor", "ASS: Listen to transform "
                                                           << tf_frame_id << " -> "
                                                           << tf_child_frame_id);
            tf2_start("acfly_slam_tf", &AcflySlamSensorPlugin::transform_cb);
        } else {
            ROS_INFO_STREAM_NAMED("acfly_slam_sensor", "ASS: Subscribe pose");
            pose_sub = ass_nh.subscribe("pose", 10, &AcflySlamSensorPlugin::pose_cb, this);
            pose_cov_sub =
                ass_nh.subscribe("pose_cov", 10, &AcflySlamSensorPlugin::pose_cov_cb, this);
            odom_sub = ass_nh.subscribe("odom", 10, &AcflySlamSensorPlugin::odom_cb, this);
        }

        // 重置位置传感器
        reset_sub = ass_nh.subscribe("reset", 10, &AcflySlamSensorPlugin::reset_cb, this);

        // 设置位置传感器不可用
        block_sub = ass_nh.subscribe("block", 10, &AcflySlamSensorPlugin::block_cb, this);

        // 回环检测
        loop_sub = ass_nh.subscribe("loop", 10, &AcflySlamSensorPlugin::loop_cb, this);

        debug_pub = ass_nh.advertise<geometry_msgs::TwistStamped>("debug", 1000);
    }

    Subscriptions get_subscriptions() override {
        return {/* 禁用接收 */};
    }

private:
    friend class TF2ListenerMixin;
    ros::NodeHandle ass_nh;

    ros::Subscriber pose_sub;
    ros::Subscriber pose_cov_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber reset_sub;
    ros::Subscriber block_sub;
    ros::Subscriber loop_sub;
    ros::Publisher  debug_pub;
    Eigen::Affine3d tr_sensor_body{};

    std::string tf_frame_id;
    std::string tf_child_frame_id;
    double      tf_rate;

    /* ros callbacks */
    /* ROS回调函数 */

    void transform_cb(const geometry_msgs::TransformStamped &transform) {
        if (!m_uas->get_pos_sensor_connection_status(sensor_ind)) {
            register_position_sensor();
        } else {
            Eigen::Affine3d tr;
            tf::transformMsgToEigen(transform.transform, tr);

            update_position_sensor(transform.header.stamp, tr.translation(),
                                   Eigen::Vector3d::Zero(), Eigen::Quaterniond(tr.rotation()));
        }
    }

    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose_stamp) {
        if (!m_uas->get_pos_sensor_connection_status(sensor_ind)) {
            register_position_sensor();
        } else {
            Eigen::Affine3d tr;
            tf::poseMsgToEigen(pose_stamp->pose, tr);

            update_position_sensor(pose_stamp->header.stamp, tr.translation(),
                                   Eigen::Vector3d::Zero(), Eigen::Quaterniond(tr.rotation()));
        }
    }

    void pose_cov_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_cov_stamp) {
        if (!m_uas->get_pos_sensor_connection_status(sensor_ind)) {
            register_position_sensor();
        } else {
            Eigen::Affine3d tr;
            tf::poseMsgToEigen(pose_cov_stamp->pose.pose, tr);

            update_position_sensor(pose_cov_stamp->header.stamp, tr.translation(),
                                   Eigen::Vector3d::Zero(), Eigen::Quaterniond(tr.rotation()));
        }
    }

    void odom_cb(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        if (!m_uas->get_pos_sensor_connection_status(sensor_ind)) {
            register_position_sensor();
        } else {
            Eigen::Affine3d             tr;
            Eigen::Matrix<double, 6, 1> vel;
            tf::poseMsgToEigen(odom_msg->pose.pose, tr);
            tf::twistMsgToEigen(odom_msg->twist.twist, vel);
            Eigen::Vector3d linear_vel;
            linear_vel << vel[0], vel[1], vel[2];

            tr = tr_sensor_body.inverse() * tr;

            // ROS_INFO("%lf,%lf,%lf", tr_sensor_body(0, 0), tr_sensor_body(0, 1),
            //          tr_sensor_body(0, 2));
            // ROS_INFO("%lf,%lf,%lf", tr_sensor_body(1, 0), tr_sensor_body(1, 1),
            //          tr_sensor_body(1, 2));
            // ROS_INFO("%lf,%lf,%lf", tr_sensor_body(2, 0), tr_sensor_body(2, 1),
            //          tr_sensor_body(2, 2));
            // ROS_INFO("--------------------");

            geometry_msgs::TwistStamped debug_msg;
            debug_msg.header = odom_msg->header;
            debug_msg.twist.linear.x = linear_vel(0);
            debug_msg.twist.linear.y = linear_vel(1);
            debug_msg.twist.linear.z = linear_vel(2);
            debug_pub.publish(debug_msg);

#ifdef LOG_ENABLE
            LOG_ODOM(timeConvert(odom_msg->header.stamp.sec, odom_msg->header.stamp.nsec),
                     tr.translation()[0], tr.translation()[1], tr.translation()[2],
                     Eigen::Quaterniond(tr.rotation()).x(), Eigen::Quaterniond(tr.rotation()).y(),
                     Eigen::Quaterniond(tr.rotation()).z(), Eigen::Quaterniond(tr.rotation()).w(),
                     linear_vel[0], linear_vel[1], linear_vel[2], 0.0, 0.0, 0.0, "poseSensor");
#endif

            update_position_sensor(odom_msg->header.stamp, tr.translation(), linear_vel,
                                   Eigen::Quaterniond(tr.rotation()));
        }
    }

    void reset_cb(const std_msgs::Bool::ConstPtr &reset) {
        if (reset->data)
            unregister_position_sensor();
    }

    void block_cb(const std_msgs::Bool::ConstPtr &block) {
        if (block->data)
            set_position_sensor_unavailable();
    }

    void loop_cb(const std_msgs::Bool::ConstPtr &loop) {
        // 可能会有多线程读写问题
        if (loop->data)
            reset_counter++;
    }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::AcflySlamSensorPlugin, mavros::plugin::PluginBase)