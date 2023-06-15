/**
 * @file command.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @brief This file is from mavros open source respository, thanks for their contribution.
 * @version 1.0
 * @date 2022-01-24
 *
 * @copyright Copyright (c) 2022 acfly
 * @copyright Copyright 2014,2015,2016,2017 Vladimir Ermakov.
 * For commercial use, please contact acfly: https://www.acfly.cn
 *
 */

#include <chrono>
#include <condition_variable>
#include <mavros/mavros_plugin.h>
#include <mavros/px4_custom_mode.h>

#include <mavros_msgs/CommandAck.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandInt.h>
#include <mavros_msgs/CommandLandLocal.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandSetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandTakeoffLocal.h>
#include <mavros_msgs/CommandTriggerControl.h>
#include <mavros_msgs/CommandTriggerInterval.h>
#include <mavros_msgs/CommandVtolTransition.h>

namespace mavros {
namespace std_plugins {
static constexpr double ACK_TIMEOUT_DEFAULT = 5.0;
using utils::enum_value;
using lock_guard  = std::lock_guard<std::mutex>;
using unique_lock = std::unique_lock<std::mutex>;

class CommandTransaction {
public:
    std::mutex              cond_mutex_;
    std::condition_variable ack_;
    uint16_t                expected_command_;
    uint8_t                 result_;

    explicit CommandTransaction(uint16_t command)
        : ack_(),
          expected_command_(command),
          // Default result if wait ack timeout
          // 等待应答超时的缺省结果
          result_(enum_value(mavlink::common::MAV_RESULT::FAILED)) {}
};

/**
 * @brief Command plugin, send any command via COMMAND_LONG
 * @brief 命令ROS插件，通过COMMAND_LONG的mavlink信息发送指令
 * @note 该插件可以通过ROS话题和服务向飞控下发COMMAND信息，并监控应答信息
 */
class CommandPlugin : public plugin::PluginBase {
public:
    CommandPlugin() : PluginBase(), cmd_nh("~cmd"), use_comp_id_system_control(false) {}

    void initialize(UAS &uas_) override {
        PluginBase::initialize(uas_);

        double command_ack_timeout;

        cmd_nh.param("command_ack_timeout", command_ack_timeout, ACK_TIMEOUT_DEFAULT);
        cmd_nh.param("use_comp_id_system_control", use_comp_id_system_control, false);

        command_ack_timeout_dt = ros::Duration(command_ack_timeout);

        command_long_srv =
            cmd_nh.advertiseService("command", &CommandPlugin::command_long_cb, this);
        command_ack_srv =
            cmd_nh.advertiseService("command_ack", &CommandPlugin::command_ack_cb, this);
        command_int_srv =
            cmd_nh.advertiseService("command_int", &CommandPlugin::command_int_cb, this);
        set_mode_srv = cmd_nh.advertiseService("set_mode", &CommandPlugin::set_mode_cb, this);
        arming_srv   = cmd_nh.advertiseService("arming", &CommandPlugin::arming_cb, this);
        set_home_srv = cmd_nh.advertiseService("set_home", &CommandPlugin::set_home_cb, this);
        takeoff_srv  = cmd_nh.advertiseService("takeoff", &CommandPlugin::takeoff_cb, this);
        land_srv     = cmd_nh.advertiseService("land", &CommandPlugin::land_cb, this);
        takeoff_local_srv =
            cmd_nh.advertiseService("takeoff_local", &CommandPlugin::takeoff_local_cb, this);
        land_local_srv = cmd_nh.advertiseService("land_local", &CommandPlugin::land_local_cb, this);
        trigger_control_srv =
            cmd_nh.advertiseService("trigger_control", &CommandPlugin::trigger_control_cb, this);
        trigger_interval_srv =
            cmd_nh.advertiseService("trigger_interval", &CommandPlugin::trigger_interval_cb, this);
        vtol_transition_srv =
            cmd_nh.advertiseService("vtol_transition", &CommandPlugin::vtol_transition_cb, this);
    }

    Subscriptions get_subscriptions() override {
        return {
            make_handler(&CommandPlugin::handle_command_ack),
        };
    }

private:
    using L_CommandTransaction = std::list<CommandTransaction>;

    std::mutex mutex;

    ros::NodeHandle    cmd_nh;
    ros::ServiceServer command_long_srv;
    ros::ServiceServer command_int_srv;
    ros::ServiceServer command_ack_srv;
    ros::ServiceServer set_mode_srv;
    ros::ServiceServer arming_srv;
    ros::ServiceServer set_home_srv;
    ros::ServiceServer takeoff_srv;
    ros::ServiceServer land_srv;
    ros::ServiceServer takeoff_local_srv;
    ros::ServiceServer land_local_srv;
    ros::ServiceServer trigger_control_srv;
    ros::ServiceServer trigger_interval_srv;
    ros::ServiceServer vtol_transition_srv;

    bool use_comp_id_system_control;

    L_CommandTransaction ack_waiting_list;
    ros::Duration        command_ack_timeout_dt;

    /* message handlers */
    /* 信息回调句柄 */

    void handle_command_ack(const mavlink::mavlink_message_t  *msg,
                            mavlink::common::msg::COMMAND_ACK &ack) {
        lock_guard lock(mutex);

        // 源id仅接受FCU的应答
        if (ack.target_system != m_uas->fcu_link->get_system_id() ||
            ack.target_component != m_uas->fcu_link->get_component_id())
            return;

        for (auto &tr : ack_waiting_list) {
            if (tr.expected_command_ == ack.command) {
                using mavlink::common::MAV_RESULT;

                tr.result_ = ack.result;
                switch (ack.result) {
                case enum_value(MAV_RESULT::ACCEPTED):
                    ROS_INFO_STREAM_NAMED("cmd", "CMD: Command " + std::to_string(ack.command) +
                                                     " accepted.");
                    break;
                case enum_value(MAV_RESULT::CANCELLED):
                    ROS_INFO_STREAM_NAMED("cmd", "CMD: Command " + std::to_string(ack.command) +
                                                     " cancelled.");
                    break;
                case enum_value(MAV_RESULT::DENIED):
                    ROS_WARN_STREAM_NAMED("cmd", "CMD: Command " + std::to_string(ack.command) +
                                                     " denied.");
                    break;
                case enum_value(MAV_RESULT::FAILED):
                    ROS_ERROR_STREAM_NAMED("cmd", "CMD: Command " + std::to_string(ack.command) +
                                                      " failed.");
                    break;
                case enum_value(MAV_RESULT::IN_PROGRESS):
                    ROS_INFO_STREAM_NAMED("cmd", "CMD: Command " + std::to_string(ack.command) +
                                                     " in progress.");
                    break;
                case enum_value(MAV_RESULT::TEMPORARILY_REJECTED):
                    ROS_WARN_STREAM_NAMED("cmd", "CMD: Command " + std::to_string(ack.command) +
                                                     " temporarily rejected.");
                    break;
                case enum_value(MAV_RESULT::UNSUPPORTED):
                    ROS_WARN_STREAM_NAMED("cmd", "CMD: Command " + std::to_string(ack.command) +
                                                     " unsupported.");
                    break;

                default:
                    break;
                }

                tr.ack_.notify_all();
                return;
            }
        }

        ROS_WARN_THROTTLE_NAMED(10, "cmd", "CMD: Unexpected command %u, result %u", ack.command,
                                ack.result);
    }

    /* mid-level functions */
    /* 中间件函数 */

    bool wait_ack_for(CommandTransaction &tr) {
        unique_lock lock(tr.cond_mutex_);
        if (tr.ack_.wait_for(lock, std::chrono::nanoseconds(command_ack_timeout_dt.toNSec())) ==
            std::cv_status::timeout) {
            ROS_WARN_NAMED("cmd", "CMD: Command %u -- wait ack timeout", tr.expected_command_);
            return false;
        } else {
            return true;
        }
    }

    /**
     * Common function for COMMAND_LONG service callbacks.
     * @note success is bool in messages, but has unsigned char type in C++
     * COMMAND_LONG服务的通用回调函数
     * @note 在信息中success变量为bool型，但在C++语法中为unsigned char型
     */
    bool send_command_long_and_wait(bool           broadcast,
                                    uint16_t       command,
                                    uint8_t        confirmation,
                                    float          param1,
                                    float          param2,
                                    float          param3,
                                    float          param4,
                                    float          param5,
                                    float          param6,
                                    float          param7,
                                    unsigned char &success,
                                    uint8_t       &result) {
        using mavlink::common::MAV_RESULT;

        unique_lock lock(mutex);

        L_CommandTransaction::iterator ack_it;

        // check transactions
        // 查重
        for (const auto &tr : ack_waiting_list) {
            if (tr.expected_command_ == command) {
                ROS_WARN_THROTTLE_NAMED(10, "cmd", "CMD: Command %u already in progress", command);
                return false;
            }
        }

        // Don't expect any ACK in broadcast mode.
        // 广播模式无应答
        bool is_ack_required =
            (confirmation != 0 || m_uas->is_ardupilotmega() || m_uas->is_px4()) && !broadcast;
        if (is_ack_required)
            ack_it = ack_waiting_list.emplace(ack_waiting_list.end(), command);

        command_long(broadcast, command, confirmation, param1, param2, param3, param4, param5,
                     param6, param7);

        if (is_ack_required) {
            lock.unlock();
            bool is_not_timeout = wait_ack_for(*ack_it);
            lock.lock();

            success = is_not_timeout && ack_it->result_ == enum_value(MAV_RESULT::ACCEPTED);
            result  = ack_it->result_;

            ack_waiting_list.erase(ack_it);
        } else {
            success = true;
            result  = enum_value(MAV_RESULT::ACCEPTED);
        }

        return true;
    }

    // Common function for COMMAND_INT service callbacks.
    // COMMAND_INT服务的通用回调函数
    bool send_command_int(bool           broadcast,
                          uint8_t        frame,
                          uint16_t       command,
                          uint8_t        current,
                          uint8_t        autocontinue,
                          float          param1,
                          float          param2,
                          float          param3,
                          float          param4,
                          int32_t        x,
                          int32_t        y,
                          float          z,
                          unsigned char &success) {
        // COMMAND_INT don't produce COMMAND_ACK, so wait don't needed.
        // COMMAND_INT不产生COMMAND_ACK，所以不等待
        command_int(broadcast, frame, command, current, autocontinue, param1, param2, param3,
                    param4, x, y, z);

        success = true;
        return true;
    }

    bool send_command_ack(uint16_t       command,
                          uint8_t        req_result,
                          uint8_t        progress,
                          int32_t        result_param2,
                          unsigned char &success,
                          uint8_t       &res_result) {
        using mavlink::common::MAV_RESULT;

        command_ack(command, req_result, progress, result_param2);

        success    = true;
        res_result = enum_value(MAV_RESULT::ACCEPTED);

        return true;
    }

    /* low-level send */
    /* 底层发送 */

    template <typename MsgT>
    inline void set_target(MsgT &cmd, bool broadcast) {
        using mavlink::minimal::MAV_COMPONENT;

        const uint8_t tgt_sys_id  = (broadcast) ? 0 : m_uas->get_tgt_system();
        const uint8_t tgt_comp_id = (broadcast) ? 0
                                    : (use_comp_id_system_control)
                                        ? enum_value(MAV_COMPONENT::COMP_ID_SYSTEM_CONTROL)
                                        : m_uas->get_tgt_component();

        cmd.target_system    = tgt_sys_id;
        cmd.target_component = tgt_comp_id;
    }

    void command_long(bool     broadcast,
                      uint16_t command,
                      uint8_t  confirmation,
                      float    param1,
                      float    param2,
                      float    param3,
                      float    param4,
                      float    param5,
                      float    param6,
                      float    param7) {
        const uint8_t confirmation_fixed = (broadcast) ? 0 : confirmation;

        mavlink::common::msg::COMMAND_LONG cmd{};
        set_target(cmd, broadcast);

        cmd.command      = command;
        cmd.confirmation = confirmation_fixed;
        cmd.param1       = param1;
        cmd.param2       = param2;
        cmd.param3       = param3;
        cmd.param4       = param4;
        cmd.param5       = param5;
        cmd.param6       = param6;
        cmd.param7       = param7;

        UAS_FCU(m_uas)->send_message_ignore_drop(cmd);
    }

    void command_int(bool     broadcast,
                     uint8_t  frame,
                     uint16_t command,
                     uint8_t  current,
                     uint8_t  autocontinue,
                     float    param1,
                     float    param2,
                     float    param3,
                     float    param4,
                     int32_t  x,
                     int32_t  y,
                     float    z) {
        mavlink::common::msg::COMMAND_INT cmd{};
        set_target(cmd, broadcast);

        cmd.frame        = frame;
        cmd.command      = command;
        cmd.current      = current;
        cmd.autocontinue = autocontinue;
        cmd.param1       = param1;
        cmd.param2       = param2;
        cmd.param3       = param3;
        cmd.param4       = param4;
        cmd.x            = x;
        cmd.y            = y;
        cmd.z            = z;

        UAS_FCU(m_uas)->send_message_ignore_drop(cmd);
    }

    void command_ack(uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2) {
        mavlink::common::msg::COMMAND_ACK cmd{};
        set_target(cmd, false);

        cmd.command       = command;
        cmd.result        = result;
        cmd.progress      = progress;
        cmd.result_param2 = result_param2;

        UAS_FCU(m_uas)->send_message_ignore_drop(cmd);
    }

    /* ros callbacks */
    /* ros回调函数 */

    bool command_long_cb(mavros_msgs::CommandLong::Request  &req,
                         mavros_msgs::CommandLong::Response &res) {
        return send_command_long_and_wait(req.broadcast, req.command, req.confirmation, req.param1,
                                          req.param2, req.param3, req.param4, req.param5,
                                          req.param6, req.param7, res.success, res.result);
    }

    bool command_int_cb(mavros_msgs::CommandInt::Request  &req,
                        mavros_msgs::CommandInt::Response &res) {
        return send_command_int(req.broadcast, req.frame, req.command, req.current,
                                req.autocontinue, req.param1, req.param2, req.param3, req.param4,
                                req.x, req.y, req.z, res.success);
    }

    bool command_ack_cb(mavros_msgs::CommandAck::Request  &req,
                        mavros_msgs::CommandAck::Response &res) {
        return send_command_ack(req.command, req.result, req.progress, req.result_param2,
                                res.success, res.result);
    }

    bool set_mode_cb(mavros_msgs::CommandSetMode::Request  &req,
                     mavros_msgs::CommandSetMode::Response &res) {
        using mavlink::minimal::MAV_MODE_FLAG;
        uint8_t  base_mode   = req.base_mode;
        uint32_t custom_mode = 0;

        if (req.custom_mode != "") {
            if (!m_uas->cmode_from_str(req.custom_mode, custom_mode)) {
                using mavlink::common::MAV_RESULT;
                res.success = false;
                res.result  = enum_value(MAV_RESULT::DENIED);
                return true;
            }

            base_mode |= (m_uas->get_armed()) ? enum_value(MAV_MODE_FLAG::SAFETY_ARMED) : 0;
            base_mode |= (m_uas->get_hil_state()) ? enum_value(MAV_MODE_FLAG::HIL_ENABLED) : 0;
            base_mode |= enum_value(MAV_MODE_FLAG::CUSTOM_MODE_ENABLED);
        }
        px4::custom_mode cm(custom_mode);

        using mavlink::common::MAV_CMD;
        return send_command_long_and_wait(false, enum_value(MAV_CMD::DO_SET_MODE), 1,
                                          (float)base_mode, (float)cm.main_mode, (float)cm.sub_mode,
                                          0, 0, 0, 0, res.success, res.result);
    }

    bool arming_cb(mavros_msgs::CommandBool::Request  &req,
                   mavros_msgs::CommandBool::Response &res) {
        using mavlink::common::MAV_CMD;
        return send_command_long_and_wait(false, enum_value(MAV_CMD::COMPONENT_ARM_DISARM), 1,
                                          (req.value) ? 1.0 : 0.0, 0, 0, 0, 0, 0, 0, res.success,
                                          res.result);
    }

    bool set_home_cb(mavros_msgs::CommandHome::Request  &req,
                     mavros_msgs::CommandHome::Response &res) {
        using mavlink::common::MAV_CMD;
        return send_command_long_and_wait(
            false, enum_value(MAV_CMD::DO_SET_HOME), 1, (req.current_gps) ? 1.0 : 0.0, 0, 0,
            req.yaw, req.latitude, req.longitude, req.altitude, res.success, res.result);
    }

    bool takeoff_cb(mavros_msgs::CommandTOL::Request &req, mavros_msgs::CommandTOL::Response &res) {
        using mavlink::common::MAV_CMD;
        return send_command_long_and_wait(false, enum_value(MAV_CMD::NAV_TAKEOFF), 1, req.min_pitch,
                                          0, 0, req.yaw, req.latitude, req.longitude, req.altitude,
                                          res.success, res.result);
    }

    bool land_cb(mavros_msgs::CommandTOL::Request &req, mavros_msgs::CommandTOL::Response &res) {
        using mavlink::common::MAV_CMD;
        return send_command_long_and_wait(false, enum_value(MAV_CMD::NAV_LAND), 1, 0, 0, 0, req.yaw,
                                          req.latitude, req.longitude, req.altitude, res.success,
                                          res.result);
    }

    bool takeoff_local_cb(mavros_msgs::CommandTakeoffLocal::Request  &req,
                          mavros_msgs::CommandTakeoffLocal::Response &res) {
        using mavlink::common::MAV_CMD;
        return send_command_long_and_wait(false, enum_value(MAV_CMD::NAV_TAKEOFF_LOCAL), 1,
                                          req.pitch, 0, req.ascend_rate, req.yaw, req.x, req.y,
                                          req.z, res.success, res.result);
    }

    bool land_local_cb(mavros_msgs::CommandLandLocal::Request  &req,
                       mavros_msgs::CommandLandLocal::Response &res) {
        using mavlink::common::MAV_CMD;
        return send_command_long_and_wait(false, enum_value(MAV_CMD::NAV_LAND_LOCAL), 1, req.target,
                                          req.offset, req.descend_rate, req.yaw, req.x, req.y,
                                          req.z, res.success, res.result);
    }

    bool trigger_control_cb(mavros_msgs::CommandTriggerControl::Request  &req,
                            mavros_msgs::CommandTriggerControl::Response &res) {
        using mavlink::common::MAV_CMD;
        if (req.trigger_enable == 0 || req.sequence_reset == 1 || req.trigger_pause == 1)
            m_uas->first_sync_flag = false;

        return send_command_long_and_wait(
            false, enum_value(MAV_CMD::DO_TRIGGER_CONTROL), 1, (req.trigger_enable) ? 1.0 : 0.0,
            (req.sequence_reset) ? 1.0 : 0.0, (req.trigger_pause) ? 1.0 : 0.0, 0, 0, 0, 0,
            res.success, res.result);
    }

    bool trigger_interval_cb(mavros_msgs::CommandTriggerInterval::Request  &req,
                             mavros_msgs::CommandTriggerInterval::Response &res) {
        using mavlink::common::MAV_CMD;

        // trigger interval can only be set when triggering is disabled
        // 禁用触发时才能设置触发间隔
        return send_command_long_and_wait(false, enum_value(MAV_CMD::DO_SET_CAM_TRIGG_INTERVAL), 1,
                                          req.cycle_time, req.integration_time, 0, 0, 0, 0, 0,
                                          res.success, res.result);
    }

    bool vtol_transition_cb(mavros_msgs::CommandVtolTransition::Request  &req,
                            mavros_msgs::CommandVtolTransition::Response &res) {
        using mavlink::common::MAV_CMD;
        return send_command_long_and_wait(false, enum_value(MAV_CMD::DO_VTOL_TRANSITION), false,
                                          req.state, 0, 0, 0, 0, 0, 0, res.success, res.result);
    }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::CommandPlugin, mavros::plugin::PluginBase)
