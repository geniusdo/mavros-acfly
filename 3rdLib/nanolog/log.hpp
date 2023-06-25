#include "NanoLogCpp17.h"
#include <string>
using namespace NanoLog::LogLevels;

static void initLog(const char *file) {
    NanoLog::setLogFile(file);
    NanoLog::preallocate();
    NanoLog::setLogLevel(NOTICE);
}
static uint64_t timeConvert(int time_H, int time_L) {
    uint64_t time;
    time = static_cast<uint64_t>(time_H) * 1000000000 + static_cast<uint64_t>(time_L);
    return time;
}
static void LOG_INT(uint64_t time, int data, char *label) {
    uint32_t time_H = static_cast<uint32_t>(time / 1000000000);
    uint32_t time_L = static_cast<uint32_t>(time % 1000000000);
    NANO_LOG(NOTICE, "$INT %d %d %d '%s'", time_H, time_L, data, label);
}

static void LOG_FLOAT(uint64_t time, float data, char *label) {
    uint32_t time_H = static_cast<uint32_t>(time / 1000000000);
    uint32_t time_L = static_cast<uint32_t>(time % 1000000000);
    NANO_LOG(NOTICE, "$FLOAT %d %d %f '%s'", time_H, time_L, data, label);
}

static void LOG_STRING(uint64_t time, char *data, char *label) {
    uint32_t time_H = static_cast<uint32_t>(time / 1000000000);
    uint32_t time_L = static_cast<uint32_t>(time % 1000000000);
    NANO_LOG(NOTICE, "$STRING %d %d %s '%s'", time_H, time_L, data, label);
}

static void LOG_ODOM(uint64_t time,
              float    pos_x,
              float    pos_y,
              float    pos_z,
              float    orientation_x,
              float    orientation_y,
              float    orientation_z,
              float    orientation_w,
              float    twist_linear_x,
              float    twist_linear_y,
              float    twist_linear_z,
              float    twist_angular_x,
              float    twist_angular_y,
              float    twist_angular_z,
              char    *label) {
    uint32_t time_H = static_cast<uint32_t>(time / 1000000000);
    uint32_t time_L = static_cast<uint32_t>(time % 1000000000);
    NANO_LOG(NOTICE, "$ODOM %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f '%s'", time_H, time_L,
             pos_x, pos_y, pos_z, orientation_x, orientation_y, orientation_z, orientation_w,
             twist_linear_x, twist_linear_y, twist_linear_z, twist_angular_x, twist_angular_y,
             twist_angular_z, label);
}

static void LOG_p_xyz(uint64_t time, float pos_x, float pos_y, float pos_z, char *label) {
    LOG_ODOM(time, pos_x, pos_x, pos_x, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, label);
}

static void LOG_pv_xyz(uint64_t time,
                float    pos_x,
                float    pos_y,
                float    pos_z,
                float    twist_linear_x,
                float    twist_linear_y,
                float    twist_linear_z,
                char    *label) {
    LOG_ODOM(time, pos_x, pos_x, pos_x, 0.0, 0.0, 0.0, 1.0, twist_linear_x, twist_linear_y,
             twist_linear_z, 0.0, 0.0, 0.0, label);
}
static void LOG_TRAJ(uint64_t time,
              float    ref_p_x,
              float    ref_p_y,
              float    ref_p_z,
              float    ref_v_x,
              float    ref_v_y,
              float    ref_v_z,
              float    ref_yaw,
              float    ref_yaw_rate,
              char    *label) {
    uint32_t time_H = static_cast<uint32_t>(time / 1000000000);
    uint32_t time_L = static_cast<uint32_t>(time % 1000000000);
    NANO_LOG(NOTICE, "$TRAJ %d %d %f %f %f %f %f %f %f %f '%s'", time_H, time_L, ref_p_x, ref_p_y,
             ref_p_z, ref_v_x, ref_v_y, ref_v_z, ref_yaw, ref_yaw_rate, label);
}
static void LOG_SYNC() {
    NanoLog::sync();
}