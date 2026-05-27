//
// Created by Clemens Elflein on 27.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "SystemModel.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "ros/ros.h"
#include "xbot_msgs/AbsolutePose.h"
#include "xbot_msgs/WheelTick.h"
#include "xbot_positioning/GPSControlSrv.h"
#include "xbot_positioning/KalmanState.h"
#include "xbot_positioning/SetPoseSrv.h"
#include "xbot_positioning_core.h"

// YL: added in order to init yaw using dock heading
#include "mower_map/GetDockingPointSrv.h"
#include "xbot_msgs/RobotState.h"

ros::Publisher odometry_pub;
ros::Publisher xbot_absolute_pose_pub;

// Debug Publishers
ros::Publisher kalman_state;
ros::Publisher dbg_expected_motion_vector;

// The kalman filters
xbot::positioning::xbot_positioning_core core{};

// True, if we don't want to do gyro calibration on launch
bool skip_gyro_calibration;

// True, if we have wheel ticks (i.e. last_ticks is valid)
bool has_ticks;
xbot_msgs::WheelTick last_ticks;
bool has_gps;
xbot_msgs::AbsolutePose last_gps;

bool first_gps_fix = true;

// True, if last_imu is valid and gyro_offset is valid
bool has_gyro;
sensor_msgs::Imu last_imu;
ros::Time gyro_calibration_start;
double gyro_offset;
int gyro_offset_samples;

// Current speed calculated by wheel ticks
double vx = 0.0;

// Min speed for motion vector to be fed into kalman filter
double min_speed = 0.0;

// Max position accuracy to allow for GPS updates
double max_gps_accuracy;

// True, if we should publish debug topics (expected motion vector and kalman state)
bool publish_debug;

// Antenna offset (offset between point of rotation and antenna)
double antenna_offset_x, antenna_offset_y;
double imu_yaw_deadband = 0.0;

nav_msgs::Odometry odometry;
xbot_positioning::KalmanState state_msg;
xbot_msgs::AbsolutePose xb_absolute_pose_msg;

bool gps_enabled = true;
int gps_outlier_count = 0;
int valid_gps_samples = 0;

ros::Time last_gps_time(0.0);

// --- YL Dock-yaw init ---
static bool have_dock_heading = false;
static double dock_heading = 0.0;          // radians
static bool yaw_initialized_from_dock = false;
static bool is_charging_now = false;

// Service client to retrieve the docking pose (and thus dock heading)
ros::ServiceClient dockingPointClient;

// Map GPS position_accuracy + RTK flags to a sensible EKF covariance (sigma, in meters).
// Smaller sigma = trust GPS more.
inline double gpsPositionSigma(const xbot_msgs::AbsolutePose &msg) {
    const bool is_rtk     = (msg.flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK) != 0;
    const bool is_rtk_fix = (msg.flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED) != 0;
    const bool is_rtk_flt = (msg.flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT) != 0;

    double acc = msg.position_accuracy;
    if (!(acc > 0.0) || !std::isfinite(acc)) acc = 5.0;  // fallback if unknown

    double scale = 10.0;  // default for poor/no-RTK fixes
    if (is_rtk_fix)       scale = 1.0;
    else if (is_rtk_flt)  scale = 3.0;
    else if (is_rtk)      scale = 5.0;

    double sigma = scale * acc;
    const double MIN_SIGMA = 0.005;  // 5 mm
    const double MAX_SIGMA = 500.0;
    if (sigma < MIN_SIGMA) sigma = MIN_SIGMA;
    if (sigma > MAX_SIGMA) sigma = MAX_SIGMA;
    return sigma;
}


void onImu(const sensor_msgs::Imu::ConstPtr &msg) {
    if (!has_gyro) {
        if (!skip_gyro_calibration) {
            if (gyro_offset_samples == 0) {
                ROS_INFO_STREAM("Started gyro calibration");
                gyro_calibration_start = msg->header.stamp;
                gyro_offset = 0;
            }
            gyro_offset += msg->angular_velocity.z;
            gyro_offset_samples++;
            if ((msg->header.stamp - gyro_calibration_start).toSec() < 5) {
                last_imu = *msg;
                return;
            }
            has_gyro = true;
            if (gyro_offset_samples > 0) {
                gyro_offset /= gyro_offset_samples;
            } else {
                gyro_offset = 0;
            }
            gyro_offset_samples = 0;
            ROS_INFO_STREAM("Calibrated gyro offset: " << gyro_offset);
        } else {
            ROS_WARN("Skipped gyro calibration");
            has_gyro = true;
            return;
        }
    }

    double dt = (msg->header.stamp - last_imu.header.stamp).toSec();
    double yaw_rate = msg->angular_velocity.z - gyro_offset;

    // Suppress micro-rates that are likely IMU bias drift when the robot is stationary.
    // Configurable via ~imu_yaw_deadband (rad/s). Set to 0.0 to disable.
    if (imu_yaw_deadband > 0.0 && std::fabs(yaw_rate) < imu_yaw_deadband) {
        yaw_rate = 0.0;
    }

    core.predict(vx, yaw_rate, dt);
    auto x = core.updateSpeed(vx, yaw_rate, 0.01);

    odometry.header.stamp = ros::Time::now();
    odometry.header.seq++;
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = x.x_pos();
    odometry.pose.pose.position.y = x.y_pos();
    odometry.pose.pose.position.z = 0;
    tf2::Quaternion q(0.0, 0.0, x.theta());
    odometry.pose.pose.orientation = tf2::toMsg(q);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header = odometry.header;
    odom_trans.child_frame_id = odometry.child_frame_id;
    odom_trans.transform.translation.x = odometry.pose.pose.position.x;
    odom_trans.transform.translation.y = odometry.pose.pose.position.y;
    odom_trans.transform.translation.z = odometry.pose.pose.position.z;
    odom_trans.transform.rotation = odometry.pose.pose.orientation;

    static tf2_ros::TransformBroadcaster transform_broadcaster;
    transform_broadcaster.sendTransform(odom_trans);

    if (publish_debug) {
        auto state = core.getState();
        state_msg.x = state.x();
        state_msg.y = state.y();
        state_msg.theta = state.theta();
        state_msg.vx = state.vx();
        state_msg.vr = state.vr();

        kalman_state.publish(state_msg);
    }

    odometry_pub.publish(odometry);

    xb_absolute_pose_msg.header = odometry.header;
    xb_absolute_pose_msg.sensor_stamp = 0;
    xb_absolute_pose_msg.received_stamp = 0;
    xb_absolute_pose_msg.source = xbot_msgs::AbsolutePose::SOURCE_SENSOR_FUSION;
    xb_absolute_pose_msg.flags = xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_DEAD_RECKONING;

    xb_absolute_pose_msg.orientation_valid = true;
    // TODO: send motion vector
    xb_absolute_pose_msg.motion_vector_valid = false;
    // TODO: set real value from kalman filter, not the one from the GPS.
    if (has_gps) {
        xb_absolute_pose_msg.position_accuracy = last_gps.position_accuracy;
    } else {
        xb_absolute_pose_msg.position_accuracy = 999;
    }
    if ((ros::Time::now() - last_gps_time).toSec() < 10.0) {
        xb_absolute_pose_msg.flags |= xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE;
    } else {
        // on GPS timeout, we set accuracy to 0.
        xb_absolute_pose_msg.position_accuracy = 999;
    }
    // TODO: set real value
    xb_absolute_pose_msg.orientation_accuracy = 0.01;
    xb_absolute_pose_msg.pose = odometry.pose;
    xb_absolute_pose_msg.vehicle_heading = x.theta();
    xb_absolute_pose_msg.motion_heading = x.theta();

    xbot_absolute_pose_pub.publish(xb_absolute_pose_msg);


    last_imu = *msg;
}

void onWheelTicks(const xbot_msgs::WheelTick::ConstPtr &msg) {
    if (!has_ticks) {
        last_ticks = *msg;
        has_ticks = true;
        return;
    }
    double dt = (msg->stamp - last_ticks.stamp).toSec();

    double d_wheel_l = (double) (msg->wheel_ticks_rl - last_ticks.wheel_ticks_rl) * (
                           1 / (double) msg->wheel_tick_factor);
    double d_wheel_r = (double) (msg->wheel_ticks_rr - last_ticks.wheel_ticks_rr) * (
                           1 / (double) msg->wheel_tick_factor);

    if (msg->wheel_direction_rl) {
        d_wheel_l *= -1.0;
    }
    if (msg->wheel_direction_rr) {
        d_wheel_r *= -1.0;
    }


    double d_ticks = (d_wheel_l + d_wheel_r) / 2.0;
    vx = d_ticks / dt;

    last_ticks = *msg;
}

void onTwistIn(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    vx = msg->twist.linear.x;
}

bool setGpsState(xbot_positioning::GPSControlSrvRequest &req, xbot_positioning::GPSControlSrvResponse &res) {
    gps_enabled = req.gps_enabled;
    return true;
}

bool setPose(xbot_positioning::SetPoseSrvRequest &req, xbot_positioning::SetPoseSrvResponse &res) {
    tf2::Quaternion q;
    tf2::fromMsg(req.robot_pose.orientation, q);

    ROS_INFO_STREAM_THROTTLE(1,"setPose");
    tf2::Matrix3x3 m(q);
    double unused1, unused2, yaw;

    m.getRPY(unused1, unused2, yaw);
    core.setState(req.robot_pose.position.x, req.robot_pose.position.y, yaw, 0, 0);
    return true;
}

void onPose(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
    if (!gps_enabled) {
        // ROS_INFO_STREAM_THROTTLE(gps_message_throttle, "dropping GPS update, since gps_enabled = false.");
        return;
    }
    // TODO fuse with high covariance?
    if ((msg->flags & (xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED)) == 0) {
        // ROS_INFO_STREAM_THROTTLE(1, "Dropped GPS update, since it's not RTK Fixed");
        return;
    }

    if (msg->position_accuracy > max_gps_accuracy) {
        // ROS_INFO_STREAM_THROTTLE(
        //    1, "Dropped GPS update, since it's not accurate enough. Accuracy was: " << msg->position_accuracy <<
        //    ", limit is:" << max_gps_accuracy);
        return;
    }

    double time_since_last_gps = (ros::Time::now() - last_gps_time).toSec();
    if (time_since_last_gps > 5.0) {
        ROS_WARN_STREAM("Last GPS was " << time_since_last_gps << " seconds ago.");
        has_gps = false;
        valid_gps_samples = 0;
        gps_outlier_count = 0;
        last_gps = *msg;
        // we have GPS for next time
        last_gps_time = ros::Time::now();
        return;
    }

    tf2::Vector3 gps_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    tf2::Vector3 last_gps_pos(last_gps.pose.pose.position.x, last_gps.pose.pose.position.y,
                              last_gps.pose.pose.position.z);

    double distance_to_last_gps = (last_gps_pos - gps_pos).length();

    // YL: on first GPS fix, bypass the outlier check entirely so we seed the EKF immediately
    if (first_gps_fix || distance_to_last_gps < 5.0) {
        // inlier, we treat it normally
        // store the gps as last
        last_gps = *msg;
        first_gps_fix = false;
        last_gps_time = ros::Time::now();

        gps_outlier_count = 0;
        valid_gps_samples++;

        double pos_sigma = gpsPositionSigma(*msg);

        if (!has_gps) {
            // YL: removed valid_gps_samples > 10 gate — seed EKF on first valid fix
            // we don't even have gps yet, set odometry to first estimate
            core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, pos_sigma);

            has_gps = true;
        } else if (has_gps) {
            // gps was valid before, we apply the filter
            core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, pos_sigma);
            if (publish_debug) {
                auto m = core.om2.h(core.ekf.getState());
                geometry_msgs::Vector3 dbg;
                dbg.x = m.vx();
                dbg.y = m.vy();
                dbg_expected_motion_vector.publish(dbg);
            }
            if (std::sqrt(std::pow(msg->motion_vector.x, 2) + std::pow(msg->motion_vector.y, 2)) >= min_speed) {
                core.updateOrientation2(msg->motion_vector.x, msg->motion_vector.y, 10000.0);
            }
        }
    } else {
        ROS_WARN_STREAM("GPS outlier found. Distance was: " << distance_to_last_gps);
        gps_outlier_count++;
        // ~10 sec
        if (gps_outlier_count > 10) {
            ROS_ERROR_STREAM("too many outliers, assuming that the current gps value is valid.");
            // store the gps as last
            last_gps = *msg;
            last_gps_time = ros::Time::now();
            has_gps = false;

            valid_gps_samples = 0;
            gps_outlier_count = 0;
        }
    }
}

// YL: Called when xbot_monitoring publishes the robot state.
// When the mower is charging and we haven't yet initialized yaw from the dock,
// retrieve the dock pose from mower_map and use its yaw to seed the EKF.
void onRobotState(const xbot_msgs::RobotState::ConstPtr &msg) {
    is_charging_now = msg->is_charging;

    if (is_charging_now && !yaw_initialized_from_dock) {
        mower_map::GetDockingPointSrv dockSrv;
        if (dockingPointClient.call(dockSrv)) {
            // Extract yaw from the docking pose quaternion
            tf2::Quaternion q;
            tf2::fromMsg(dockSrv.response.docking_pose.orientation, q);
            tf2::Matrix3x3 m(q);
            double unused1, unused2, yaw;
            m.getRPY(unused1, unused2, yaw);

            dock_heading = yaw;
            have_dock_heading = true;

            // Seed the EKF: keep current x,y, override heading with dock heading
            auto s = core.getState();
            core.setState(s.x(), s.y(), dock_heading, 0.0, 0.0);

            yaw_initialized_from_dock = true;
            ROS_INFO_STREAM("[xbot_positioning] Yaw initialized from dock heading: "
                            << dock_heading << " rad (" << (dock_heading * 180.0 / M_PI) << " deg)");
        } else {
            ROS_WARN_STREAM("[xbot_positioning] Could not retrieve docking point to initialize yaw.");
        }
    }

    // Allow re-initialization next time the mower docks
    if (!is_charging_now) {
        yaw_initialized_from_dock = false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xbot_positioning");

    has_gps = false;
    gps_enabled = true;
    vx = 0.0;
    has_gyro = false;
    has_ticks = false;
    gyro_offset = 0;
    gyro_offset_samples = 0;

    valid_gps_samples = 0;
    gps_outlier_count = 0;

    antenna_offset_x = antenna_offset_y = 0;

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    ros::ServiceServer gps_service = n.advertiseService("xbot_positioning/set_gps_state", setGpsState);
    ros::ServiceServer pose_service = n.advertiseService("xbot_positioning/set_robot_pose", setPose);

    paramNh.param("skip_gyro_calibration", skip_gyro_calibration, false);
    paramNh.param("gyro_offset", gyro_offset, 0.0);
    paramNh.param("min_speed", min_speed, 0.01);
    paramNh.param("max_gps_accuracy", max_gps_accuracy, 0.1);
    paramNh.param("debug", publish_debug, false);
    paramNh.param("antenna_offset_x", antenna_offset_x, 0.0);
    paramNh.param("antenna_offset_y", antenna_offset_y, 0.0);
    paramNh.param("imu_yaw_deadband", imu_yaw_deadband, 0.0);

    core.setAntennaOffset(antenna_offset_x, antenna_offset_y);

    ROS_INFO_STREAM("Antenna offset: " << antenna_offset_x << ", " << antenna_offset_y);

    if (gyro_offset != 0.0 && skip_gyro_calibration) {
        ROS_WARN_STREAM("Using gyro offset of: " << gyro_offset);
    }

    odometry_pub = paramNh.advertise<nav_msgs::Odometry>("odom_out", 50);
    xbot_absolute_pose_pub = paramNh.advertise<xbot_msgs::AbsolutePose>("xb_pose_out", 50);
    if (publish_debug) {
        dbg_expected_motion_vector = paramNh.advertise<geometry_msgs::Vector3>("debug_expected_motion_vector", 50);
        kalman_state = paramNh.advertise<xbot_positioning::KalmanState>("kalman_state", 50);
    }

    ros::Subscriber imu_sub = paramNh.subscribe("imu_in", 10, onImu);
    ros::Subscriber twist_sub = paramNh.subscribe("twist_in", 10, onTwistIn);
    ros::Subscriber pose_sub = paramNh.subscribe("xb_pose_in", 10, onPose);
    ros::Subscriber wheel_tick_sub = paramNh.subscribe("wheel_ticks_in", 10, onWheelTicks);

    // YL: subscribe to robot state to detect charging and initialize yaw from dock heading
    dockingPointClient = n.serviceClient<mower_map::GetDockingPointSrv>("mower_map_service/get_docking_point");
    ros::Subscriber robot_state_sub = n.subscribe("/xbot_monitoring/robot_state", 5, onRobotState);

    ros::spin();
    return 0;
}
