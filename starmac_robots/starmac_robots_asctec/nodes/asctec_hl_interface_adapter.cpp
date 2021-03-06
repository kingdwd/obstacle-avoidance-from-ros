/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, UC Regents
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of California nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <algorithm>
#include <cmath>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <dynamic_reconfigure/server.h>
#include <starmac_robots_asctec/AscTecAdapterConfig.h>
#include <string>
#include <angles/angles.h>
//#include "asctec_msgs/CtrlInput.h"
//#include "asctec_msgs/LLStatus.h"
//#include "asctec_msgs/IMUCalcData.h"
#include "asctec_hl_comm/mav_status.h"
#include "asctec_hl_comm/mav_ctrl.h"
#include "std_msgs/Bool.h"
#include "flyer_controller/control_mode_output.h"
#include "starmac_robots_asctec/pid.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include "starmac_robots_asctec/misc.h"

//#define _USE_MATH_DEFINES

using namespace std;

// Never output commands greater than these values no matter what parameters are set:
static double ROLL_SCALE = M_PI / 180.0; // rad/deg
static double PITCH_SCALE = M_PI / 180.0; // rad/deg
static double MAX_ROLL_CMD = M_PI_2; // rad
static double MAX_PITCH_CMD = M_PI_2; // rad
static double MAX_YAW_RATE_CMD = 2.0 * M_PI; // rad/s
static double MAX_THRUST_CMD = 32; // N

// Per email from AscTec,
// """The standard parameter for K_stick_yaw is 120, resulting in a maximum rate of
// 254.760 degrees per second. I.e. a 360° turn takes about 1.5 seconds."""
static double YAW_SCALE = M_PI / 180.0; // rad/s per deg/s

static double THRUST_SCALE = 1.0 / 32.0; // 1/N - approximate

namespace starmac_robots_asctec
{

class AscTecHLInterfaceAdapterNodelet : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // Diagnostic Updater
  diagnostic_updater::Updater diag_updater;
  double min_freq;
  double max_freq;
  diagnostic_updater::FrequencyStatus freq_status;
  // Dynamic Reconfigure Server
  dynamic_reconfigure::Server<starmac_robots_asctec::AscTecAdapterConfig> dyn_reconfig_srv;
  // Parameters
  //string output_topic; // Topic on which to publish output
  //string estop_topic; // Topic on which to publish emergency stop command
  ros::Duration deadman_timeout; // timeout in seconds
  double alt_KP; // N per meter
  double alt_KI; // N per meter-second
  double alt_KD; // N per m/s
  double alt_KDD; // N per m/s/s
  double alt_Ilimit; // N
  double yaw_KP; // deg/s per deg
  double yaw_KI; // deg/s per deg-second
  double yaw_KD; // deg/s per deg/s
  double yaw_Ilimit; // deg/s
  double yaw_rate_limit; // deg/s
  double roll_slew_rate_limit; // deg/s
  double pitch_slew_rate_limit; // deg/s
  double yaw_slew_rate_limit; // deg/s
  double alt_slew_rate_limit; // m/s/s
  double nominal_thrust; // N TODO: change to mass?
  double thrust_mult; // 1.0 means full power
  double max_thrust; // N
  bool thrust_autoadjust; // set to true to multiply thrust by 1/(cos(roll)*cos(pitch))
  double accel_bias; // m/s/s - for a perfect sensor this would be 9.81
  unsigned int land_now_thrust_decrease_rate; // [N/s] - how fast to decrease commanded thrust
  double land_now_min_thrust_ratio; // reduce thrust to this level (as fraction of nominal_thrust) then keep it constant
  double pitch_trim; // deg - will be added to commanded pitch (AscTec sign convention)
  double roll_trim; // deg - will be added to commanded roll (AscTec sign convention)
  double yaw_offset; // deg
  // Publishers
  ros::Publisher output;
  ros::Publisher estop_pub;
  ros::Publisher imu_pub;
  // Subscribers
  ros::Subscriber motor_enable_sub;
  ros::Subscriber control_input_sub;
  ros::Subscriber state_sub;
  ros::Subscriber mav_status_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber land_now_sub;
  // Timers
  ros::Timer deadman_timer;
  // Members
  bool latest_motor_enable;
  bool deadman; // latches to true if motor enable not seen for more than deadman_timeout
  ros::Time prev_control_input_time;
  ros::Time last_motor_enable_time;
  ros::Time last_mav_status_time;
  ros::Time last_state_time;
  ros::Time last_imu_time;
  starmac_robots_asctec::Pid alt_pid;
  starmac_robots_asctec::Pid yaw_pid;
  nav_msgs::Odometry latest_state;
  asctec_hl_comm::mav_status latest_mav_status;
  flyer_controller::control_mode_output prev_control_input_msg;
  ros::Duration max_motor_enable_age;
  ros::Duration max_mav_status_age;
  ros::Duration max_control_input_age;
  ros::Duration max_state_age;
  double alt_p_term;
  double alt_i_term;
  double alt_d_term;
  double alt_dd_term;
  double last_z_accel;
  bool land_now;
  bool estop_sent;
  bool got_first_state;
  double latest_battery_voltage;
  double min_battery_voltage;
  bool battery_low_warning_sent;
  bool mav_status_cb_called;
  bool got_first_control_input;
  double last_thrust;

public:
  AscTecHLInterfaceAdapterNodelet() :
        diag_updater(),
        min_freq(0.1),
        max_freq(1000),
        freq_status(diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq)),
        // Params:
        //output_topic("autopilot/CTRL_INPUT"),
        //estop_topic("autopilot/ESTOP"),
        deadman_timeout(0.5), alt_KP(2.0),
        alt_KI(0.1),
        alt_KD(3.0),
        alt_KDD(0.0),
        alt_Ilimit(5.0),
        yaw_KP(1.0),
        yaw_KI(1.0),
        yaw_KD(0.0),
        yaw_Ilimit(5.0),
        yaw_rate_limit(4.0), //
        roll_slew_rate_limit(10),
        pitch_slew_rate_limit(10),
        yaw_slew_rate_limit(5),
        alt_slew_rate_limit(0.1), //
        nominal_thrust(14.0),
        thrust_mult(1.0),
        max_thrust(18.0),
        thrust_autoadjust(true),
        accel_bias(9.81),
        land_now_thrust_decrease_rate(200),
        // Members:
        latest_motor_enable(false), deadman(false), prev_control_input_time(0), last_motor_enable_time(0), //
        last_mav_status_time(0), last_state_time(0), max_motor_enable_age(ros::Duration(0)),
        max_mav_status_age(ros::Duration(0)), max_control_input_age(ros::Duration(0)), max_state_age(ros::Duration(0)),
        land_now_min_thrust_ratio(0.5), pitch_trim(0), roll_trim(0), yaw_offset(0),
        last_z_accel(0), land_now(false), estop_sent(false), got_first_state(false), latest_battery_voltage(0), min_battery_voltage(0),
        battery_low_warning_sent(false), mav_status_cb_called(false), got_first_control_input(false),
        last_thrust(0)
  {
  }

  void onInit()
  {
    nh_priv = getPrivateNodeHandle();
    initDiagnostics();
    //initDynamicReconfigure();
    initParameters();
    initPublishers();
    initSubscribers();
    initTimers();
  }

private:

  void initDiagnostics()
  {
    // Diagnostics
    diag_updater.add("AscTecAdapter Status", this, &AscTecHLInterfaceAdapterNodelet::diagnostics);
    diag_updater.add(freq_status);
    diag_updater.setHardwareID("none");
    diag_updater.force_update();
  }

  void initDynamicReconfigure()
  {
    // Dynamic Reconfigure Server
    dynamic_reconfigure::Server<starmac_robots_asctec::AscTecAdapterConfig>::CallbackType f =
        boost::bind(&AscTecHLInterfaceAdapterNodelet::cfgCallback, this, _1, _2);
    dyn_reconfig_srv.setCallback(f);
  }

  void initParameters()
  {
    // Parameters
    //nh_priv.param("output_topic", output_topic, output_topic);
    //nh_priv.param("estop_topic", estop_topic, estop_topic);
    double temp = deadman_timeout.toSec();
    nh_priv.param("deadman_timeout", temp, temp);
    deadman_timeout.fromSec(temp);
    nh_priv.param("alt_KP", alt_KP, alt_KP);
    nh_priv.param("alt_KI", alt_KI, alt_KI);
    nh_priv.param("alt_KD", alt_KD, alt_KD);
    nh_priv.param("alt_KDD", alt_KDD, alt_KDD);
    nh_priv.param("alt_Ilimit", alt_Ilimit, alt_Ilimit);
    nh_priv.param("yaw_KP", yaw_KP, yaw_KP);
    nh_priv.param("yaw_KI", yaw_KI, yaw_KI);
    nh_priv.param("yaw_KD", yaw_KD, yaw_KD);
    nh_priv.param("yaw_Ilimit", yaw_Ilimit, yaw_Ilimit);
    nh_priv.param("yaw_rate_limit", yaw_rate_limit, yaw_rate_limit);
    nh_priv.param("roll_slew_rate_limit", roll_slew_rate_limit, roll_slew_rate_limit);
    nh_priv.param("pitch_slew_rate_limit", pitch_slew_rate_limit, pitch_slew_rate_limit);
    nh_priv.param("yaw_slew_rate_limit", yaw_slew_rate_limit, yaw_slew_rate_limit);
    nh_priv.param("alt_slew_rate_limit", alt_slew_rate_limit, alt_slew_rate_limit);
    nh_priv.param("nominal_thrust", nominal_thrust, nominal_thrust);
    nh_priv.param("thrust_mult", thrust_mult, thrust_mult);
    nh_priv.param("max_thrust", max_thrust, max_thrust);
    nh_priv.param("thrust_autoadjust", thrust_autoadjust, thrust_autoadjust);
    nh_priv.param("accel_bias", accel_bias, accel_bias);
    int temp_uint; // ROS parameters can't be unsigned
    nh_priv.param("land_now_thrust_decrease_rate", temp_uint, (int)land_now_thrust_decrease_rate);
    land_now_thrust_decrease_rate = (unsigned int)temp_uint;
    nh_priv.param("land_now_min_thrust_ratio", land_now_min_thrust_ratio, land_now_min_thrust_ratio);
    nh_priv.param("pitch_trim", pitch_trim, pitch_trim);
    nh_priv.param("roll_trim", roll_trim, roll_trim);
    nh_priv.param("yaw_offset", yaw_offset, yaw_offset);
    // Altitude and Yaw PID control
    setAltControllerParams(alt_KP, alt_KI, alt_KD, alt_Ilimit);
    setYawControllerParams(yaw_KP, yaw_KI, yaw_KD, yaw_Ilimit);
  }

  void initPublishers()
  {
    // Publishers
    output = nh.advertise<asctec_hl_comm::mav_ctrl> ("fcu/control", 10);
    //estop_pub = nh.advertise<std_msgs::Bool> ("asctec/ESTOP", 10);
    imu_pub = nh.advertise<sensor_msgs::Imu> ("asctec/imu", 10);
  }

  void initTimers()
  {
    // Timers
    double deadman_timer_period = deadman_timeout.toSec() / 2;
    min_freq = 0.95 / deadman_timer_period;
    max_freq = 1.05 / deadman_timer_period;
    deadman_timer = nh.createTimer(ros::Duration(deadman_timer_period),
                                   &AscTecHLInterfaceAdapterNodelet::deadmanCallback, this);
  }

  void initSubscribers()
  {
    // Subscribers
    motor_enable_sub = nh.subscribe("teleop_flyer/motor_enable", 1,
                                    &AscTecHLInterfaceAdapterNodelet::motorEnableCallback, this,
                                    ros::TransportHints().tcpNoDelay());
    control_input_sub = nh.subscribe("controller_mux/output", 1,
                                     &AscTecHLInterfaceAdapterNodelet::controlInputCallback, this,
                                     ros::TransportHints().tcpNoDelay());
    state_sub = nh.subscribe("odom", 1, &AscTecHLInterfaceAdapterNodelet::stateCallback, this,
                             ros::TransportHints().tcpNoDelay());

    mav_status_sub = nh.subscribe("fcu/status", 1, &AscTecHLInterfaceAdapterNodelet::mavStatusCallback, this,
                                  ros::TransportHints().tcpNoDelay());
    //    imu_sub = nh.subscribe("asctec/IMU_CALCDATA", 1, &AscTecMavFrameworkAdapterNodelet::imuCallback, this,
    //                           ros::TransportHints().tcpNoDelay());
    land_now_sub = nh_priv.subscribe("land_now", 10, &AscTecHLInterfaceAdapterNodelet::landNowCallback, this,
                                     ros::TransportHints().tcpNoDelay());
  }

  void cfgCallback(starmac_robots_asctec::AscTecAdapterConfig &config, uint32_t level)
  {
    ROS_INFO("Reconfigure request");
    if (isSafeToReconfigure())
    {
      setYawControllerParams(config.yaw_KP, config.yaw_KI, config.yaw_KD, config.yaw_Ilimit);
      roll_trim = config.roll_trim;
      pitch_trim = config.pitch_trim;
    }
    else
    {
      ROS_ERROR("Cannot reconfigure; isSafeToReconfigure() returned false");
    }
  }

  void setAltControllerParams(double KP, double KI, double KD, double Ilimit)
  {
    alt_pid.initPid(KP, KI, KD, Ilimit, -Ilimit);
    ROS_INFO("Altitude controller gains set: KP=%f, KI=%f, KD=%f, Ilimit=%f", KP, KI, KD, Ilimit);
  }

  void setYawControllerParams(double KP, double KI, double KD, double Ilimit)
  {
    yaw_pid.initPid(KP, KI, KD, Ilimit, -Ilimit);
    ROS_INFO("Yaw controller gains set: KP=%f, KI=%f, KD=%f, Ilimit=%f", KP, KI, KD, Ilimit);
  }

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    if (deadman)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Deadman Tripped");
    }
    else
    {
      if (min_battery_voltage < 10.0)
      {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Low Battery");
        if (not battery_low_warning_sent)
        {
          ROS_WARN("Battery voltage below 10V");
          battery_low_warning_sent = true;
        }
      }
      else
      {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
      }
    }
    stat.add("Motor Enable", latest_motor_enable);
    stat.add("Deadman Tripped", deadman);
    stat.add("max ll_status age", max_mav_status_age);
    stat.add("max control inputs age", max_control_input_age);
    stat.add("max motor_enable age", max_motor_enable_age);
    stat.add("max state age", max_state_age);
    stat.add("alt P term", alt_p_term);
    stat.add("alt I term", alt_i_term);
    stat.add("alt D term", alt_d_term);
    stat.add("alt DD term", alt_dd_term);
    stat.add("Battery Voltage (current)", latest_battery_voltage);
    stat.add("Battery Voltage (minimum)", min_battery_voltage);
  }

  void motorEnableCallback(const std_msgs::BoolConstPtr& msg)
  {
    static bool first = true;
    latest_motor_enable = msg->data;
    ros::Time now = ros::Time::now();
    if (!first)
    {
      max_motor_enable_age = max(now - last_motor_enable_time, max_motor_enable_age);
    }
    else
    {
      first = false;
    }
    last_motor_enable_time = now;
  }

  void stateCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    static bool first = true;
    latest_state = *msg;
    ROS_DEBUG_STREAM("Got state with z = " << latest_state.pose.pose.position.z);
    ros::Time now = ros::Time::now();
    if (!first)
    {
      max_state_age = max(now - last_state_time, max_state_age);
    }
    else
    {
      first = false;
      got_first_state = true;
    }
    last_state_time = now;
  }

  //  TODO: replace IMU callback (or is it necessary anymore??)
  //  void imuCallback(const asctec_msgs::IMUCalcDataConstPtr& msg)
  //  {
  //    //    static double avg_accel = 0;
  //    //    static int n_meas = 0;
  //    ros::Time now = ros::Time::now();
  //    last_imu_time = now;
  //    // calculate quaternion orientation - minus signs convert to ENU frame
  //    btQuaternion orientation;
  //    orientation.setRPY(msg->angle_roll * ASC_TO_ROS_ANGLE * -1.0, msg->angle_nick * ASC_TO_ROS_ANGLE,
  //                       msg->angle_yaw * ASC_TO_ROS_ANGLE * -1.0);
  //
  //    btMatrix3x3 R_enu_imu(orientation);
  //
  //    btVector3 acc_imu(msg->acc_x_calib * ASC_TO_ROS_ACC, msg->acc_y_calib * ASC_TO_ROS_ACC,
  //                      msg->acc_z_calib * ASC_TO_ROS_ACC);
  //    btVector3 acc_enu = R_enu_imu * acc_imu;
  //    //    ROS_INFO("acc_inertial = %g, %g, %g", acc_inertial[0], acc_inertial[1], acc_inertial[2]);
  //    last_z_accel = acc_enu[2] + accel_bias; // -ve values = ascending
  //    //    ROS_INFO("last_z_accel = %g", last_z_accel);
  //    //    ROS_INFO("norm of accel = %g (raw), %g (rotated), %g (avg z)", acc_imu.length(), acc_inertial.length(), avg_accel);
  //    //    n_meas++;
  //    //    avg_accel = (avg_accel * (n_meas - 1) + acc_inertial[2]) / n_meas;
  //
  //    sensor_msgs::Imu imu_msg;
  //
  //    imu_msg.header.stamp = msg->header.stamp;
  //    imu_msg.header.frame_id = ros::this_node::getNamespace() + "/imu";
  //
  //    imu_msg.linear_acceleration.x = msg->acc_x_calib * ASC_TO_ROS_ACC;
  //    imu_msg.linear_acceleration.y = msg->acc_y_calib * ASC_TO_ROS_ACC;
  //    imu_msg.linear_acceleration.z = msg->acc_z_calib * ASC_TO_ROS_ACC;
  //
  //    for (int i = 0; i < 8; i++)
  //    {
  //      imu_msg.linear_acceleration_covariance[i] = 0.0;
  //      imu_msg.angular_velocity_covariance[i] = 0.0;
  //    }
  //
  //    imu_msg.angular_velocity.x = msg->angvel_roll * ASC_TO_ROS_ANGVEL * -1.0;
  //    imu_msg.angular_velocity.y = msg->angvel_nick * ASC_TO_ROS_ANGVEL;
  //    imu_msg.angular_velocity.z = msg->angvel_yaw * ASC_TO_ROS_ANGVEL * -1.0;
  //
  //    imu_msg.orientation.x = orientation.getX();
  //    imu_msg.orientation.y = orientation.getY();
  //    imu_msg.orientation.z = orientation.getZ();
  //    imu_msg.orientation.w = orientation.getW();
  //
  //    imu_pub.publish(imu_msg);
  //  }

  void landNowCallback(const std_msgs::BoolConstPtr& msg)
  {
    land_now = msg->data;
  }

  void mavStatusCallback(const asctec_hl_comm::mav_statusConstPtr& msg)
  {
    latest_mav_status = *msg;
    ros::Time now = ros::Time::now();
    if (mav_status_cb_called)
    {
      max_mav_status_age = max(now - last_mav_status_time, max_mav_status_age);
    }
    else
    {
      min_battery_voltage = 9999;
      mav_status_cb_called = true;
    }
    last_mav_status_time = now;
    latest_battery_voltage = (double)latest_mav_status.battery_voltage;
    min_battery_voltage = min(latest_battery_voltage, min_battery_voltage);
  }

  void controlInputCallback(const flyer_controller::control_mode_outputConstPtr& msg)
  {
    ros::Time now = ros::Time::now();
    ros::Duration dt = now - prev_control_input_time;
    static bool prev_motor_enable = false;
    if (got_first_control_input)
    {
      max_control_input_age = max(dt, max_control_input_age);
    }
    else
    {
      got_first_control_input = true;
    }

    // Slew limiting
    double roll_cmd_limited, pitch_cmd_limited, yaw_cmd_limited, alt_cmd_limited;
    double dt_sec = dt.toSec();
    // TODO: Think about whether when limiting occurs, it should occur on all axes proportionally
    // or at least, if roll is being limited by some factor x, then pitch should also, otherwise the overall
    // direction of the commanded thrust vector will be different
    limitSlewRate(msg->roll_cmd, prev_control_input_msg.roll_cmd, dt_sec, roll_slew_rate_limit, roll_cmd_limited);
    //    if (roll_cmd_limited != msg.roll_cmd) {
    //      ROS_INFO_STREAM("Roll command was slew rate limited. Orig cmd: " << msg.roll_cmd << "; New cmd: " << roll_cmd_limited);
    //    }
    limitSlewRate(msg->pitch_cmd, prev_control_input_msg.pitch_cmd, dt_sec, pitch_slew_rate_limit, pitch_cmd_limited);

    if (not msg->direct_yaw_rate_commands)
      limitSlewRate(msg->yaw_cmd, prev_control_input_msg.yaw_cmd, dt_sec, yaw_slew_rate_limit, yaw_cmd_limited);

    if (not msg->direct_thrust_commands)
      limitSlewRate(msg->alt_cmd, prev_control_input_msg.alt_cmd, dt_sec, alt_slew_rate_limit, alt_cmd_limited);

    if (latest_motor_enable and not prev_motor_enable)
    {
      double pe, de, ie;
      yaw_pid.getCurrentPIDErrors(&pe, &de, &ie);
      ROS_INFO_STREAM("Yaw integrator was at:" << ie);
      alt_pid.reset();
      yaw_pid.reset();
      ROS_INFO("Reset yaw & alt integrators");
    }
    prev_motor_enable = latest_motor_enable;
    double thrust_cmd = 0; // [N]
    if (got_first_state)
    {
      if (not msg->direct_thrust_commands)
      {
        //ROS_DEBUG("Running computeThrust, alt_cmd_limited = %f, dt = %f", alt_cmd_limited, dt.toSec());
        thrust_cmd = computeThrust(alt_cmd_limited, dt);
      }
      else
      {
        double yaw, pitch, roll;
        getLatestYPR(yaw, pitch, roll);
        double tfactor = 1;
        if (thrust_autoadjust)
          tfactor = 1 / (cos(pitch) * cos(roll));
        thrust_cmd = tfactor * msg->thrust_cmd;
      }
    }
    else
    {
      ROS_DEBUG("got_first_state was false..");
    }
    double yaw_rate_cmd = 0; // [deg/s]
    if (not land_now)
    {
      // Don't want to try and control yaw during rapid landing since once on the ground
      // (which we can't assume we have a way of sensing, since we might be landing precisely
      //  *because* we've lost our state estimate) the yaw controller will try hard to
      // take out yaw error resulting in one pair of motors spinning up.
      if (msg->direct_yaw_rate_commands)
        yaw_rate_cmd = msg->yaw_rate_cmd;
      else if (got_first_state)
        yaw_rate_cmd = computeYawRate(yaw_cmd_limited, dt);
    }
    else
    {
      // Also zero out pitch and roll during rapid landing
      roll_cmd_limited = roll_trim;
      pitch_cmd_limited = pitch_trim;
    }
    asctec_hl_comm::mav_ctrlPtr ctrl_msg(new asctec_hl_comm::mav_ctrl);
    ctrl_msg->type = asctec_hl_comm::mav_ctrl::acceleration;
    ctrl_msg->header.stamp = now;
    double roll_cmd_rotated, pitch_cmd_rotated; // deg
    rotateRollAndPitch(roll_cmd_limited, pitch_cmd_limited, roll_cmd_rotated, pitch_cmd_rotated);
    roll_cmd_rotated += roll_trim;
    pitch_cmd_rotated += pitch_trim;
    if (!deadman && dt.toSec() < 1.0)
    {
      bool motors = msg->motors_on and latest_motor_enable;
      if (not motors)
      {
        thrust_cmd = 0;
      }
      assemble_command(roll_cmd_rotated, pitch_cmd_rotated, yaw_rate_cmd, thrust_cmd, ctrl_msg);
    }
    if (deadman)
    {

      // TODO: call motor off service if deadman tripped (?)
      //      if (not estop_sent)
      //      {
      //        std_msgs::Bool estop_msg;
      //        estop_msg.data = true;
      //        estop_pub.publish(estop_msg);
      //        estop_sent = true;
      //      }
      //      if (latest_mav_status.flying == 1)
      //        ctrl_msg = zero_thrust_full_left_yaw; // send motor shutoff
      //      else
      //        ctrl_msg = assemble_command(roll_cmd_limited, pitch_cmd_limited, yaw_rate_cmd, thrust_cmd, false, false, false,
      //                                    false); // Send roll, pitch, yaw values but don't enable those bits
    }
    ctrl_msg->header.stamp = now;
    output.publish(ctrl_msg);
    prev_control_input_time = now;
    prev_control_input_msg = *msg;
  }

  void rotateRollAndPitch(const double roll_cmd, const double pitch_cmd, double& roll_cmd_rotated,
                          double& pitch_cmd_rotated)
  {
    double cy = cos(angles::from_degrees(yaw_offset));
    double sy = sin(angles::from_degrees(yaw_offset));
    roll_cmd_rotated = cy * roll_cmd - sy * pitch_cmd;
    pitch_cmd_rotated = sy * roll_cmd + cy * pitch_cmd;
  }

  double computeThrust(const double alt_cmd, ros::Duration& dt)
  {
    // TODO: probably shouldn't have statics here, they should be member vars
    // (though as long as there's only one instance of AscTecAdapter in the program
    //  it's really just academic)
    static bool landing_now = false;
    static int land_now_start_thrust = 0;
    static ros::Time land_now_start_time;
    if (land_now)
    {
      if (!landing_now)
      {
        ROS_INFO("Received land_now instruction. Decreasing thrust at %d counts/s to %d counts",
                 land_now_thrust_decrease_rate, int(land_now_min_thrust_ratio * double(nominal_thrust)));
        land_now_start_thrust = last_thrust;
        land_now_start_time = ros::Time::now();
        landing_now = true;
      }
      last_thrust = max(
                        land_now_min_thrust_ratio * nominal_thrust,
                        land_now_start_thrust - (ros::Time::now() - land_now_start_time).toSec()
                            * land_now_thrust_decrease_rate);
    }
    else
    {
      if (landing_now)
      {
        // We were landing but not anymore, so reset things:
        landing_now = false;
      }
      double alt_err = (-latest_state.pose.pose.position.z) - alt_cmd;
      double alt_vel_err = -latest_state.twist.twist.linear.z;
      alt_pid.updatePid(alt_err, alt_vel_err, dt);
      double pe, ie, de;
      alt_pid.getCurrentPIDErrors(&pe, &ie, &de);
      alt_p_term = alt_KP * pe;
      alt_i_term = alt_KI * ie;
      alt_d_term = alt_KD * de;
      alt_dd_term = alt_KDD * last_z_accel;
      double yaw, pitch, roll;
      getLatestYPR(yaw, pitch, roll);
      double tfactor = 1;
      if (thrust_autoadjust)
        tfactor = 1 / (cos(pitch) * cos(roll));
      last_thrust = tfactor * (nominal_thrust + alt_pid.getCurrentCmd() + alt_dd_term);
    }
    return last_thrust;
  }

  double computeYawRate(double yaw_cmd, // deg
                        ros::Duration& dt)
  {
    double yaw, pitch, roll; // rad
    getLatestYPR(yaw, pitch, roll);
    //double yaw_err = angles::to_degrees(yaw) - yaw_cmd;
    double yaw_err = angles::to_degrees(
                                        angles::shortest_angular_distance(angles::from_degrees(yaw_cmd),
                                                                          yaw + angles::from_degrees(yaw_offset))); // deg
    //ROS_INFO_STREAM("Yaw error = " << yaw_err << " [deg]");
    yaw_pid.updatePid(yaw_err, dt);
    double yaw_rate_cmd = min(yaw_rate_limit, max(-yaw_rate_limit, yaw_pid.getCurrentCmd())); // deg/s
    //ROS_INFO_STREAM("Yaw rate command = " << yaw_rate_cmd << " [deg/s]");
    return yaw_rate_cmd;
  }

  void getLatestYPR(double& yaw, // rad
                    double& pitch, // rad
                    double& roll // rad
  )
  {
    tf::Quaternion temp_q;
    //    ROS_INFO("About to do tf::quaternionMsgToTF..");
    //    ROS_INFO_STREAM("orientation: " << latest_state.pose.pose.orientation.x << " "
    //        << latest_state.pose.pose.orientation.y << " " << latest_state.pose.pose.orientation.z << " "
    //        << latest_state.pose.pose.orientation.w);
    tf::quaternionMsgToTF(latest_state.pose.pose.orientation, temp_q);
    //    ROS_INFO("done tf::quaternionMsgToTF.. ");
    tf::Matrix3x3 temp_mat = tf::Matrix3x3(temp_q);
    temp_mat.getEulerYPR(yaw, pitch, roll);
  }

  void assemble_command(double roll, // deg
                        double pitch, // deg
                        double yaw_rate, // deg/s
                        double thrust, // N,
                        asctec_hl_comm::mav_ctrlPtr ctrl_msg)
  {
    ctrl_msg->y = -float(min(MAX_ROLL_CMD, max(-MAX_ROLL_CMD, roll * ROLL_SCALE)));
    ctrl_msg->x = -float(min(MAX_PITCH_CMD, max(-MAX_PITCH_CMD, pitch * PITCH_SCALE)));
    ctrl_msg->yaw = -float(min(MAX_YAW_RATE_CMD, max(-MAX_YAW_RATE_CMD, yaw_rate * YAW_SCALE)));
    ctrl_msg->z = float(min(MAX_THRUST_CMD, min(max_thrust, max(0.0, thrust * thrust_mult * THRUST_SCALE))));
  }

  void deadmanCallback(const ros::TimerEvent& e)
  {
    // TODO: figure out what to do for deadman now..
    //    // This method, called periodically by a timer, checks for the various conditions that
    //    // result in a 'dead man' determination:
    //    // - motor_enable signal from teleop_flyer too old
    //    // - control inputs too old
    //    // - ll_status from asctec too old
    //    // - state estimate too old
    //    ros::Time now = ros::Time::now();
    //    //    double motor_enable_age = (now - last_motor_enable_time).toSec();
    //    //    double control_inputs_age = (now - last_control_input_time).toSec();
    //    //    double ll_status_age = (now - last_ll_status_time).toSec();
    //    //    double state_estimate_age = (now - last_state_time).toSec();
    //    string deadman_timeout_reason;
    //    ros::Duration deadman_timeout_dt(0);
    //    freq_status.tick();
    //
    //    if (max_motor_enable_age > deadman_timeout)
    //    {
    //      deadman = true;
    //      deadman_timeout_reason = "motor enable";
    //      deadman_timeout_dt = max_motor_enable_age;
    //    }
    //    if (max_control_input_age > deadman_timeout)
    //    {
    //      deadman = true;
    //      deadman_timeout_reason = "control inputs";
    //      deadman_timeout_dt = max_control_input_age;
    //    }
    //    if (max_mav_status_age > deadman_timeout)
    //    {
    //      deadman = true;
    //      deadman_timeout_reason = "LL_Status";
    //      deadman_timeout_dt = max_mav_status_age;
    //    }
    //    if (max_state_age > deadman_timeout)
    //    {
    //      deadman = true;
    //      deadman_timeout_reason = "state";
    //      deadman_timeout_dt = max_state_age;
    //    }
    //    // Otherwise, if deadman conditions are no longer present,
    //    // and the ll_status (which must be recent if the above is true)
    //    // is indicating that the vehicle is not flying
    //    // (i.e. motors not spinning), then reset the deadman condition.
    //
    //    else if (latest_mav_status.motor_status == 0)
    //    {
    //      deadman = false;
    //    }
    //    if (deadman)
    //    {
    //      ROS_ERROR_STREAM(
    //                       "Deadman timeout exceeded(" << deadman_timeout_reason << "), dt = "
    //                           << deadman_timeout_dt.toSec());
    //    }
    diag_updater.update();
  }

  bool isSafeToReconfigure()
  {
    return (not latest_motor_enable); // for now this should suffice
  }
};
PLUGINLIB_DECLARE_CLASS(starmac_robots_asctec, AscTecHLInterfaceAdapterNodelet, starmac_robots_asctec::AscTecHLInterfaceAdapterNodelet, nodelet::Nodelet)

} // namespace

