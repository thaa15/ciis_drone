#ifndef COPTER_CIIS_H_
#define COPTER_CIIS_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/StreamRate.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/OverrideRCIn.h"
#include "sensor_msgs/TimeReference.h"

#include "geographic_msgs/GeoPoseStamped.h"
#include "ciis_drone/Motor.h" 

class CopterCIIS {
  public:
    CopterCIIS(ros::NodeHandle& nh, ros::Rate& rate);
    ~CopterCIIS();

    // Movement
    void point_move(const float x, const float y, const float z);
    void global_move(const float latitude, const float longitude, const float altitude);

    void arm();
    void disarm();
    void goFlight();

    // Position 
    void local_pos_pose_cb(const geometry_msgs::PoseStamped& data);
    void rates_attitude_cb(const geometry_msgs::PoseStamped& data);

    // MAVROS STATE
    void state_cb(const mavros_msgs::State& msg);
    void pwm_cb(const ciis_drone::Motor& data);

    bool armed_state,guided_state, manual_state;
    bool cmd_state,flight_status = true,conn_state;
    float pos_x = 0, pos_y = 0, pos_z = 0, alt_trgt;
    float roll = 0, pitch = 0, yaw = 0;
    std::string mode_state;

  private:
    ros::NodeHandle _nh;
    ros::Rate _rate;
    mavros_msgs::OverrideRCIn actuator_output_msg;
    // Subscriber
    ros::Subscriber state_sub;
    ros::Subscriber local_pos_pose_sub;
    ros::Subscriber pwm_ref_sub;
    ros::Subscriber rates_attitude_sub;

    // Service Client
    ros::ServiceClient command_arm_cli; 
    ros::ServiceClient command_land_cli;
     // Publisher 
    ros::Publisher setpoint_position_pub;
    ros::Publisher setpoint_global_pub;
    ros::Publisher actuator_outputs_pub;
};


#endif