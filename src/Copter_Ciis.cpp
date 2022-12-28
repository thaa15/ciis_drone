#include "Copter_Ciis.h"

// CONSTRUCTOR //
CopterCIIS::CopterCIIS(ros::NodeHandle& nh, ros::Rate& rate) : _nh(nh), _rate(rate) {
	state_sub = _nh.subscribe("/mavros/state", 10, &CopterCIIS::state_cb, this);
	local_pos_pose_sub = _nh.subscribe("/mavros/local_position/pose", 1, &CopterCIIS::local_pos_pose_cb, this);
    rates_attitude_sub = _nh.subscribe("/mavros/setpoint_attitude/target_attitude",1,&CopterCIIS::rates_attitude_cb,this);
    pwm_ref_sub = _nh.subscribe("/mavros/local_position/pose", 1, &CopterCIIS::pwm_cb, this);
	command_arm_cli = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	command_land_cli = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

	setpoint_position_pub = _nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	setpoint_global_pub = _nh.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10);
    actuator_outputs_pub = _nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

	while (ros::ok() && !conn_state){
		ROS_INFO_THROTTLE(4, "Waiting for FCU connection");
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}
    ROS_INFO("Heartbeat Found! FCU connection established");
    ros::Duration(2).sleep();
}

// DESTRUCTOR //
CopterCIIS::~CopterCIIS() {}

void CopterCIIS::state_cb(const mavros_msgs::State &msg){
	conn_state = msg.connected;
	armed_state = msg.armed;
	guided_state = msg.guided;
	manual_state = msg.manual_input;
	mode_state = msg.mode;

}

void CopterCIIS::pwm_cb(const ciis_drone::Motor& data){
    actuator_output_msg.channels[0] = data.pwm1;
    actuator_output_msg.channels[1] = data.pwm2;
    actuator_output_msg.channels[2] = data.pwm3;
    actuator_output_msg.channels[3] = data.pwm4;
    flight_status = data.flight;
}

void CopterCIIS::local_pos_pose_cb(const geometry_msgs::PoseStamped& data) {
	pos_x = data.pose.position.x;
	pos_y = data.pose.position.y;
	pos_z = data.pose.position.z;
}

void CopterCIIS::rates_attitude_cb(const geometry_msgs::PoseStamped& data) {
	pos_x = data.pose.position.x;
	pos_y = data.pose.position.y;
	pos_z = data.pose.position.z;
}

void CopterCIIS::arm(){
	mavros_msgs::CommandBool command_bool;
    command_bool.request.value = true;
    if (command_arm_cli.call(command_bool)) {
        ROS_INFO("Arm success");
    } else {
        ROS_WARN("Arm failed");
    }
}

void CopterCIIS::goFlight(){
	for(int i = 10; ros::ok() && i > 0; --i){
		actuator_outputs_pub.publish(actuator_output_msg);
	}
	while(ros::ok() && flight_status) {
        actuator_outputs_pub.publish(actuator_output_msg);
        // ROS_INFO("Copter mode set to %s", copter_mode.c_str());
        ros::spinOnce();
        _rate.sleep();
    }
}


void CopterCIIS::disarm(){
	mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;
	if (command_arm_cli.call(disarm_cmd)) {
		ROS_INFO("Disarm success");
	}
	else {
		ROS_WARN("Disarm failed");
	}
}

void CopterCIIS::point_move(const float x, const float y, const float z){
	geometry_msgs::PoseStamped setpoint_pos;
	float r_pos_to_dest,tolerance =0.2;
	ROS_INFO("Moving to %f, %f, %f", x, y, z);
	//setpoint_position.header.seq = seq_count;
    setpoint_pos.pose.position.x = x;
    setpoint_pos.pose.position.y = y;
    setpoint_pos.pose.position.z = z;
	for (int i = 10000; ros::ok() && i > 0; --i)
    {

      setpoint_position_pub.publish(setpoint_pos);

      float deltaX = abs(pos_x- x);
      float deltaY = abs(pos_y - y);
      float deltaZ = abs(pos_z - z);
      float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
      //ROS_INFO("Mag: %f", dMag);
	  if( dMag < tolerance)
      {
        break;
      }
      ros::spinOnce();
      ros::Duration(0.5).sleep();
      if(i == 1)
      {
        ROS_INFO("Failed to reach destination. Stepping to next task.");
      }
    }
	ROS_INFO("Finished Moving to %f, %f, %f", x, y, z);
}	

   

// void CopterCIIS::global_move(const float latitude, const float longitude, const float altitude){
// 	geographic_msgs::GeoPoseStamped setpoint_glo;
// 	ROS_INFO("Moving to Lat:%f, Long:%f, Alt: %f", latitude, longitude, altitude);

// 	setpoint_glo.pose.position.latitude = latitude;
// 	setpoint_glo.pose.position.longitude = longitude;
// 	setpoint_glo.pose.position.altitude = altitude;
//     float tolerance = 0.1;
// 	for (int i = 10000; ros::ok() && i > 0; --i)
// 		{

// 		setpoint_global_pub.publish(setpoint_glo);

// 		float deltaX = abs(gps_alt- altitude);
// 		float deltaY = abs(gps_lat - latitude);
// 		float deltaZ = abs(gps_long - longitude);
// 		//cout << " dx " << deltaX << " dy " << deltaY << " dz " << deltaZ << endl;
// 		float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
// 		//ROS_INFO("Mag: %f", dMag);
// 		ROS_INFO("MAG %f", dMag);
// 		if( dMag < tolerance)
// 		{
// 			break;
// 		}
// 		if (deltaX < 0.1 && deltaY < 0.1 && deltaZ < 0.1){break;}
// 		ros::spinOnce();
// 		ros::Duration(0.5).sleep();
// 		if(i == 1)
// 		{
// 			ROS_INFO("Failed to reach destination. Stepping to next task.");
// 		}
// 		}
// 	ROS_INFO("Moved to Lat:%f, Long:%f, Alt: %f", latitude, longitude, altitude);

// }

// void CopterCIIS::land(){
//   const float h = 0;
//   float tolerance = 0.10;
//   int counter = 0;

//   mavros_msgs::CommandTOL land_msg;
// 	land_msg.request.yaw = 0;
// 	land_msg.request.altitude = h;

// 	if(command_land_cli.call(land_msg) && land_msg.response.success) {
// 		ROS_INFO("Initiating LAND");
// 	}
// 	else {
// 		ROS_ERROR("LAND failed");
// 	}

// 	//sleep(7);

// 	while(ros::ok()) {
// 		ROS_INFO("POS Z: %f", pos_z);

// 		if((pos_z <  (tolerance)) || (counter > 2000)) {
// 		ROS_INFO("LAND completed");
// 		return;
// 		}
		
// 		ros::spinOnce();
// 		_rate.sleep();
// 	}	
// 	CopterCIIS::disarm();
// }

