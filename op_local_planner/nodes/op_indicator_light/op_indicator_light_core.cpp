#include "op_indicator_light_core.h"
#include "op_ros_helpers/op_ROSHelpers.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/OpenDriveMapLoader.h"


namespace IndicatorLightNS
{

IndicatorLight::IndicatorLight()
{
	m_ControlFrequency = 10;

    ros::NodeHandle _nh;

    tf::StampedTransform transform;
	tf::TransformListener tf_listener;
	PlannerHNS::ROSHelpers::getTransformFromTF("world", "map", tf_listener, transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();

    sub_current_pose 		= 	nh.subscribe("/current_pose", 1,	&IndicatorLight::callbackGetCurrentPose, this);
	sub_GlobalPlannerPaths 	= 	nh.subscribe("/lane_waypoints_array", 1, &IndicatorLight::callbackGetGlobalPlannerPath, this);
	sub_vehicle_cmd 		=   nh.subscribe("/op_controller_cmd", 1, &IndicatorLight::callbackVehicleCommand, this);

    pub_indicator_state = nh.advertise<std_msgs::Int32>("indicator_control", 1);
    
    int iSource = 0;
	nh.getParam("/op_common_params/mapSource" , iSource);
	nh.getParam("/op_common_params/mapFileName" , m_MapPath);
    if(iSource == 6)
		m_MapType = PlannerHNS::MAP_OPEN_DRIVE_FILE;
}

IndicatorLight::~IndicatorLight()
{
}

void IndicatorLight::MainLoop(){

	ros::Rate loop_rate(m_ControlFrequency);

	while (ros::ok())
	{

		ros::spinOnce();
		if(m_MapType == PlannerHNS::MAP_OPEN_DRIVE_FILE && !bMap)
		{
			bMap = true;
			PlannerHNS::OpenDriveMapLoader xodr_loader;
			xodr_loader.EnableLaneStitching();
			xodr_loader.LoadXODR(m_MapPath, m_Map);
		}
		m_CurrentIndicatorState = IndicatorLight::GetCurrentIndicatorLightState();
		SendIndicatorLightTopics();

		loop_rate.sleep();
	}
}

void IndicatorLight::SendIndicatorLightTopics() {
	std_msgs::Int32 temp;
	temp.data = m_CurrentIndicatorState;
	pub_indicator_state.publish(temp);
}

int IndicatorLight::GetCurrentIndicatorLightState() {
	int temp_indicatorState = 0;

	if( m_GlobalPaths.size() > 0 && bNewCurrentPos) {
		bNewCurrentPos = false;
		

		PlannerHNS::WayPoint *pPoint = 0;
		PlannerHNS::WayPoint *pTempPoint = 0;
		PlannerHNS::Lane* pLane = 0;
		pPoint = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(m_CurrentPos, m_Map);
		pLane = PlannerHNS::MappingHelpers::GetClosestLaneFromMap(m_CurrentPos, m_Map, 1, true);
		if(pLane){
			if(pLane->toLanes.size() > 1){
				// bApproachingJunction = false;
				// bIndicateTurn = false;
				// bInJunction = false;
				if(pPoint){
					m_DistanceToJunction = INT_MAX;
					for(pTempPoint = pPoint; pTempPoint->pFronts.size() < 2; pTempPoint = pTempPoint->pFronts.at(0)) {
						// iterate trough all following waypoints until a junction is reached
						m_DistanceToJunction = hypot(m_CurrentPos.pos.x - pTempPoint->pos.x, m_CurrentPos.pos.y - pTempPoint->pos.y);
					}
					if(pTempPoint->pFronts.size() > 1){
						for ( int i = 0; i < pTempPoint->pFronts.size(); i++ ){
							for ( int j = 0; j < pLane->toLanes.size(); j++ ){
								if(pTempPoint->pFronts.at(i)->pLane->id == pLane->toLanes.at(j)->id) {
									//found point matching a lane
									m_nextJunctionLaneIds.push_back(pLane->toLanes.at(j)->id);
								}
							}
						}
					}
					// ROS_INFO(">> Distance to junction: %f", m_DistanceToJunction);
				}
			}
			if(bInJunction == false){
				for(int i = 0; i < m_nextJunctionLaneIds.size(); i++) {
					if(m_nextJunctionLaneIds.at(i) == pLane->id) {
						bInJunction = true;
						bApproachingJunction = false;
						m_DistanceToJunction = INT_MAX;
					}
				}
			}
		}		
	
		// turn on indicator in front of junction
		if(m_DistanceToJunction < JUNCTION_DISTANCE_LIMIT && bApproachingJunction == false) {
			bApproachingJunction = true;
			m_DistanceToJunction = INT_MAX;
		}

		// turn on indicator 
		if(bApproachingJunction || bInJunction) {
			temp_indicatorState = 42;
			bIndicateTurn = true;
		}

		if(bIndicateTurn && pPoint && m_turnMode == 0) {
			// ROS_INFO("Checking turn direction based on global path");
			if( m_GlobalPaths.size() > 0) {
				PlannerHNS::RelativeInfo info;
				PlannerHNS::PlanningHelpers::GetRelativeInfoLimited(m_GlobalPaths.at(0), *pPoint, info, 0);
				// ROS_INFO("index: %d", info.iFront);
				// ROS_INFO("pose: %f, %f", m_GlobalPaths.at(0).at(info.iFront).pos.x, m_GlobalPaths.at(0).at(info.iFront).pos.y);
				int offset = 15;
				if((m_GlobalPaths.at(0).size() - 1) > (info.iFront + offset))
				{
					m_turnMode = IndicatorLight::TurningDirFromAngles(pPoint->pos.a, m_GlobalPaths.at(0).at(info.iFront + offset).pos.a);
				}
				else {
					ROS_ERROR("UNABLE TO FIND TURN DIRECTION!");
				}
			}
		}
	}

	if(bIndicateTurn){
		temp_indicatorState	= m_turnMode;
	}

	return temp_indicatorState;
}

int IndicatorLight::TurningDirFromAngles(const double& a1, const double& a2)
{
	// TODO: Make it work in long junctions
	// ROS_INFO(">> Angles heading: %f, %f", a1, a2);
	float angle1 = std::fmod(a1, 2 * M_PI);
	float angle2 = std::fmod(a2, 2 * M_PI);
 
	if(angle1 < 0)
		angle1 = 2 * M_PI + angle1;
	if(angle2 < 0)
		angle2 = 2 * M_PI + angle2;

	if(angle1 > M_PI)
		angle1 = (- 2 * M_PI) + angle1;
	if(angle2 > M_PI)
		angle2 = (- 2 * M_PI) + angle2;

	// ROS_INFO(">> Angles heading: %f, %f", angle1, angle2);

	double turning_angle = angle1 - angle2;

	// ROS_INFO(">> Angle Turn: %f", turning_angle);

  	if( turning_angle > 0.1 ) {
	  return 2;
	}
	if( turning_angle < -0.1 ) {
	  return 1;
	}

   return 0;
}

void IndicatorLight::callbackVehicleCommand(const autoware_msgs::VehicleCmd& msg)
{
  	if(abs(msg.steer_cmd.steer) < 5.0 && bIndicateTurn == true){
		bIndicateTurn = false;
		bInJunction = false;
		m_turnMode = 0;
	}
}

void IndicatorLight::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPos = PlannerHNS::WayPoint(msg->pose.position.x + m_OriginPos.position.x, msg->pose.position.y + m_OriginPos.position.y,
			msg->pose.position.z + m_OriginPos.position.z, tf::getYaw(msg->pose.orientation));
	bNewCurrentPos = true;
}

void IndicatorLight::callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0)
	{
		m_GlobalPaths.clear();
		for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
		{
			PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), m_temp_path);
			m_GlobalPaths.push_back(m_temp_path);
		}

		bool bOldGlobalPath = true;
		if(m_GlobalPathsToUse.size() == m_GlobalPaths.size())
		{
			for(unsigned int i=0; i < m_GlobalPaths.size(); i++)
			{
				bOldGlobalPath = PlannerHNS::PlanningHelpers::CompareTrajectories(m_GlobalPaths.at(i), m_GlobalPathsToUse.at(i));
			}
		}
		else
		{
			bOldGlobalPath = false;
		}

		if(!bOldGlobalPath)
		{
			for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
			{
				PlannerHNS::PlanningHelpers::CalcAngleAndCost(m_GlobalPaths.at(i));
			}
			std::cout << "Received New Global Paths Indicator Light! " << m_GlobalPaths.size() << std::endl;
		}
	}
}

}