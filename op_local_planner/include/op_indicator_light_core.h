
#ifndef OP_TRAFFIC_LIGHT_CORE
#define OP_TRAFFIC_LIGHT_CORE

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include "vector_map_msgs/LaneArray.h"
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/VehicleCmd.h>
#include <std_msgs/Int32.h>
#include "op_planner/PlannerCommonDef.h"
#include "op_planner/DecisionMaker.h"
#include "op_planner/PlanningHelpers.h"
#include "op_utility/DataRW.h"

#define JUNCTION_DISTANCE_LIMIT 5.0

namespace IndicatorLightNS
{

enum INDICATOR_TYPE
{
	LETF,
	RIGHT,
	OFF,
	EMERGENCY
};

class IndicatorLight
{
    protected:
        //Control Related
	    int m_ControlFrequency;

        int m_CurrentIndicatorState = 0;
        int m_CurrentLaneId = -1;
        int m_turnMode = 0; //  1 = left, 2 = right
        std::vector<int> m_nextJunctionLaneIds;
        bool bApproachingJunction = false;
        bool bInJunction = false;
        bool bIndicateTurn = false;

        // parameter
        double m_DistanceToJunction = 3.0;

        geometry_msgs::Pose m_OriginPos;
        PlannerHNS::WayPoint m_CurrentPos;
        bool bNewCurrentPos;

        PlannerHNS::MAP_SOURCE_TYPE m_MapType;
        std::string m_MapPath;
        PlannerHNS::RoadNetwork m_Map;
        bool bMap = false;

        ros::NodeHandle nh;

        ros::Subscriber sub_current_pose;
        ros::Subscriber sub_GlobalPlannerPaths;
        ros::Subscriber sub_vehicle_cmd;
        
        ros::Publisher  pub_indicator_state;

        std::vector<PlannerHNS::WayPoint> m_temp_path;
	    std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPaths;
	    std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPathsToUse;

        void callbackVehicleCommand(const autoware_msgs::VehicleCmd& msg);
        void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
        void callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);
        void SendIndicatorLightTopics(void);
        int GetCurrentIndicatorLightState(void);
        int TurningDirFromAngles(const double& a1, const double& a2);

    public:
        IndicatorLight();
        ~IndicatorLight();
        void MainLoop(void);

};

}

#endif // OP_TRAFFIC_LIGHT_CORE
