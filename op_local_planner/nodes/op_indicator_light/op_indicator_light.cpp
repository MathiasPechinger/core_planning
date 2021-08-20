#include <ros/ros.h>
#include "../include/op_indicator_light_core.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "op_motion_predictor");
	IndicatorLightNS::IndicatorLight  indicator_light;
	indicator_light.MainLoop();
	return 0;
}