#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manager.h>

using namespace warch_plan;

//define dynamic call back function
// void paramCallback(bz_plan::dynamicParamConfig& config)
// {
//     ROS_INFO("Request: %s", config.is_use_fm? "is_use_fm = True":"is_use_astar = Ture");
//     // planner.setUseFmFlag(config.is_use_fm);
// }
int main(int argc, char** argv)
{
    ros::init(argc, argv, "warch_plan_node");
    ros::NodeHandle nh("~");

    PlanManager warch_planner; 
    warch_planner.init(nh);

    // dynamic_reconfigure::Server<bz_plan::dynamicParamConfig> server;
    // dynamic_reconfigure::Server<bz_plan::dynamicParamConfig>::CallbackType f_paramCallback;
    // f_paramCallback = boost::bind(&paramCallback,_1);
    // server.setCallback(f_paramCallback);

    // // ros::Duration(1.0).sleep();
    // ros::spin();

    // return 0;

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();           
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}
