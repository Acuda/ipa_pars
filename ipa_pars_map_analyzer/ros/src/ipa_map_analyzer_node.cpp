#include <ros/ros.h>
#include <ipa_pars_map_analyzer_client/ipa_pars_map_analyzer_client.h>

int main(int argc, char **argv)
{
    ros::init (argc, argv, "ipa_pars_map_analyzer_client_node");
    MapAnalyzer *mA = new MapAnalyzer();

    if(!mA->initialize())
    {
        ROS_ERROR("Failed to initialize MapAnalyzerClient");
        return -1;
    }

    ros::spin();
    delete mA;
    return 0;
}
