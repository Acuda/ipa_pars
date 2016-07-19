#include <ros/ros.h>
#include <ipa_pars_map_analyzer/ipa_pars_map_analyzer_client.h>

int main(int argc, char **argv)
{
    ros::init (argc, argv, "ipa_pars_map_analyzer_client_node");
    ParsMapAnalyzerClient *pmac = new ParsMapAnalyzerClient(argv[1]);

    if(!pmac->initialize())
    {
        ROS_ERROR("Failed to initialize ParsMapAnalyzerClient");
        return -1;
    }

    pmac->run();
    delete pmac;
    return 0;
}
