//
// Created by student on 10/7/19.
//

#include <obstacle_reporter/ObstacleReporter.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_reporter_node");

    ros::NodeHandlePtr nh(new ros::NodeHandle);
    int pubRate = 0;

    nh->param("PublishRate", pubRate, 10);

    try {
        std::unique_ptr<ObstacleReporter> Reporter(new ObstacleReporter(
            pubRate,
            nh));
        ROS_WARN("Starting Reporter");
        Reporter->start();

    } catch (std::exception& exception) {
        ROS_FATAL_STREAM("Fauled to start the reporter. Exception: " <<
        exception.what());
        return 1;
    }
    return 0;
}