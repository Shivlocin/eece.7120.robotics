//
// Created by student on 10/6/19.
//

#include <obstacle_labeler/ObstacleLabeler.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_labeler_node");

    ros::NodeHandlePtr nh(new ros::NodeHandle());
    int pubRate = 0;

    nh->param("PublishRate", pubRate, 10);

    try {
        std::unique_ptr<ObstacleLabeler> Labeler(new ObstacleLabeler(
            pubRate,
            nh));
        ROS_WARN("Starting Labeler");
        Labeler->start();

    } catch (std::exception& exception) {
        ROS_FATAL_STREAM("Failed to start the labeler. Exception: " <<
        exception.what());
        return 1;
    }
    return 0;
}