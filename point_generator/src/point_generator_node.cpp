//
// Created by student on 10/2/19.
//
#include <ros/ros.h>
#include <point_generator/PointGenerator.hpp>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "point_generator_node");

    ros::NodeHandlePtr nh(new ros::NodeHandle());
    int pubRate = 0;

    nh->param("PublishRate", pubRate, 5);

    try {
        std::unique_ptr<PointGenerator> Generator(new PointGenerator(
            nh,
            pubRate));
        ROS_WARN("Starting PointGenerator");
        Generator->start();

    } catch (std::exception& exception) {
        ROS_FATAL_STREAM("Failed to start the generator. Exception: " <<
        exception.what());
        return 1;
    }
    return 0;
}