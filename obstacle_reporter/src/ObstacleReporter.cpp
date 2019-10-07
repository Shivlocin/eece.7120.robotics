//
// Created by student on 10/7/19.
//

#include <obstacle_reporter/ObstacleReporter.hpp>
#include <sstream>

ObstacleReporter::ObstacleReporter(int rate, ros::NodeHandlePtr nh)
:
m_cycleRate(rate),
m_nh(nh)
{
    m_sub = m_nh->subscribe<ReportObstacleHandle>(
        "registered_obstacles",
        10,
        &ObstacleReporter::newObstacleCallback,
        this);
}

void ObstacleReporter::start()
{
    m_running = true;
    while(ros::ok() && m_running) {
        ros::spinOnce();
        m_cycleRate.sleep();
    }
}

void ObstacleReporter::newObstacleCallback(ReportObstacleHandle msg)
{
    if(msg->valid) {
        ROS_INFO_STREAM("Detected an obstacle with id: " << msg->id <<
                        " at position: " <<
                        "\n\t x: " << msg->position.x <<
                        "\n\t y: " << msg->position.y <<
                        "\n\t z: " << msg->position.z);
    }
}
