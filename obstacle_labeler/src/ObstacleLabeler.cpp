//
// Created by student on 10/6/19.
//

#include <obstacle_labeler/ObstacleLabeler.hpp>

ObstacleLabeler::ObstacleLabeler(const int rate, const ros::NodeHandlePtr& nh)
:
m_rate(rate),
m_nh(nh)
{
    m_pub = m_nh.get()->advertise<obstacle_labeler::ReportObstacle>
        ("registered_obstacles", 10);

    m_sub = m_nh.get()->subscribe<geometry_msgs::Point>(
        "obstacles_detected",
        10,
        &ObstacleLabeler::gotObstacleCallback,
        this);
}

void ObstacleLabeler::start()
{
    m_running = true;
    while( ros::ok() && m_running) {
        ros::spinOnce();
        ObstacleLabeler::obstacleHandler();
        m_rate.sleep();
    }
}

void
ObstacleLabeler::gotObstacleCallback(
    const geometry_msgs::Point::ConstPtr& msg)
{
    Obstacle temp;
    temp.x = msg->x;
    temp.y = msg->y;
    temp.z = msg->z;
    temp.valid = true;
    m_obstacleQueue.push(temp);
}

void
ObstacleLabeler::obstacleHandler()
{
    double id = m_numGen.gaussian01();
    m_obstacleQueue.size();

    if(!m_obstacleQueue.empty() && m_obstacleQueue.front().valid) {
        obstacle_labeler::ReportObstacle msg;

        msg.id = id;
        msg.position.x = m_obstacleQueue.front().x;
        msg.position.y = m_obstacleQueue.front().y;
        msg.position.z = m_obstacleQueue.front().z;
        msg.valid = m_obstacleQueue.front().valid;
        m_obstacleQueue.pop();

        m_pub.publish(msg);
    }
}