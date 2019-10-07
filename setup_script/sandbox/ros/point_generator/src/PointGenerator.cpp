//
// Created by student on 10/2/19.
//

#include <point_generator/PointGenerator.hpp>
#include <geometry_msgs/Point.h>

PointGenerator::PointGenerator(const ros::NodeHandlePtr& nh, int rate)
    :
    m_nh(nh),
    m_pubrate(rate)
{
    m_pub = nh->advertise<geometry_msgs::Point>("obstacles_detected", 100);
}

void
PointGenerator::start()
{
    m_running = true;
    while(m_running && ros::ok()) {

        /*
         * Get a set of random numbers. Returns an array of doubles with the
         * following format.
         *  return[1] = x
         *  return[2] = y
         *  return[3] = z
         *  return[4] = w
         */
        m_numGen.quaternion(m_quat);

        publishPoint(m_quat);
        m_pubrate.sleep();
    }
}

bool
PointGenerator::publishPoint(const double value[4])
{
    geometry_msgs::Point message;

    message.x = value[1];
    message.y = value[2];
    message.z = value[3];

    m_pub.publish(message);
}