//
// Created by student on 10/2/19.
//

#ifndef ROS_POINTGENERATOR_HPP
#define ROS_POINTGENERATOR_HPP

#include <ros/ros.h>
#include <random_numbers/random_numbers.h>



class PointGenerator
{
public:
    PointGenerator(const ros::NodeHandlePtr& nh, int rate);

    ~PointGenerator() { m_running = false; };

    void start();

protected:

private:

    bool publishPoint(const double value[4] );

    /**
     * Member variable nodehandles
     */
    ros::NodeHandlePtr m_nh;

    /**
    * Member variable referencing the publisher
    */
     ros::Publisher m_pub;

    /**
    * Control flag for the loop
    */
    bool m_running = false;

    /**
    * Member variable holding the publish rate control
    */
    ros::Rate m_pubrate;

    /**
     * Random number generator
     */
    random_numbers::RandomNumberGenerator m_numGen;

    /**
     * Returned quaternion
     */
     double m_quat[4]{0};
};


#endif //ROS_POINTGENERATOR_HPP
