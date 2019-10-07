//
// Created by student on 10/6/19.
//

#ifndef ROS_OBSTACLELABELER_HPP
#define ROS_OBSTACLELABELER_HPP

#include <ros/ros.h>
#include <thread>
#include <queue>
#include <mutex>
#include <obstacle_labeler/ReportObstacle.h>
#include <random_numbers/random_numbers.h>

struct Obstacle{
    double x, y, z, id;
    bool valid = false;
};

typedef std::queue<Obstacle> ObstacleQueue;

class ObstacleLabeler
{
public:
    ObstacleLabeler(const int rate, const ros::NodeHandlePtr& nh);

    ~ObstacleLabeler() {m_running = false;};

     void start();

protected:
    void gotObstacleCallback(const geometry_msgs::Point::ConstPtr& msg);

private:

    void obstacleHandler();

    /**
    * Member variables containing private and public nodehandle pointers
    */
    ros::NodeHandlePtr m_nh;

    /**
    * Member variable holding the subscriber and callback
    */
    ros::Subscriber m_sub;

    /**
    * Member variable holding the publisher
    */
    ros::Publisher m_pub;

    /**
    * Vector of reported points waiting to be published.
    */
    ObstacleQueue m_obstacleQueue;

    /**
    * Member variable holding the running flag
    */
    bool m_running = false;

    /**
    * Rate of spin
    */
    ros::Rate m_rate;

    /**
     * Member variable holding the Random number generator
     */
     random_numbers::RandomNumberGenerator m_numGen;
};



#endif //ROS_OBSTACLELABELER_HPP
