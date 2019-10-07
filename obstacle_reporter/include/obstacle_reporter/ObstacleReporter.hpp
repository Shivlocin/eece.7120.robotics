//
// Created by student on 10/7/19.
//

#ifndef ROS_OBSTACLEREPORTER_HPP
#define ROS_OBSTACLEREPORTER_HPP

#include <ros/ros.h>
#include <obstacle_labeler/ReportObstacle.h>

typedef obstacle_labeler::ReportObstacleConstPtr ReportObstacleHandle;

class ObstacleReporter
{
public:
    ObstacleReporter(int rate, ros::NodeHandlePtr nh);

    ~ObstacleReporter() {m_running = false;};

    void start();

protected:
    void newObstacleCallback(ReportObstacleHandle msg);

private:

    /**
    * Member variable holding the running flag
    */
    bool m_running = false;

    /**
    * Member variable holding ros::rate
    */
    ros::Rate m_cycleRate;

    /**
    * Member variable holding the nodehandle
    */
    ros::NodeHandlePtr m_nh;

    /**
     * Member varialbe holding the subscriber
     */

    ros::Subscriber m_sub;
};



#endif //ROS_OBSTACLEREPORTER_HPP
