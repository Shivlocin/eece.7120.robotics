#!/usr/bin/env python
import csv
import rospy
from geometry_msgs.msg import Vector3


if __name__ == "__main__":
    rospy.init_node('pub_and_sub_example', anonymous=True)
    pub = rospy.Publisher("wheel_tick", Vector3)
    rate = rospy.Rate(10) # 10hz
    csv_file_name = "wheel_ticks.csv"
    if rospy.has_param("/odom_csv_file_name"):
        csv_file_name = rospy.get_param("/odom_csv_file_name")
    with open(csv_file_name, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        i = 0
        for row in csv_reader:
            ticks_left = float(row[0])
            ticks_right = float(row[1])
            ticks = Vector3()
            ticks.x = ticks_left
            ticks.y = ticks_right
            ticks.z = 0
            rospy.loginfo("Wheel Ticks: left=%f, right=%f",ticks_left,ticks_right)
            pub.publish(ticks)
            rate.sleep()
    
