#!/usr/bin/env python  
# -*- coding: utf-8 -*-
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    # listener 생성
    tfBuffer = tf2_ros.Buffer() # 버퍼 생성
    listener = tf2_ros.TransformListener(tfBuffer)

    # turtle2 생성
    rospy.wait_for_service('spawn') 
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    turtle_name = rospy.get_param('turtle', 'turtle2') # rospy.get_param('foo', 'default_value') 
    spawner(4, 2, 0, turtle_name)

    # publisher
    turtle_vel = rospy.Publisher('%s/cmd_vel' % turtle_name, geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # turtle1의 transform 정보 얻기
        try:
            past = rospy.Time.now() - rospy.Duration(5.0)
            trans = tfBuffer.lookup_transform_full(
                target_frame=turtle_name, # Give the transform from this frame
                target_time=rospy.Time.now(), # at this time...
                source_frame='carrot1', # ... to this frame
                source_time=past, # at this time
                fixed_frame='world', # Specify the frame that does not change over time, in this case the "/world" frame
                timeout=rospy.Duration(1.0) # and the time-out
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        # velocity command 생성
        msg = geometry_msgs.msg.Twist()

        msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x) # p control
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2) # p control

        # command publish
        turtle_vel.publish(msg)

        rate.sleep()