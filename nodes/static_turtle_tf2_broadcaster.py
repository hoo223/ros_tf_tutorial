#!/usr/bin/env python
import rospy 

# to get commandline arguments
import sys

# because of transformations
import tf, tf2_py

import tf2_ros # for StaticTransformBroadcaster
import geometry_msgs.msg

if __name__ == '__main__':
    if len(sys.argv) < 8:
        rospy.logerr('Invalid number of parameters\nusuage: '
                     './static_turtle_tf2_broadcaster.py'
                     'child_frame_name x y z roll pitch yaw')
        sys.exit(0)
    else:
        if sys.argv[1] == 'world':
            rospy.logerr('Your static turtle name cannot be "world"')
            sys.exit(0)

        rospy.init_node('my_static_tf2_broadcaster')
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped() # create a message

        # fill the meassage
        static_transformStamped.header.stamp = rospy.Time.now() # timestamp
        static_transformStamped.header.frame_id = "world" # parent frame
        static_transformStamped.child_frame_id = sys.argv[1] # child frame

        static_transformStamped.transform.translation.x = float(sys.argv[2])
        static_transformStamped.transform.translation.y = float(sys.argv[3])
        static_transformStamped.transform.translation.z = float(sys.argv[4])

        quat = tf.transformations.quaternion_from_euler(
            float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7])
        )
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)
        rospy.spin()