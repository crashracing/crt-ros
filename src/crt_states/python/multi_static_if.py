# based on: https://github.com/jhu-lcsr/lcsr_tf_tools/blob/master/scripts/set_multi_static.py

import rospy
import tf

import PyKDL

import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import lcsr_tf_tools.msg as lcsr_tf_tools
import tf.msg

from multi_static_transform_publisher import MultiPublisher

def send(node_name, xyz, quat, frame_id, child_frame_id, period_ms):
    # Construct transform message
    tform = geometry_msgs.TransformStamped()
    tform.header.frame_id = frame_id
    tform.header.stamp = rospy.Time(0)

    tform.child_frame_id = child_frame_id

    tform.transform.translation.x = xyz[0]
    tform.transform.translation.y = xyz[1]
    tform.transform.translation.z = xyz[2]

    tform.transform.rotation.x = quat[0]
    tform.transform.rotation.y = quat[1]
    tform.transform.rotation.z = quat[2]
    tform.transform.rotation.w = quat[3]

    static_tform = lcsr_tf_tools.StaticTransform()
    static_tform.header.stamp = rospy.Time.now()
    static_tform.transform = tform
    static_tform.publish_period = rospy.Duration(1E-3*period_ms)

    # Publish the transform
    rospy.loginfo("Registering static transform %s --> %s\n%s" %(tform.header.frame_id, tform.child_frame_id, str(tform)))
    set_pub = None

    # Get a tf_listener to make sure the frame gets published
    listener = tf.TransformListener(True, rospy.Duration(10.0))

    # Wait for the frame to be published
    while listener and not rospy.is_shutdown():
        static_tform.header.stamp = rospy.Time.now()
        set_pub = rospy.Publisher(node_name+'/set_frame', lcsr_tf_tools.StaticTransform, latch=True)
        set_pub.publish(static_tform)
        try:
            now = rospy.Time(0)
            listener.waitForTransform(tform.header.frame_id, tform.child_frame_id, now, rospy.Duration(1.0))
            pose = listener.lookupTransform(tform.header.frame_id, tform.child_frame_id, now)
            rospy.loginfo("Registered static transform %s --> %s\n%s: %s" %(tform.header.frame_id, tform.child_frame_id, str(tform), str(pose)))
            listener = None
        except tf.Exception as ex:
            rospy.logwarn("Multi static transform %s --> %s not being published yet: %s" % (tform.header.frame_id, tform.child_frame_id, str(ex)))

