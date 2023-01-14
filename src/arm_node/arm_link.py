#!/usr/bin/env python3

import tf
import tf2_ros
import geometry_msgs.msg
import rospy


REVOLUTIONS_PER_METER = 6000.0

def get_arm_transforms(arm_base_position_revolutions: float, arm_upper_position_revolutions: float):
    """
    Returns the carriage transform as a single element in a stamped transform list.
    """

    arm_base_transform = geometry_msgs.msg.TransformStamped()
    
    arm_base_transform.header.stamp = rospy.Time.now()
    arm_base_transform.header.frame_id = "base_link"

    arm_base_transform.child_frame_id = "carriage_link"

    arm_base_transform.transform.translation.x = float(0.5)
    arm_base_transform.transform.translation.y = float(0.0)
    arm_base_transform.transform.translation.z = float(arm_base_position_revolutions / REVOLUTIONS_PER_METER)

    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(0))
    arm_base_transform.transform.rotation.x = quat[0]
    arm_base_transform.transform.rotation.y = quat[1]
    arm_base_transform.transform.rotation.z = quat[2]
    arm_base_transform.transform.rotation.w = quat[3]



    arm_upper_transform = geometry_msgs.msg.TransformStamped()
    
    arm_upper_transform.header.stamp = rospy.Time.now()
    arm_upper_transform.header.frame_id = "base_link"

    arm_upper_transform.child_frame_id = "carriage_link"

    arm_upper_transform.transform.translation.x = float(0.5)
    arm_upper_transform.transform.translation.y = float(0.0)
    arm_upper_transform.transform.translation.z = float(arm_upper_position_revolutions / REVOLUTIONS_PER_METER)

    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(0))
    arm_upper_transform.transform.rotation.x = quat[0]
    arm_upper_transform.transform.rotation.y = quat[1]
    arm_upper_transform.transform.rotation.z = quat[2]
    arm_upper_transform.transform.rotation.w = quat[3]

    return [arm_base_transform, arm_upper_transform]