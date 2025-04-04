#!/usr/bin/env python
import rospy
import tf2_ros
import numpy as np
import tf.transformations as tf_trans
from geometry_msgs.msg import Point, Quaternion, Pose, TransformStamped
from leap_motion_controller.msg import Hand
import numpy as np

DEFAULT_BASE_LINK = 'leap_base_link'

class HandTfBroadcaster:
    def __init__(self):
        rospy.init_node('hand_tf_broadcaster')

        self.left_hand_sub = rospy.Subscriber('/leapmotion/hands/left', Hand, self.left_hand_callback, queue_size=1)
        self.right_hand_sub = rospy.Subscriber('/leapmotion/hands/right', Hand, self.right_hand_callback, queue_size=1)

        self.base_link = rospy.get_param('base_link', DEFAULT_BASE_LINK)
        self.left_link = "leap_left_hand"
        self.right_link = "leap_right_hand"

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        self.rate = rospy.Rate(50)  # Hz
        rospy.spin()

    def left_hand_callback(self, hand_msg):
        self.broadcast_hand_tf(hand_msg, self.left_link)


    def right_hand_callback(self, hand_msg):
        self.broadcast_hand_tf(hand_msg, self.right_link)


    def broadcast_hand_tf(self, hand_msg, hand_link):
        t = TransformStamped()
        t.header.stamp = hand_msg.header.stamp
        t.header.frame_id = self.base_link
        t.child_frame_id = hand_link
        t.transform.translation = hand_msg.palm_center.position
        t.transform.rotation = hand_msg.palm_center.orientation
        self.tfBroadcaster.sendTransform(t)
        for finger_id, finger in enumerate(hand_msg.finger_list):
            finger_id = "leap_right_f" + str(finger_id) if hand_link == self.right_link else "leap_left_f" + str(finger_id)
            # Process each finger and publish the corresponding TF per articulation
            self.broadcast_finger_articulations(finger, finger_id, hand_msg.header, hand_link, hand_msg.palm_center)
        self.rate.sleep()



    def compute_quaternion(self, start, end):
        """Compute quaternion to align the bone from start to end, enforcing Z ⟂ X."""
        direction = np.array([end.x - start.x, end.y - start.y, end.z - start.z])
        norm = np.linalg.norm(direction)

        if norm < 1e-6:
            return Quaternion(0, 0, 0, 1)  # Identity quaternion (no rotation)

        X = direction / norm  # Normalized direction

        # Compute Z as a perpendicular vector to X
        Z_ref = np.array([0, 1, 0])
        Z = np.cross(X, Z_ref)
        Z = Z / np.linalg.norm(Z)  # Normalize

        # Compute Y to complete the right-handed frame
        Y = np.cross(Z, X)

        # Construct full 4×4 transformation matrix
        R = np.eye(4)  # Start with identity
        R[:3, 0] = X  # First column → X-axis
        R[:3, 1] = Y  # Second column → Y-axis
        R[:3, 2] = Z  # Third column → Z-axis

        # Convert to quaternion
        q = tf_trans.quaternion_from_matrix(R)

        return Quaternion(q[0], q[1], q[2], q[3])


    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

        return np.array([x, y, z, w])


    def broadcast_finger_articulations(self, finger, finger_id, header, parent_frame_id, parent_origin):
        frame_id = parent_frame_id
        origin_pos = parent_origin.position
        origin_rot = parent_origin.orientation
        # Iterate through the bones of the finger
        for index_bone, bone in enumerate(finger.bone_list):
            bone_start =  bone.bone_start
            bone_end = bone.bone_end

            # Create a tf frame for each articulation point
            t = TransformStamped()
            t.header.stamp = header.stamp
            t.header.frame_id = frame_id 
            t.child_frame_id = f"{finger_id}_j{index_bone}"
            # Get relative position of the bone start given the origin
            R = tf_trans.quaternion_matrix([origin_rot.x, origin_rot.y, origin_rot.z, origin_rot.w])[:3, :3]
            relative_bone_start = np.dot(R.T, np.array([bone_start.x - origin_pos.x, bone_start.y - origin_pos.y, bone_start.z - origin_pos.z]))
            relative_bone_end = np.dot(R.T, np.array([bone_end.x - origin_pos.x, bone_end.y - origin_pos.y, bone_end.z - origin_pos.z]))
            relative_bone_start = Point(relative_bone_start[0], relative_bone_start[1], relative_bone_start[2])
            relative_bone_end = Point(relative_bone_end[0], relative_bone_end[1], relative_bone_end[2])
            articulation_rot = self.compute_quaternion(relative_bone_start, relative_bone_end)
            t.transform.translation = relative_bone_start
            t.transform.rotation = articulation_rot
            self.tfBroadcaster.sendTransform(t)


if __name__ == "__main__":
    broadcaster = HandTfBroadcaster()