#!/usr/bin/env python3
import leap
import rospy
from geometry_msgs.msg import Point, Quaternion, Pose, Vector3
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from leap_motion_controller.msg import Hand, Finger, Bone
import copy
import numpy as np
 

class LeapMotionController(leap.Listener):
    def __init__(self):
        super().__init__()

        rospy.init_node('lmc')

        self.left_link = "leap_left_hand"
        self.right_link = "leap_right_hand"
        self.base_link = rospy.get_param('base_link', 'leap_base_link')
        self.palm_pose_smooth_factor = rospy.get_param('palm_smooth_factor', 0.99)
        self.bones_smooth_factor = rospy.get_param('bones_smooth_factor', 0.98)

        self.pub_left = rospy.Publisher('/leapmotion/hands/left', Hand, queue_size=1)
        self.pub_right = rospy.Publisher('/leapmotion/hands/right', Hand, queue_size=1)
        self.pub_left_grab = rospy.Publisher('/leapmotion/hands/left/grab', Range, queue_size=1)
        self.pub_right_grab = rospy.Publisher('/leapmotion/hands/right/grab', Range, queue_size=1)
        self.pub_right_pinch = rospy.Publisher('/leapmotion/hands/right/pinch', Range, queue_size=1)
        self.pub_left_pinch = rospy.Publisher('/leapmotion/hands/left/pinch', Range, queue_size=1)

        self.last_right_hand_msg = Hand()
        self.last_left_hand_msg = Hand()

        rospy.loginfo('LeapMotionController Node is Up!')
        connection = leap.Connection()
        connection.add_listener(self)
        with connection.open():
            connection.set_tracking_mode(leap.TrackingMode.Desktop)
            rospy.spin()


    def leap_to_ros_coords(self, position=None, orientation=None):
        '''Converts Leap coordinates to ROS coordinates.
        Also, the leap coordinates are provided in millimeters, so they are converted to meters.'''
        new_position, new_orientation = None, None
        if position is not None:
            new_position = Point(- position[2] / 1000.0, - position[0] / 1000.0, position[1] / 1000.0)
        if orientation is not None:
            new_orientation = Quaternion(- orientation[2], - orientation[0], orientation[1], orientation[3])

        return new_position, new_orientation
    

    def on_connection_event(self, event):
        rospy.loginfo('Connected to a Leap Motion Controller.')


    def on_tracking_event(self, frame):
        # Get the most recent frame and report some basic information
        rospy.loginfo_throttle(2, "Tracking....\nFrame id: %d, hands: %d" % (
              frame.tracking_frame_id, len(frame.hands)))
        
        # Get the current time
        time = rospy.Time.now()

        # Get the Leap Motion data, structure it, and publish it
        if len(frame.hands) > 0:
            for hand in frame.hands:
                hand_msg = Hand()
                hand_msg.header = Header()
                hand_msg.header.frame_id = self.base_link
                hand_msg.header.stamp = time
                hand_msg.lmc_hand_id = hand.id

                # Get the hand's normal vector and direction
                orientation = hand.palm.orientation
                pos = hand.palm.position

                # Convert from Leap coordinates (right-handed Cartesian coordinate system, Y-axis up) to ROS coordinates (left-handed Cartesian coordinate system, Z-axis up
                pos, orientation = self.leap_to_ros_coords(pos, orientation)

                # Create a Pose message for the hand's palm center
                palm_pose = Pose()
                palm_pose.position = pos
                palm_pose.orientation = orientation
                
                # Provide a filtered palm pose applying an exponential smoothing filter
                last_pose = self.last_left_hand_msg.palm_center_filtered if str(hand.type) == "HandType.Left" else self.last_right_hand_msg.palm_center_filtered
                palm_pose_filtered = self.palm_exponential_smoothing(palm_pose, last_pose, self.palm_pose_smooth_factor)
                
                # Add the palm pose, normal, and direction to the hand message
                hand_msg.palm_center = palm_pose
                hand_msg.palm_center_filtered = palm_pose_filtered
                hand_msg.normal= hand.palm.normal
                hand_msg.direction = hand.palm.direction
                hand_msg.grab_strength = hand.grab_strength
                hand_msg.pinch_strength = hand.pinch_strength

                # Get the hand's grab strength
                grab = self.get_grab_range_msg(hand, time)

                # Get the hand's pinch strength
                pinch = self.get_pinch_range_msg(hand, time)

                # Add the fingers to the hand message
                hand_msg.finger_list = self.get_finger_list(hand, hand_msg.header)

                # Apply an exponential smoothing filter to each finger's bones
                last_fingerlist = self.last_left_hand_msg.finger_list_filtered if str(hand.type) == "HandType.Left" else self.last_right_hand_msg.finger_list_filtered
                hand_msg.finger_list_filtered = self.finger_bones_smoothing(hand_msg.finger_list, last_fingerlist, self.bones_smooth_factor)

                # Publish hand pose and grab strength
                if str(hand.type) == "HandType.Left":
                    self.pub_left.publish(hand_msg)
                    self.pub_left_grab.publish(grab)
                    self.pub_left_pinch.publish(pinch)
                    self.last_left_hand_msg = self.hand_msg_deepcopy(hand_msg)

                else:
                    self.pub_right.publish(hand_msg)
                    self.pub_right_grab.publish(grab)
                    self.pub_right_pinch.publish(pinch)
                    self.last_right_hand_msg = self.hand_msg_deepcopy(hand_msg)


    def hand_msg_deepcopy(self, hand_msg):
        new_hand_msg = Hand()
        new_hand_msg.header = copy.deepcopy(hand_msg.header)
        new_hand_msg.lmc_hand_id = hand_msg.lmc_hand_id
        new_hand_msg.palm_center = copy.deepcopy(hand_msg.palm_center)
        new_hand_msg.palm_center_filtered = copy.deepcopy(hand_msg.palm_center_filtered)
        new_hand_msg.normal = self.vector3_msg_deepcopy(hand_msg.normal)
        new_hand_msg.direction = self.vector3_msg_deepcopy(hand_msg.direction)
        new_hand_msg.grab_strength = hand_msg.grab_strength
        new_hand_msg.pinch_strength = hand_msg.pinch_strength
        new_hand_msg.finger_list = copy.deepcopy(hand_msg.finger_list)
        new_hand_msg.finger_list_filtered = copy.deepcopy(hand_msg.finger_list_filtered)
        return new_hand_msg
    

    def vector3_msg_deepcopy(self, vector3_msg):
        new_vector3_msg = Vector3()
        new_vector3_msg.x = vector3_msg.x
        new_vector3_msg.y = vector3_msg.y
        new_vector3_msg.z = vector3_msg.z
        return new_vector3_msg


    def get_joint_position(self, bone):
        if bone:
            pos, _ = self.leap_to_ros_coords(position=bone)
            return pos
        else:
            return None
        

    def get_grab_range_msg(self, hand, time):
        grab = Range()
        grab.header = Header()
        grab.header.frame_id = self.left_link if str(hand.type) == "HandType.Left" else self.right_link
        grab.header.stamp = time
        grab.field_of_view = 1
        grab.min_range = 0
        grab.max_range = 1
        grab.range = hand.grab_strength
        return grab
    

    def get_pinch_range_msg(self, hand, time):
        pinch = Range()
        pinch.header = Header()
        pinch.header.frame_id = self.left_link if str(hand.type) == "HandType.Left" else self.right_link
        pinch.header.stamp = time
        pinch.field_of_view = 1
        pinch.min_range = 0
        pinch.max_range = 1
        pinch.range = hand.pinch_strength
        return pinch
    

    def get_finger_list(self, hand, header):
        finger_list = []
        # Iterate through the fingers
        for index_digit in range(0, 5):
            finger = hand.digits[index_digit]
            
            finger_msg = Finger()
            finger_msg.header = header
            finger_msg.type = index_digit
            finger_msg.is_extended = finger.is_extended

            # Add the bones to the finger message
            finger_msg.bone_list = self.get_bone_list(finger, header)

            # Add the finger to the finger list
            finger_list.append(finger_msg)
        
        return finger_list
    

    def get_bone_list(self, finger, header):
        bone_list = []
        # Iterate through the bones of the finger
        for index_bone in range(0, 4):
            bone = finger.bones[index_bone]

            bone_msg = Bone()
            bone_msg.header = header
            bone_msg.type = index_bone
            bone_msg.bone_start =  self.get_joint_position(bone.prev_joint)
            bone_msg.bone_end = self.get_joint_position(bone.next_joint)

            bone_list.append(bone_msg)

        return bone_list
    

    def quaternion_slerp(self, q0, q1, t):
        """Spherical linear interpolation between two quaternions."""
        q0 = np.array([q0.x, q0.y, q0.z, q0.w])
        q1 = np.array([q1.x, q1.y, q1.z, q1.w])
        dot = np.dot(q0, q1)
        if dot < 0.0:
            q1 = -q1
            dot = -dot
        DOT_THRESHOLD = 0.9995
        if dot > DOT_THRESHOLD:
            result = q0 + t * (q1 - q0)
            result /= np.linalg.norm(result)
            return result
        theta_0 = np.arccos(dot)
        sin_theta_0 = np.sin(theta_0)
        theta = theta_0 * t
        sin_theta = np.sin(theta)
        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        result = (s0 * q0) + (s1 * q1)
        result /= np.linalg.norm(result)
        return result


    def palm_exponential_smoothing(self, current_pose, last_pose, smooth_factor):
        """Applies an exponential smoothing filter to the current pose."""
        filtered_pose = Pose()
        filtered_pose.position.x = smooth_factor * last_pose.position.x + (1 - smooth_factor) * current_pose.position.x
        filtered_pose.position.y = smooth_factor * last_pose.position.y + (1 - smooth_factor) * current_pose.position.y
        filtered_pose.position.z = smooth_factor * last_pose.position.z + (1 - smooth_factor) * current_pose.position.z
        # Spherically interpolate (slerp) between last and current orientation to avoid invalid quaternions
        q_filtered = self.quaternion_slerp(last_pose.orientation, current_pose.orientation, 1 - smooth_factor)
        filtered_pose.orientation = Quaternion(q_filtered[0], q_filtered[1], q_filtered[2], q_filtered[3])

        return filtered_pose
    

    def finger_bones_smoothing (self, fingerlist, last_fingerlist, smooth_factor):
        """Applies an exponential smoothing filter to each finger's bones."""
        if last_fingerlist is None or len(last_fingerlist) != len(fingerlist):
            return fingerlist
        filtered_fingerlist = []
        for finger, last_finger in zip(fingerlist, last_fingerlist):
            filtered_finger = Finger()
            filtered_finger.header = finger.header
            filtered_finger.type = finger.type
            filtered_finger.is_extended = finger.is_extended
            filtered_bone_list = []
            for bone, last_bone in zip(finger.bone_list, last_finger.bone_list):
                filtered_bone = Bone()
                filtered_bone.header = bone.header
                filtered_bone.type = bone.type
                filtered_bone.bone_start = Point(
                    smooth_factor * last_bone.bone_start.x + (1 - smooth_factor) * bone.bone_start.x,
                    smooth_factor * last_bone.bone_start.y + (1 - smooth_factor) * bone.bone_start.y,
                    smooth_factor * last_bone.bone_start.z + (1 - smooth_factor) * bone.bone_start.z,
                )
                filtered_bone.bone_end = Point(
                    smooth_factor * last_bone.bone_end.x + (1 - smooth_factor) * bone.bone_end.x,
                    smooth_factor * last_bone.bone_end.y + (1 - smooth_factor) * bone.bone_end.y,
                    smooth_factor * last_bone.bone_end.z + (1 - smooth_factor) * bone.bone_end.z,
                )
                filtered_bone_list.append(filtered_bone)
            filtered_finger.bone_list = filtered_bone_list
            filtered_fingerlist.append(filtered_finger)
        return filtered_fingerlist


if __name__ == '__main__':
    LeapMotionController()
