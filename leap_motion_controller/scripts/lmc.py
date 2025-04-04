#!/usr/bin/env python
import leap
import rospy
from geometry_msgs.msg import Point, Quaternion, Pose
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from leap_motion_controller.msg import Hand, Finger, Bone

DEFAULT_BASE_LINK = 'leap_base_link'

class LeapMotionController(leap.Listener):
    def __init__(self):
        super().__init__()

        rospy.init_node('lmc')

        self.left_link = "leap_left_hand"
        self.right_link = "leap_right_hand"
        self.base_link = rospy.get_param('base_link', DEFAULT_BASE_LINK)

        self.pub_left = rospy.Publisher('/leapmotion/hands/left', Hand, queue_size=1)
        self.pub_right = rospy.Publisher('/leapmotion/hands/right', Hand, queue_size=1)
        self.pub_left_grab = rospy.Publisher('/leapmotion/hands/left/grab', Range, queue_size=1)
        self.pub_right_grab = rospy.Publisher('/leapmotion/hands/right/grab', Range, queue_size=1)
        self.pub_right_pinch = rospy.Publisher('/leapmotion/hands/right/pinch', Range, queue_size=1)
        self.pub_left_pinch = rospy.Publisher('/leapmotion/hands/left/pinch', Range, queue_size=1)

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
                
                # Add the palm pose, normal, and direction to the hand message
                hand_msg.palm_center = palm_pose
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

                # Publish hand pose and grab strength
                if str(hand.type) == "HandType.Left":
                    self.pub_left.publish(hand_msg)
                    self.pub_left_grab.publish(grab)
                    self.pub_left_pinch.publish(pinch)
                else:
                    self.pub_right.publish(hand_msg)
                    self.pub_right_grab.publish(grab)
                    self.pub_right_pinch.publish(pinch)


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


if __name__ == '__main__':
    LeapMotionController()
