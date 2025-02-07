#!/usr/bin/env python
import leap
import rospy
import tf2_ros
from geometry_msgs.msg import Point, Quaternion, PoseStamped, TransformStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Header


DEFAULT_BASE_LINK = 'leap_base_link'

class LeapMotionController(leap.Listener):
    def __init__(self):
        super().__init__()
        rospy.init_node('lmc')
        self.base_link = rospy.get_param('~base_link', DEFAULT_BASE_LINK)
        self.left_link = "leap_left_hand"
        self.right_link = "leap_right_hand"
        self.pub_left = rospy.Publisher('/hands/left/pose', PoseStamped, queue_size=10)
        self.pub_right = rospy.Publisher('/hands/right/pose', PoseStamped, queue_size=10)
        self.pub_left_grab = rospy.Publisher('/hands/left/grab', Range, queue_size=10)
        self.pub_right_grab = rospy.Publisher('/hands/right/grab', Range, queue_size=10)

        # Define a TF link for each hand with base_link as parent
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        rospy.loginfo('LeapMotionController Node is Up!')
        connection = leap.Connection()
        connection.add_listener(self)
        with connection.open():
            connection.set_tracking_mode(leap.TrackingMode.Desktop)
            rospy.spin()

    def leap_to_ros_coords(self, pos, orientation):
        '''Converts Leap coordinates to ROS coordinates by flipping the y and z axes and changing the sign of the y axis.
        Also, the leap coordinates are provided in millimeters, so they are converted to meters.'''
        new_pos = Point(pos.x/1000, -pos.z/1000, pos.y/1000)
        new_orientation = Quaternion(orientation.x, - orientation.z, orientation.y, orientation.w)

        return new_pos, new_orientation
    

    def on_connection_event(self, event):
        rospy.loginfo('Connected to a Leap Motion Controller.')


    def on_tracking_event(self, frame):
        # Get the most recent frame and report some basic information
        rospy.loginfo('Tracking...')

        print("Frame id: %d, hands: %d" % (
              frame.tracking_frame_id, len(frame.hands)))

        if len(frame.hands) > 0: #recently changed in API
            for hand in frame.hands:
                # Get the hand's normal vector and direction
                normal = hand.palm.normal
                direction = hand.palm.direction
                orientation = hand.palm.orientation
                pos = hand.palm.position

                # Convert from Leap coordinates (right-handed Cartesian coordinate system, Y-axis up) to ROS coordinates (left-handed Cartesian coordinate system, Z-axis up
                pos, orientation = self.leap_to_ros_coords(pos, orientation)

                time = rospy.Time.now()

                # Broadcast the hand's pose as a TF frame
                t = TransformStamped()
                t.header.stamp = time
                t.header.frame_id = self.base_link
                t.child_frame_id = self.left_link if str(hand.type) == "HandType.Left" else self.right_link
                t.transform.translation.x = pos.x 
                t.transform.translation.y = pos.y 
                t.transform.translation.z = pos.z 
                t.transform.rotation.x = orientation.x
                t.transform.rotation.y = orientation.y
                t.transform.rotation.z = orientation.z
                t.transform.rotation.w = orientation.w
                self.tfBroadcaster.sendTransform(t)

                # Create a PoseStamped message for the hand's pose
                ros_pose = PoseStamped()
                ros_pose.header = Header()
                ros_pose.header.frame_id = self.base_link
                ros_pose.header.stamp = time
                ros_pose.pose.position.x = pos.x 
                ros_pose.pose.position.y = pos.y 
                ros_pose.pose.position.z = pos.z 
                ros_pose.pose.orientation.x = orientation.x
                ros_pose.pose.orientation.y = orientation.y
                ros_pose.pose.orientation.z = orientation.z
                ros_pose.pose.orientation.w = orientation.w

                # Get the hand's grab strength
                grab = Range()
                grab.header = Header()
                grab.header.frame_id = self.left_link if str(hand.type) == "HandType.Left" else self.right_link
                grab.header.stamp = time
                grab.field_of_view = 1
                grab.min_range = 0
                grab.max_range = 1
                grab.range = hand.grab_strength

                # Publish hand pose and grab strength
                if str(hand.type) == "HandType.Left":
                    self.pub_left.publish(ros_pose)
                    self.pub_left_grab.publish(grab)
                else:
                    self.pub_right.publish(ros_pose)
                    self.pub_right_grab.publish(grab)





if __name__ == '__main__':
    LeapMotionController()
