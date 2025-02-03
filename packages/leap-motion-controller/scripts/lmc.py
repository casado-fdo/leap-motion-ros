#!/usr/bin/env python
import leap
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from lmc_msgs.msg import Hand, Hands
from std_msgs.msg import Header


class LeapMotionController(leap.Listener):
    def __init__(self):
        super().__init__()
        rospy.init_node('lmc')
        #self.pub = rospy.Publisher('~hands', Hands, queue_size=10)
        self.pub = rospy.Publisher('/hands', PoseStamped, queue_size=10)
        rospy.loginfo('LeapMotionController Node is Up!')
        connection = leap.Connection()
        connection.add_listener(self)
        with connection.open():
            connection.set_tracking_mode(leap.TrackingMode.Desktop)
            #connection.enable_gesture(leap.GestureType.TYPE_SWIPE)
            rospy.spin()

    def on_connection_event(self, event):
        rospy.loginfo('Connected to a Leap Motion Controller.')

    def on_device_event(self, event):
        with event.device.open():
            rospy.loginfo(f'Serial: {event.device.get_info().serial}')

    #def on_tracking_event(self, event):
    #    hands_lst = [Hand(pose=Pose(position=Point(x=hand.palm.position.x, y=hand.palm.position.y, z=hand.palm.position.z), orientation=Quaternion(x=hand.palm.orientation.x, y=hand.palm.orientation.y, z=hand.palm.orientation.z, w=hand.palm.orientation.w)), pinch_distance=hand.pinch_distance, confidence=hand.confidence, visible_time=hand.visible_time) for hand in event.hands]
    #    hands_msg = Hands(header=Header(seq=event.tracking_frame_id, stamp=rospy.Time.now(), frame_id='lmc'), hands=hands_lst)
    #    self.pub.publish(hands_msg)

    def on_tracking_event(self, frame):
        # Get the most recent frame and report some basic information
        rospy.loginfo('Tracking...')

        print("Frame id: %d, hands: %d" % (
              frame.tracking_frame_id, len(frame.hands)))

        if len(frame.hands) > 0: #recently changed in API
            # Get the first hand
            #we are seeking one left and one right hands
            there_is_right_hand=False
            there_is_left_hand=False

            for hand in frame.hands:
                if str(hand.type) == "HandType.Left":
                    hand_type = "Left" 
                    there_is_left_hand=True
                    self.right_hand=hand
                else:
                    hand_type = "Right"
                    there_is_right_hand=True
                    self.left_hand=hand

            if not there_is_right_hand:
                self.right_hand=False

            if not there_is_left_hand:
                self.left_hand=False

            self.hand = frame.hands[0] #old way

            # Check if the hand has any fingers
            digits = self.hand.digits
            #if len(digits) > 0:
            #    for fingerName in self.fingerNames:
            #        #finger = fingers.finger_type(Leap.Finger.TYPE_THUMB)[0]
            #        #self.thumb.importFinger(finger)
            #        finger = fingers.finger_type(getattr(leap.Finger, 'TYPE_%s' % fingerName.upper()))[0]
            #        getattr(self, fingerName).importFinger(finger)

            # Get the hand's sphere radius and palm position
            # print "Hand sphere radius: %f mm, palm position: %s" % (self.hand.sphere_radius, hand.palm_position)

            # Get the hand's normal vector and direction
            normal = self.hand.palm.normal
            direction = self.hand.palm.direction
            orientation = self.hand.palm.orientation
            pos = self.hand.palm.position

            ros_pose = PoseStamped()
            ros_pose.header = Header()
            ros_pose.header.frame_id = 'map'
            ros_pose.header.stamp = rospy.Time.now()
            ros_pose.pose.position.x = pos.x
            ros_pose.pose.position.y = pos.y
            ros_pose.pose.position.z = pos.z
            ros_pose.pose.orientation.x = orientation.x
            ros_pose.pose.orientation.y = orientation.y
            ros_pose.pose.orientation.z = orientation.z
            ros_pose.pose.orientation.w = orientation.w

            self.pub.publish(ros_pose)


if __name__ == '__main__':
    LeapMotionController()
