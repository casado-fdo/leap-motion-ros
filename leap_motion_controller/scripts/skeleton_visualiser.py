#!/usr/bin/env python

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from leap_motion_controller.msg import Hand


class SkeletonVisualizer:
    def __init__(self):
        rospy.init_node('hand_visualizer')
        self.left_hand_pub = rospy.Publisher('/leapmotion/hands/left/marker', MarkerArray, queue_size=10)
        self.right_hand_pub = rospy.Publisher('/leapmotion/hands/right/marker', MarkerArray, queue_size=10)
        self.left_hand_sub = rospy.Subscriber('/leapmotion/hands/left', Hand, self.left_hand_callback)
        self.right_hand_sub = rospy.Subscriber('/leapmotion/hands/right', Hand, self.right_hand_callback)

    def left_hand_callback(self, hand_msg):
        self.marker_publisher(hand_msg, self.left_hand_pub, color=(0.2, 0.6, 0.9, 1.0))

    def right_hand_callback(self, hand_msg):
        self.marker_publisher(hand_msg, self.right_hand_pub, color=(0.9, 0.6, 0.2, 1.0))

    def marker_publisher(self, hand_msg, pub, color=(1.0, 1.0, 1.0, 1.0)):
        marker_array = MarkerArray()

        # Define a counter for marker IDs
        marker_id = 0

        # Create line markers (for bones)
        bone_marker = Marker()
        bone_marker.header.frame_id = "leap_base_link"
        bone_marker.header.stamp = rospy.Time.now()
        bone_marker.ns = "hand_skeleton"
        bone_marker.id = marker_id
        bone_marker.type = Marker.LINE_LIST
        bone_marker.action = Marker.ADD
        bone_marker.scale.x = 0.004  # Line thickness
        bone_marker.color.r = 1.0
        bone_marker.color.g = 1.0
        bone_marker.color.b = 1.0
        bone_marker.color.a = 1.0  # Fully visible

        # Create point markers (for joints)
        point_marker = Marker()
        point_marker.header.frame_id = "leap_base_link"
        point_marker.header.stamp = rospy.Time.now()
        point_marker.ns = "hand_joints"
        point_marker.id = marker_id + 1
        point_marker.type = Marker.SPHERE_LIST
        point_marker.action = Marker.ADD
        point_marker.scale.x = 0.015  # Sphere size
        point_marker.scale.y = 0.015
        point_marker.scale.z = 0.015
        point_marker.color.r = color[0]
        point_marker.color.g = color[1]
        point_marker.color.b = color[2]
        point_marker.color.a = color[3]

        # Add bones and joints
        for finger in hand_msg.finger_list:
            for bone in finger.bone_list:
                start = Point(bone.bone_start.x, bone.bone_start.y, bone.bone_start.z)
                end = Point(bone.bone_end.x, bone.bone_end.y, bone.bone_end.z)
                
                # Add line for the bone
                bone_marker.points.append(start)
                bone_marker.points.append(end)

                # Add points for the joints
                point_marker.points.append(start)
                point_marker.points.append(end)

        # Add markers to MarkerArray
        marker_array.markers.append(bone_marker)
        marker_array.markers.append(point_marker)

        # Publish the MarkerArray
        pub.publish(marker_array)

if __name__ == '__main__':
    SkeletonVisualizer()
    rospy.spin()
