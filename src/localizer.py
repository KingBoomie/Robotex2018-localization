#! /usr/bin/env python

from __future__ import print_function
import rospy

# tf message
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler

# Image processing
from cv_bridge import CvBridge
import cv2

# Messages
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from serial.msg import WheelSpeed
from localization.msg import BasketAngle

# Localization source
import localization_src.localizer

loc = localization_src.localizer.Localizer('/home/robot/catkin_ws/src/localization/src/calibration/calibration-rs-480-640.yml')

def camera_listener(data):
    # print("Frame arrived")
    # Get the CvBridge object and the Publisher object
    global bridge, relative_angle, pose

    # Convert the received image to BGR8 format
    img = bridge.imgmsg_to_cv2(data, "bgr8")

    out = cv2.transpose(img)
    out = cv2.flip(out,flipCode=1)

    # Estimate the localization based on the image
    loc.estimateLocation(out, True)

    # Get the map
    field_map = loc.getLatestMap()
    
    # If any of the angles are None then set a default value to it
    if field_map.relative_angle_to_blue == None:
        field_map.relative_angle_to_blue = 9999999

    if field_map.relative_angle_to_pink == None:
        field_map.relative_angle_to_pink = 9999999

    # Create a new message object
    message = BasketAngle()
    
    # Get the contents of the message
    message.blue = field_map.relative_angle_to_blue * 180 / 3.141
    message.pink = field_map.relative_angle_to_pink * 180 / 3.141

    # Publish the message
    relative_angle.publish(message)

    # ----------------------
    # Send global position
    if field_map.robot_position[0] is not None and field_map.absolute_robot_angle is not None:

        yaw = field_map.absolute_robot_angle

        pose_out = PoseWithCovarianceStamped()

        # Populate header
        pose_out.header.stamp = rospy.Time.now()
        pose_out.header.frame_id = "aruco_map"

        # Populate position
        (x,y) = field_map.robot_position
        pose_out.pose.pose.position.x = x
        pose_out.pose.pose.position.y = y

        # Populate orientation
        pose_out.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))

        # Publish it
        pose.publish(pose_out)


    return

def wheelspeed_listener(data):
    # Add the motor speeds
    loc.updateMapWithMotorData(data.wheel1, data.wheel2, data.wheel3, data.wheel4)
    return

def localization_start():
    # Initialize ROS node
    rospy.init_node('localization_node')

    # Make a CvBridge instance
    global bridge
    bridge = CvBridge()

    # Publisher object for the RelativeBasketAngle message
    global relative_angle
    relative_angle = rospy.Publisher('relativeangle', BasketAngle, queue_size=10)

    # Publisher for pose message
    global pose
    pose = rospy.Publisher('aruco_pose', PoseWithCovarianceStamped, queue_size=10)

    #print(rospy.get_param())

    # Subscribe to wheelspeed
    rospy.Subscriber('wheelspeed', WheelSpeed, wheelspeed_listener)
    
    # Subscribe to Camera
    rospy.Subscriber('camera/color/image_raw', Image, camera_listener)

    # Start the node
    rospy.spin()

if __name__ == "__main__":
    localization_start()
