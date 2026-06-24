#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist

class Detector(Node):

    def __init__(self):
        super().__init__('detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/world/empty/model/follower_vehicle/link/chassis/sensor/rgb_camera/image',
            self.image_callback,
            10
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.get_logger().info("Detector node started")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = (0,120,70)
        upper_red1 = (10,255,255)

        lower_red2 = (170,120,70)
        upper_red2 = (180,255,255)


        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = mask1 + mask2


        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )


        twist = Twist()


        if len(contours) > 0:

            largest = max(contours, key=cv2.contourArea)

            M = cv2.moments(largest)

            if M["m00"] > 0:

                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])

                cv2.circle(
                    frame,
                    (cx,cy),
                    10,
                    (0,255,0),
                    -1
                )


                height,width,_ = frame.shape

                center_x = width//2

                error = cx - center_x


                twist.linear.x = 0.0

                if abs(error) < 20:

                    twist.angular.z = 0.0

                else:

                    twist.angular.z = 0.005 * error

                    twist.angular.z = max(
                        min(twist.angular.z, 1.0),
                        -1.0
                    )

                self.cmd_pub.publish(twist)
                self.get_logger().info(f"cx={cx}, error={error}, rotation={twist.angular.z}")


        # ALWAYS display
        cv2.imshow("Detection", frame)
        cv2.waitKey(1)
        


def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()