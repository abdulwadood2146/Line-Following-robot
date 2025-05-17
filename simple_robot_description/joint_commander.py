import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from scipy.ndimage import center_of_mass
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Movement parameters
TURN_LEFT_SPD = 0.10
TURN_RIGHT_SPD = 0.10
STRAIGHT_SPD = 0.25
ANGULAR_VEL = 0.8
ROTATE_IN_PLACE_VEL = 1.2
BACKWARD_SPD = -0.1

# Thresholds
LEFT_TURN_THRESHOLD = 350
RIGHT_TURN_THRESHOLD = 450
HARD_LEFT_THRESHOLD = 200
HARD_RIGHT_THRESHOLD = 600

OFFSET_Y = 500
TIMER_PERIOD = 0.1

# Globals
prev = (400, 400)
img_bgr = np.zeros((800, 800, 3), dtype=np.uint8)
bridge = None
pub = None
at_junction = False
checking_sides = False
check_direction = 'left'
check_frames = 0
side_detected = None
backward_frames = 0
backward_done = False


def image_callback(msg):
    global img_bgr
    img_bgr = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


def detect_path_in_view(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    cropped = mask[-300:]
    return np.sum(cropped) > 5000


def callback():
    global prev, img_bgr, at_junction, checking_sides, check_direction
    global check_frames, side_detected, backward_frames, backward_done

    move = Twist()

    try:
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        cropped = mask[-300:]
        coords_bin = center_of_mass(cropped)
        y = coords_bin[0] + OFFSET_Y
        x = coords_bin[1]

        if checking_sides:
            move.linear.x = 0.0
            if check_direction == 'left':
                move.angular.z = ROTATE_IN_PLACE_VEL
            else:
                move.angular.z = -ROTATE_IN_PLACE_VEL

            check_frames += 1
            if check_frames >= 5:
                has_path = detect_path_in_view(img_bgr)
                if has_path:
                    side_detected = check_direction
                    print(f"Detected path to the {side_detected}")

                if check_direction == 'left':
                    check_direction = 'right'
                    check_frames = 0
                else:
                    checking_sides = False
                    if side_detected == 'left':
                        move.angular.z = ROTATE_IN_PLACE_VEL
                    elif side_detected == 'right':
                        move.angular.z = -ROTATE_IN_PLACE_VEL
                    else:
                        backward_frames = 0
                        backward_done = False
            pub.publish(move)
            return

        if not backward_done and np.isnan(x) or np.isnan(y):
            if backward_frames < 10:
                print("Backing up to see better...")
                move.linear.x = BACKWARD_SPD
                move.angular.z = 0.0
                backward_frames += 1
                pub.publish(move)
                return
            else:
                backward_done = True
                checking_sides = True
                check_direction = 'left'
                check_frames = 0
                side_detected = None
                move.linear.x = 0.0
                move.angular.z = 0.0

        else:
            prev = (x, y)
            print(f"Line center: ({x:.2f}, {y:.2f})")

            if x < HARD_LEFT_THRESHOLD:
                move.linear.x = 0.0
                move.angular.z = ROTATE_IN_PLACE_VEL
            elif x > HARD_RIGHT_THRESHOLD:
                move.linear.x = 0.0
                move.angular.z = -ROTATE_IN_PLACE_VEL
            elif x < LEFT_TURN_THRESHOLD:
                move.linear.x = TURN_LEFT_SPD
                move.angular.z = ANGULAR_VEL
            elif LEFT_TURN_THRESHOLD <= x <= RIGHT_TURN_THRESHOLD:
                move.linear.x = STRAIGHT_SPD
                move.angular.z = 0.0
            else:
                move.linear.x = TURN_RIGHT_SPD
                move.angular.z = -ANGULAR_VEL

        pub.publish(move)

    except Exception as e:
        print(f"Callback error: {e}")


def main(args=None):
    rclpy.init(args=args)

    global bridge, pub
    bridge = CvBridge()
    node = rclpy.create_node('move_robot')

    node.create_subscription(
        Image,
        '/camera1/image_raw',
        image_callback,
        rclpy.qos.qos_profile_sensor_data
    )

    pub = node.create_publisher(
        Twist,
        '/cmd_vel',
        rclpy.qos.qos_profile_system_default
    )

    node.create_timer(TIMER_PERIOD, callback)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
