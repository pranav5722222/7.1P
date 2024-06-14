import rospy
from duckietown_msgs.msg import Twist2DStamped, AprilTagDetectionArray

class FollowTarget:
    def __init__(self):
        # Set up ROS node
        rospy.init_node('follow_target_node', anonymous=True)

        # Register shutdown handler to clean up before exiting
        rospy.on_shutdown(self.shutdown_handler)

        # Initialize publisher for velocity commands
        self.velocity_publisher = rospy.Publisher('/duckie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        
        # Subscribe to AprilTag detection messages
        rospy.Subscriber('/duckie/apriltag_detection_node/detections', AprilTagDetectionArray, self.detection_callback, queue_size=1)
        
        # Spin to process callbacks
        rospy.spin()

    # Handle incoming AprilTag detection messages
    def detection_callback(self, msg):
        self.navigate_to_tag(msg.detections)

    # Clean shutdown function to stop the robot safely
    def shutdown_handler(self):
        rospy.loginfo("Shutdown initiated. Stopping the robot...")
        self.stop_robot()

    # Publish zero velocities to stop the robot
    def stop_robot(self):
        stop_msg = Twist2DStamped()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.v = 0.0
        stop_msg.omega = 0.0
        self.velocity_publisher.publish(stop_msg)

    # Navigate the robot towards the detected AprilTag
    def navigate_to_tag(self, detections):
        if not detections:
            self.stop_robot()
            return

        self.stop_robot()  # Ensure robot stops before new command
        tag = detections[0]
        x = tag.transform.translation.x
        y = tag.transform.translation.y
        z = tag.transform.translation.z

        rospy.loginfo("Detected tag at x: %f, y: %f, z: %f", x, y, z)
        rospy.sleep(1)

        if z > 0.15:
            self.move_forward()
        elif z < 0.20:
            self.move_backward()

        if x > 0.05:
            self.turn_left()
        elif x < -0.05:
            self.turn_right()

    # Move the robot forward
    def move_forward(self):
        move_msg = Twist2DStamped()
        move_msg.header.stamp = rospy.Time.now()
        move_msg.v = 0.2
        move_msg.omega = 0.0
        self.velocity_publisher.publish(move_msg)
        rospy.sleep(0.2)
        self.stop_robot()

    # Move the robot backward
    def move_backward(self):
        move_msg = Twist2DStamped()
        move_msg.header.stamp = rospy.Time.now()
        move_msg.v = -0.4
        move_msg.omega = 0.0
        self.velocity_publisher.publish(move_msg)
        rospy.sleep(0.2)
        self.stop_robot()

    # Turn the robot to the left
    def turn_left(self):
        turn_msg = Twist2DStamped()
        turn_msg.header.stamp = rospy.Time.now()
        turn_msg.v = 0.0
        turn_msg.omega = -0.4
        self.velocity_publisher.publish(turn_msg)
        rospy.sleep(0.4)
        self.stop_robot()

    # Turn the robot to the right
    def turn_right(self):
        turn_msg = Twist2DStamped()
        turn_msg.header.stamp = rospy.Time.now()
        turn_msg.v = 0.0
        turn_msg.omega = 0.4
        self.velocity_publisher.publish(turn_msg)
        rospy.sleep(0.4)
        self.stop_robot()

if __name__ == '__main__':
    try:
        follower = FollowTarget()
    except rospy.ROSInterruptException:
        pass
