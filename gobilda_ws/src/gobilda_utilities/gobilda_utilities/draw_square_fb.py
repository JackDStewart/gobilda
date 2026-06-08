import rclpy
from rclpy.node import Node
import math


'''
    We're going to implement an open loop controller/FSM to have the turtlebot
    draw a square on the screen. Below I have commented parts of the code that
    you need to fill in to make the logic complete.
'''

# We have to use the geometry_msgs/msg/Twist to control robots
# Write in here what the correct import should be
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

class DrawSquare(Node):

    def __init__(self):
        # Init the node with a name (this is the name that appears when running)
        # 'ros2 node list'
        super().__init__('draw_square_gobilda')
        
        self.publisher_ = self.create_publisher(
            TwistStamped,
            '/gobilda/cmd_vel',
            10
        )
        
        self.subscriber_ = self.create_subscription(
            Odometry,
            '/gobilda_base_controller/odom',
            self.odom_callback,
            10
        )

        # Functions running at 20Hz
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.turning = False

        self.forward_msg = TwistStamped()
        # Robot's forward speed. No need to go too fast
        self.forward_msg.twist.linear.x = 0.2

        # What if I want the robot to turn 90 degrees?
        # Along which axis?
        # (Note that values are in rad/s)
        self.turn_msg = TwistStamped()
        self.turn_msg.twist.angular.z = 0.72     #changed this from 0.72 to 0.4

        # Utility Messages
        self.stop_msg = TwistStamped()
        self.pub_msg = TwistStamped()

        # Start the robot moving forward
        self.rotate = False
        self.init = False

        # Start position for the robot
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_heading = 0.0

        # Discritize the robot's heading
        # Track the next direction for the robot
        self.direction = 0
        self.headings = [0, math.pi/2, -math.pi, -math.pi/2]

    def angle_diff(self, a, b):
        # Shortest signed difference a - b, with wrap.
        return ((a-b) + math.pi) % (2.0 * math.pi) - math.pi

    # Think of this function as an interupt. Keep it minimal to not loose points!
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        r = R.from_quat([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ])
        
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        self.robot_heading = yaw
        self.init = True

    # Callback for the events
    # Implement here an FSM to have the robot perform the desired behavior
    def timer_callback(self):
        # Wait for at least one sensor reading from the robot
        if (not self.init):
            self.get_logger().info('Stuck waiting...')
            return

        # If robot is turining
        if (self.rotate):
            # Turn 90 degrees
            if (abs(self.angle_diff(self.robot_heading, self.start_heading)) <= (math.pi/2)):     # changed this to absoulte val and pi/2 instead of pi/4
                # Save the correct command
                self.pub_msg = self.turn_msg

            else:
                # Stop the robot for a bit
                self.pub_msg = self.stop_msg
                self.rotate = False
                
                # Save the target x,y that the robot should travel
                self.start_x = self.robot_x
                self.start_y = self.robot_y

                # Print state change
                self.get_logger().info('Changing State: Rotate -> Forward')
        
        else:
            # Measure the distance that the robot has currently traveled!
            if (math.hypot(self.robot_x - self.start_x, self.robot_y - self.start_y) <= 1.0):
                self.pub_msg = self.forward_msg

            # If we traveled 1m forward then have the robot stop and begin turning mode
            else:
                # Change the state of the robot
                self.rotate = True
                self.pub_msg = self.stop_msg

                # Save the target heading for the robot
                self.start_heading = self.robot_heading

                # Increase the direction of the robot
                self.direction = (self.direction + 1) % 4
                self.get_logger().info('Changing State: Forward -> Rotate')
        
        # self.get_logger().info(f'Current robot position (x,y): {self.robot_x}, {self.robot_y}')
        self.get_logger().info(f'Current robot heading (rads): {self.robot_heading}')
        self.publisher_.publish(self.pub_msg)

def main(args=None):
    rclpy.init(args=args)

    draw_square = DrawSquare()
    rclpy.spin(draw_square)

    draw_square.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
