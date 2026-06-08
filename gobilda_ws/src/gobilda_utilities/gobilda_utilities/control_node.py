from enum import Enum
import rclpy
from rclpy.node import Node
import math

# We have to use the geometry_msgs/msg/TwistStamped to control robots
# Write in here what the correct import should be
from geometry_msgs.msg import TwistStamped # type: ignore
from geometry_msgs.msg import Twist # type: ignore
from sensor_msgs.msg import LaserScan # type: ignore

   # FSM states
class State(Enum):
	Forward = 1
	Backward = 2
	Turn = 3
	Stop = 4
            
class ControlNode(Node):

    def __init__(self):
        # Init the node with a name (this is the name that appears when running)
        # 'ros2 node list'
        super().__init__('control_node')

        
        # Functions running at 10Hz
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


        # Remember that the 'create_publisher' function takes in three arguments
        # Message Type | Topic Name | Queue Length

        # cmd_vel publisher
        self.publisher_ = self.create_publisher(TwistStamped, '/gobilda/cmd_vel', 10)

		# lidar subscriber
        self.subscription_ = self.create_subscription(LaserScan,'/scan', self.lidar_callback, 10)
        self.subscription_  # prevent unused variable warning
        
        self.front_distance = float('inf') # initialize front distance
        self.back_distance = float('inf') # initialize back distance

        self.backup_timer = 0 # to track backward duration
        self.state = State.Forward # initial state

        # Forward message
        self.forward_msg = TwistStamped()
        self.forward_msg.twist = Twist()
        self.forward_msg.twist.linear.x = 2.0
        
		# Backward message
        self.backward_msg = TwistStamped()
        self.backward_msg.twist = Twist()
        self.backward_msg.twist.linear.x = -2.0

        # Turn msg
        self.turn_msg = TwistStamped()
        self.turn_msg.twist = Twist()
        self.turn_msg.twist.angular.z = math.pi / 5  


    def get_distance(self, msg: LaserScan, center_angle: float, fov: int) -> float:
        min_distance = float('inf')
        angle_range = math.radians(fov / 2)
        cur_angle = msg.angle_min
        center_angle = math.radians(center_angle)
        if center_angle > math.pi:
            center_angle -= 2 * math.pi
        elif center_angle <= -math.pi:
            center_angle += 2 * math.pi
        min_angle = center_angle - angle_range
        max_angle = center_angle + angle_range

        for dist in msg.ranges:
            if not math.isfinite(dist) or dist < msg.range_min or dist > msg.range_max:
                cur_angle += msg.angle_increment
                continue

            # handle wraparound in [-pi, pi]
            if (min_angle <= max_angle and min_angle <= cur_angle <= max_angle) or \
            (min_angle > max_angle and (cur_angle >= min_angle or cur_angle <= max_angle)):
                min_distance = min(min_distance, dist)

            cur_angle += msg.angle_increment

        return min_distance


    # Callback for the events
    
    def lidar_callback(self, msg):
        # Get the distance in front and back of the robot
        front_angle = 0
        self.front_distance = self.get_distance(msg, front_angle, 30) # get front distance
        back_angle = 180
        self.back_distance = self.get_distance(msg, back_angle, 30) # get back distance
        if self.state == State.Forward or self.state == State.Turn:
            self.get_logger().info(f'Front distance: {self.front_distance:.2f} m')
        if self.state == State.Backward:
            self.get_logger().info(f'Back distance: {self.back_distance:.2f} m')

        
    def timer_callback(self):        
        # FSM logic
        match self.state:
            case State.Forward:
                # go forward until obstacle is detected
                if self.front_distance < 2:
                    self.state = State.Backward
                    self.backup_timer = 0  # reset backup timer
                    self.get_logger().info('Robot is Moving Backward!')
                    return
                msg = self.forward_msg
                self.publisher_.publish(msg)

            case State.Backward:
                self.get_logger().info(f'Backing up for {self.backup_timer:.1f} seconds')
                # go backward for 1 second or until back is obstructed
                if self.back_distance < 1 or self.backup_timer >= 1:
                    self.state = State.Turn
                    self.get_logger().info('Robot is Turning!')
                    return
                msg = self.backward_msg
                self.publisher_.publish(msg)    
                self.backup_timer += 0.1  # increment backup timer

            case State.Turn:
                # turn until front is clear for 2 meters
                if self.front_distance >= 2:
                    self.state = State.Forward
                    self.get_logger().info('Robot is Moving Forward!')
                    return
                msg = self.turn_msg
                self.publisher_.publish(msg)
            
            case State.Stop:
                # stop the robot
                msg = TwistStamped()  # zero velocities
                self.publisher_.publish(msg)
                self.get_logger().info('Robot has Stopped :(')

            case _: # default case, go to STOP
                self.state = State.Stop


def main(args=None):
    rclpy.init(args=args)

    control_node = ControlNode()

    # Spin function *important*
    # makes sure that the program does not terminate
    # immediately
    rclpy.spin(control_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()