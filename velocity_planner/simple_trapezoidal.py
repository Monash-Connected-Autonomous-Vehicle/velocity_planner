import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import TwistStamped

class SimpleTrapezoidalPlanner(Node):

    def __init__(self):
        super().__init__('simple_trapezoidal_publisher')
        self.publisher_ = self.create_publisher(TwistStamped, 'twist_cmd', 10)

        self.start_delay = 5 # seconds
        self.initial_time_ns = self.get_clock().now().nanoseconds

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.waypoints = []
        # self.init_waypoints()

    def timer_callback(self):
        msg = TwistStamped()
        # msg.waypoints = self.waypoints
        # self.publisher_.publish(msg)
        current_time_ns = self.get_clock().now().nanoseconds
        elapsed_time = float(current_time_ns - self.initial_time_ns) / 10**9
        command_vel = compute_vel_trapezoidal(elapsed_time, 5.0, 1.0, 1.0, 5.0, 5.0)
        self.get_logger().info(f'{command_vel}')

def compute_vel_trapezoidal(time: float, max_vel: float, max_accel: float, min_accel: float, start_delay: float, constant_vel_period: float):
    """ Generates a trapezoidal velocity profile with the given parameters, accelerating from zero velocity at time zero.
    time: (s) Elapsed time
    max_vel: (m/s) Velocity to which the vehicle will be commanded to accelerate. The height of the trapezoid
    max_accel: (m/s^2) Slope of the positive ramp 
    min_accel: (m/s^2) Slope of the negative ramp 
    start_delay: (s) Time to wait before beginning initial acceleration
    constant_vel_period: (s) Time for which the vehicle will be commanded to stay at constant velocity

    returns a commanded velocity (m/s)
    """
    # TODO: Validate parameters

    ramp_up_time = max_vel/max_accel
    ramp_down_time = max_vel / abs(min_accel)

    # Initial acceleration
    if time < start_delay:
        command_vel = 0
    elif time < start_delay + ramp_up_time:
        command_vel = (time-start_delay) * max_accel
    # Constant velocity
    elif time < start_delay + ramp_up_time + constant_vel_period:
        command_vel = max_vel
    # Final deceleration
    elif time < start_delay + ramp_up_time + constant_vel_period + ramp_down_time:
        command_vel = max_vel - min_accel*(time - start_delay - ramp_up_time - constant_vel_period)
    # Zero velocity
    else:
        command_vel = 0

    return command_vel

def main(args=None):
    rclpy.init(args=args)

    simple_trapezoidal = SimpleTrapezoidalPlanner()
    rclpy.spin(simple_trapezoidal)
    simple_trapezoidal.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
