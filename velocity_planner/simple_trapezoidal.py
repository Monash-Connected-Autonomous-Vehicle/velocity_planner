"""
Node that tells the vehicle to wait at rest, then speed up to a certain max velocity, remain at that velocity,
then slow down until it reaches rest, then remain at rest.
This can be used to get a good idea of nice max speed and acceleration/deceleration values to use when
controlling the car.
"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped

class SimpleTrapezoidalPlanner(Node):

    def __init__(self):
        super().__init__('simple_trapezoidal_publisher')
        self.publisher_ = self.create_publisher(TwistStamped, '/twist_cmd', 10)

        self.start_delay = 5 # seconds
        self.initial_time_ns = self.get_clock().now().nanoseconds

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = TwistStamped()
        current_time_ns = self.get_clock().now().nanoseconds
        elapsed_time = float(current_time_ns - self.initial_time_ns) / 10**9
        command_vel = compute_vel_trapezoidal(time=elapsed_time, max_vel=1.0, acceleration_amplitude=0.1, deceleration_amplitude=-1.0, start_delay=0.0, constant_vel_period=5.0)
        self.get_logger().info(f'{command_vel}')
        msg.twist.linear.x = command_vel
        self.publisher_.publish(msg)

def compute_vel_trapezoidal(time: float, max_vel: float, acceleration_amplitude: float, deceleration_amplitude: float, start_delay: float, constant_vel_period: float):
    """ Generates a trapezoidal velocity profile with the given parameters, accelerating from zero velocity at time zero.
    time: (s) Elapsed time
    max_vel: (m/s) Velocity to which the vehicle will be commanded to accelerate. The height of the trapezoid
    acceleration_amplitude: (m/s^2) Slope of the positive ramp (must be +ve)
    deceleration_amplitude: (m/s^2) Absolute value of the slope of the negative ramp (must be +ve)
    start_delay: (s) Time to wait before beginning initial acceleration
    constant_vel_period: (s) Time for which the vehicle will be commanded to stay at constant velocity

    returns a commanded velocity (m/s)

    Graph of velocity over time:
                                       max_vel
                               _________________________
                              /                         \
                             /                           \
                            /                             \
                           /                               \
    0m/s__________________/                                 \__________
        |<--start_delay-->|    |<-constant_vel_period->|
       t=0s
    """
    # TODO: Validate parameters

    ramp_up_time = max_vel / acceleration_amplitude
    ramp_down_time = max_vel / deceleration_amplitude

    # Stop the car to begin with
    if time < start_delay:
        command_vel = 0.0
    # Initial acceleration
    elif time < start_delay + ramp_up_time:
        command_vel = (time-start_delay) * acceleration_amplitude
    # Constant velocity
    elif time < start_delay + ramp_up_time + constant_vel_period:
        command_vel = max_vel
    # Final deceleration
    elif time < start_delay + ramp_up_time + constant_vel_period + ramp_down_time:
        command_vel = max_vel - abs(deceleration_amplitude)*(time - start_delay - ramp_up_time - constant_vel_period)
    # Zero velocity
    else:
        command_vel = 0.0

    return command_vel

def main(args=None):
    rclpy.init(args=args)

    simple_trapezoidal = SimpleTrapezoidalPlanner()
    rclpy.spin(simple_trapezoidal)
    simple_trapezoidal.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
