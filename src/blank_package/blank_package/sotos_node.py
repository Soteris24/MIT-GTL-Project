#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped, LEDPattern, WheelEncoderStamped

class SkeletonNode(Node):
    def __init__(self):
        super().__init__('blank_node')
        
        self.vehicle_name = os.getenv('VEHICLE_NAME')
        self.tof_sub = self.create_subscription(Range, f'/{self.vehicle_name}/range', self.check_range, 10)
        self.wheel_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10)
        self.led_pub = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)  
        self.tick_sub = self.create_subscription(WheelEncoderStamped, f'/{self.vehicle_name}/tick', self.tick_callback, 10)
        
        # State machine
        self.state = 'normal'
        self.avoidance_timer = None
        
        # Encoder ticks
        self.left_ticks = 0
        self.right_ticks = 0
        
        # Control loop at 10Hz
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def tick_callback(self, msg):
        """Callback for wheel encoder ticks"""
        if 'left' in msg.header.frame_id:
            self.left_ticks = msg.data
        elif 'right' in msg.header.frame_id:
            self.right_ticks = msg.data

    def control_loop(self):
        """Main control loop - keeps robot going straight"""
        if self.state == 'normal':
            # Calculate error (difference between wheels)
            error = self.left_ticks - self.right_ticks
            
            # Base speed
            base_speed = 0.3
            
            # Proportional gain - tune this!
            kp = 0.0001  # Start small since tick values are large
            
            # Adjust speeds based on error
            left_speed = base_speed - (kp * error)
            right_speed = base_speed + (kp * error)
            
            # Limit speeds
            left_speed = max(0.1, min(0.5, left_speed))
            right_speed = max(0.1, min(0.5, right_speed))
            
            self.run_wheels('straight', left_speed, right_speed)

    def check_range(self, msg):
        distance = msg.range
        if self.state == 'normal':
            if distance < 0.1 and distance > 0.02:
                self.start_avoidance()

    def move_forward(self):
        self.run_wheels('forward', 0.3, 0.3)

    def turn_left(self):
        self.run_wheels('left', -0.3, 0.3)

    def turn_right(self):
        self.run_wheels('right', 0.3, -0.3)

    def stop(self):
        self.run_wheels('stop', 0.0, 0.0)

    def start_avoidance(self):
        self.get_logger().info('Obstacle detected! Starting avoidance...')
        self.set_leds_red()
        self.state = 'avoiding_turn_right'
        self.turn_right()
        self.avoidance_timer = self.create_timer(0.5, self.avoidance_step_forward)

    def avoidance_step_forward(self):
        self.avoidance_timer.cancel()
        self.get_logger().info('Going forward...')
        self.set_leds_yellow()
        self.state = 'avoiding_forward'
        self.move_forward()
        self.avoidance_timer = self.create_timer(2.0, self.avoidance_step_turn_left)

    def avoidance_step_turn_left(self):
        self.avoidance_timer.cancel()
        self.get_logger().info('Turning left...')
        self.state = 'avoiding_turn_left'
        self.turn_left()
        self.avoidance_timer = self.create_timer(0.5, self.avoidance_complete)

    def avoidance_complete(self):
        self.avoidance_timer.cancel()
        self.get_logger().info('Avoidance complete!')
        self.set_leds_green()
        self.state = 'normal'

    def run_wheels(self, frame_id, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()
        wheel_msg.header.stamp = self.get_clock().now().to_msg()
        wheel_msg.header.frame_id = frame_id
        wheel_msg.vel_left = vel_left
        wheel_msg.vel_right = vel_right
        self.wheel_pub.publish(wheel_msg)

    def set_leds_red(self):
        msg = LEDPattern()
        pattern = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        msg.rgb_vals = [pattern] * 5
        self.led_pub.publish(msg)

    def set_leds_green(self):
        msg = LEDPattern()
        pattern = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        msg.rgb_vals = [pattern] * 5
        self.led_pub.publish(msg)

    def set_leds_yellow(self):
        msg = LEDPattern()
        pattern = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        msg.rgb_vals = [pattern] * 5
        self.led_pub.publish(msg)

def main():
    print('In main')
    rclpy.init()
    node = SkeletonNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop() 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()