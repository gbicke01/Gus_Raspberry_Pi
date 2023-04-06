import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data
#import tf_transformations
#from rclpy.qos import qos_profile_sensor_data

## Use velocity publisher to control create based on Odom orientation.
class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        print('Creating vel publisher')
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, '/odom', self.odom_callback,  qos_profile_sensor_data)
        timer_period = 0.1
        self.time = self.create_timer(timer_period, self.timer_callback)
        self.wheels = Twist()
        print('Input "forwards" or "turn": ')
        user_input = input()
        if(user_input == 'forwards'):
            self.wheels.linear.x = 0.2
        elif(user_input == 'turn'):
            self.wheels.linear.x = 0.0
            self.wheels.angular.z = -0.5
            self.current_yaw = 0.0
            initial_odom = self.msg
            print(initial_odom)
            self.target_yaw = -math.pi/2

    ## Euler Quaternion coordinates give the yaw (or what angle the create is facing) This helps in turning the robot.
    def odom_callback(self, msg):
        print('Odom callback')
        #msg = Odometry()
        quat = msg.pose.pose.orientation
        print(quat)
        (_, _, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        print('Euler Thing works')
        self.current_yaw = yaw
        print(yaw)

    def timer_callback(self):
        if abs(self.current_yaw - self.target_yaw) > 0.1:  # stop turn if within 0.1 radians of target
            self.vel_publisher.publish(self.wheels)
        else:
            self.vel_publisher.publish(self.wheels)

def main(args=None):
    rclpy.init(args=args)
    move_forward = VelocityPublisher()


    ## This is a proportional control statement that tells when the Create should stop turning if error is below .1
    # turn right until target yaw angle is reached
    while abs(move_forward.current_yaw - move_forward.target_yaw) > 0.1:
        x = move_forward.current_yaw
        y = move_forward.target_yaw
        print('Current yaw: ')
        print("x = {:.2f}".format(x))
        print('Target yaw: ')
        print("y = {:.3f}".format(y))
        rclpy.spin_once(move_forward)
        move_forward.timer_callback()

    move_forward.wheels.angular.z = 0.0  # stop turning once target angle is reached
    move_forward.vel_publisher.publish(move_forward.wheels)

    move_forward.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
