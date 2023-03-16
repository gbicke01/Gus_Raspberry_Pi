import rclpy
from picamera2 import Picamera2 
import cv2 as cv 
import numpy as np
from libcamera import controls
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data
from keras.models import load_model  # TensorFlow is required for Keras to work
from PIL import Image, ImageOps  # Install pillow instead of PIL
import numpy as np

#PiCam Code and Machine learning
picam2 = Picamera2()
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) #sets auto focus mode

# Disable scientific notation for clarity
np.set_printoptions(suppress=True)

# Load the model
model = load_model("/home/tuftsrobot/Documents/keras_model.h5", compile=False)

# Load the labels
class_names = open("labels.txt", "r").readlines()



WHICH_WAY = ["right", "left", "right", "left", "right", "left", "right"]
DIRECTION = [-0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5]
TURN = [math.pi/2, -math.pi/2, math.pi/2, -math.pi/2, math.pi/2, -math.pi/2, math.pi/2]

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        print('Creating vel publisher')
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, '/odom', self.odom_callback,  qos_profile_sensor_data)
        timer_period = 0.1
        self.wheels = Twist()
        self.turn_jumbo = TURN[0]
        self.jumbo_direction = DIRECTION[0]
        self.turn_kiwi = TURN[1]
        self.kiwi_direction = DIRECTION[1]
        self.turn_mario = TURN[2]
        self.mario_direction = DIRECTION[2]
        self.turn_mug = TURN[3]
        self.mug_direction = DIRECTION[3]
        self.turn_rubiks = TURN[4]
        self.rubiks_direction = DIRECTION[4]
        self.turn_tractor = TURN[5]
        self.tractor_direction = DIRECTION[5]
        self.turn_winnie = TURN[6]
        self.winnie_direction = DIRECTION[6]
        self.turn_nothing = 0
        self.current_yaw = 0.0
        self.target_yaw = 0

        self.wheels.linear.x = 0.3
        self.i = 0

    def odom_callback(self, msg):
        if self.i == 0:
            print('First Odom callback')
            quat = msg.pose.pose.orientation
            print(quat)
            (_, _, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            print('Euler Thing works')
            self.initial_orien = yaw
            self.rawdata_yaw = yaw
            self.current_yaw = self.initial_orien - self.rawdata_yaw
            print(yaw)
        else: 
            print('Odom callback')
            quat = msg.pose.pose.orientation
            print(quat)
            (_, _, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            print('Euler Thing works')
            self.rawdata_yaw = yaw
            self.current_yaw = self.initial_orien - self.rawdata_yaw
            print(yaw)
        
        self.i += 1
        self.vel_publisher.publish(self.wheels)

def rotate(move_forward, direction):
    if(direction == "left"):
        if(move_forward.wheels.angular.z > 0.0):
            print('Turning left')
            # turn right until target yaw angle is reached
            while abs(move_forward.current_yaw - move_forward.target_yaw) > 0.1:
                print('The error value is: ')
                print(abs(move_forward.current_yaw - move_forward.target_yaw))
                x = move_forward.current_yaw
                y = move_forward.target_yaw
                print('Current yaw: ')
                print("x = {:.2f}".format(x))
                print('Target yaw: ')
                print("y = {:.3f}".format(y))
                rclpy.spin_once(move_forward)

            move_forward.wheels.angular.z = 0.0  # stop turning once target angle is reached
            move_forward.wheels.linear.x = 0.3
            move_forward.vel_publisher.publish(move_forward.wheels)
        else:
            rclpy.spin_once(move_forward)
    elif(direction == "right"):
        print(move_forward.wheels.angular.z)
        if(move_forward.wheels.angular.z < 0.0):
            print('Turning right')
        # turn right until target yaw angle is reached
            while abs(move_forward.current_yaw - move_forward.target_yaw) > 0.1:
                print('The error value is: ')
                print(abs(move_forward.current_yaw + move_forward.target_yaw))
                x = move_forward.current_yaw
                y = move_forward.target_yaw
                print('Current yaw: ')
                print("x = {:.2f}".format(x))
                print('Target yaw: ')
                print("y = {:.3f}".format(y))
                rclpy.spin_once(move_forward)
            move_forward.wheels.angular.z = 0.0  # stop turning once target angle is reached
            move_forward.wheels.linear.x = 0.3
            move_forward.vel_publisher.publish(move_forward.wheels)
        else:
            rclpy.spin_once(move_forward)
    else:
            rclpy.spin_once(move_forward)
            



def main(args=None):
    rclpy.init(args=args)
    move_forward = VelocityPublisher()

    #Starts PiCam image processing sequence
    while(True):
        picam2.start()
        time.sleep(0.1)

        img_name = 'image.jpg'
        picam2.capture_file(img_name) #take image
        picam2.stop()

        # Create the array of the right shape to feed into the keras model
        # The 'length' or number of images you can put into the array is
        # determined by the first position in the shape tuple, in this case 1
        data = np.ndarray(shape=(1, 224, 224, 3), dtype=np.float32)

        # Replace this with the path to your image
        image = Image.open("image.jpg").convert("RGB")

        # resizing the image to be at least 224x224 and then cropping from the center
        size = (224, 224)
        image = ImageOps.fit(image, size, Image.Resampling.LANCZOS)

        # turn the image into a numpy array
        image_array = np.asarray(image)

        # Normalize the image
        normalized_image_array = (image_array.astype(np.float32) / 127.5) - 1

        # Load the image into the array
        data[0] = normalized_image_array

        # Predicts the model
        print('Getting Predictions')
        prediction = model.predict(data)
        index = np.argmax(prediction)
        class_name = class_names[index]
        confidence_score = prediction[0][index]

        # Print prediction and confidence score
        print("Class:", class_name[2:], end="")
        print("Confidence Score:", confidence_score)

        #self.current_yaw = 0.0
        if(''.join(str(class_name[2:]).strip()) == "Jumbo" and confidence_score >= np.float32(0.90)):
            print('Jumbo detected')
            os.system('ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "{angle: 6.28,max_rotation_speed: 1.1}"')
            break
        elif(''.join(str(class_name[2:]).strip()) == "Kiwi" and confidence_score >= np.float32(0.90)):
            print('Kiwi detected')
            move_forward.target_yaw = move_forward.turn_kiwi
            move_forward.wheels.angular.z = move_forward.kiwi_direction
            move_forward.wheels.linear.x = 0.0
            move_forward.vel_publisher.publish(move_forward.wheels)
            move_forward.i = 0
            rotate(move_forward, WHICH_WAY[1])
        elif(''.join(str(class_name[2:]).strip()) == "Mario" and confidence_score >= np.float32(0.90)):
            print('Mario detected')
            move_forward.target_yaw = move_forward.turn_mario
            move_forward.wheels.angular.z = move_forward.mario_direction
            move_forward.wheels.linear.x = 0.0
            move_forward.vel_publisher.publish(move_forward.wheels)
            move_forward.i = 0
            rotate(move_forward, WHICH_WAY[2])
        elif(''.join(str(class_name[2:]).strip()) == "Mug" and confidence_score >= np.float32(0.90)):
            print('Mug detected')
            move_forward.target_yaw = move_forward.turn_mug
            move_forward.wheels.angular.z = move_forward.mug_direction
            move_forward.wheels.linear.x = 0.0
            move_forward.vel_publisher.publish(move_forward.wheels)
            move_forward.i = 0
            rotate(move_forward, WHICH_WAY[3])
        elif(''.join(str(class_name[2:]).strip()) == "Rubiks" and confidence_score >= np.float32(0.90)):
            print('Rubiks detected')
            move_forward.target_yaw = move_forward.turn_rubiks
            move_forward.wheels.angular.z = move_forward.rubiks_direction
            move_forward.wheels.linear.x = 0.0
            move_forward.vel_publisher.publish(move_forward.wheels)
            move_forward.i = 0
            rotate(move_forward, WHICH_WAY[4])
        elif(''.join(str(class_name[2:]).strip()) == "Tractor" and confidence_score >= np.float32(0.90)):
            print('Tractor detected')
            move_forward.target_yaw = move_forward.turn_tractor
            move_forward.wheels.angular.z = move_forward.tractor_direction
            move_forward.wheels.linear.x = 0.0
            move_forward.vel_publisher.publish(move_forward.wheels)
            move_forward.i = 0
            rotate(move_forward, WHICH_WAY[5])
        elif(''.join(str(class_name[2:]).strip()) == "Winnie" and confidence_score >= np.float32(0.90)):
            print('Sees Winnie')
            move_forward.target_yaw = move_forward.turn_winnie
            move_forward.wheels.angular.z = move_forward.winnie_direction
            move_forward.wheels.linear.x = 0.0
            move_forward.vel_publisher.publish(move_forward.wheels)
            move_forward.i = 0
            rotate(move_forward, WHICH_WAY[6])
        else:
            rotate(move_forward, "none")
        
            

        

    move_forward.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()