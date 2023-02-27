# Takes an image using a raspberry pi camera & finds the percentage of pixels in the image that are red
# by Maddie Pero
# Revisions made by Gus Bickert and Calder Mazel

from picamera2 import Picamera2 
import cv2 as cv 
import numpy as np
from libcamera import controls
import time
import RPi.GPIO as GPIO
from ThreadStepperLib import Stepper
# import board
# import digitalio

# Pin numbers for right and left motor drivers
rin1 = 12
rin2 = 11
rin3 = 13
rin4 = 15
lin1 = 36
lin2 = 38
lin3 = 40
lin4 = 37



picam2 = Picamera2()

#configure the picamera
capture_config = picam2.create_still_configuration() #automatically 4608x2592 width by height (columns by rows) pixels
picam2.configure(capture_config)
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) #sets auto focus mode



# ---------------------------------------------------------------------
# MOTOR CODE
# Initialize pins using BCM mode (GPIO pin numbers not board numbers)
# ---------------------------------------------------------------------



# Define the GPIO pins for the L298N motor driver
Motor1 = [12,11,13,15]
Motor2 = [36,38,40,37]

def moveSteps(step_right, step_left):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(rin1, GPIO.OUT)
    GPIO.setup(rin2, GPIO.OUT)
    GPIO.setup(rin3, GPIO.OUT)
    GPIO.setup(rin4, GPIO.OUT)
    GPIO.output(rin1, GPIO.LOW)
    GPIO.output(rin2, GPIO.LOW)
    GPIO.output(rin3, GPIO.LOW)
    GPIO.output(rin4, GPIO.LOW)

    GPIO.setup(lin1, GPIO.OUT)
    GPIO.setup(lin2, GPIO.OUT)
    GPIO.setup(lin3, GPIO.OUT)
    GPIO.setup(lin4, GPIO.OUT)
    GPIO.output(lin1, GPIO.LOW)
    GPIO.output(lin2, GPIO.LOW)
    GPIO.output(lin3, GPIO.LOW)
    GPIO.output(lin4, GPIO.LOW)
    try:
        # Define the steps per revolution for the motor 
        steps_rev = 200

        # Set the thread number, thread ID, motor, number of steps to move, steps per revolution,
        # and the speed in revolutions per minute
        stepper1 = Stepper(1,"Motor #1",Motor1, -1-step_right, steps_rev, 20)
        stepper2 = Stepper(2,"Motor #2",Motor2, 1+step_left, steps_rev, 20)

        # Start the motor threads
        stepper1.start()
        stepper2.start()

        # Check to see if both threads are done and clean up GPIO pins when done
        while True:
            if stepper1.is_alive() == False and stepper2.is_alive() ==False:
                GPIO.cleanup()
                break
    except KeyboardInterrupt:
        GPIO.cleanup()


#----------------------------------------
#WE'RE ONLY USING PROPORTIONAL CONTROL
#IGNORE COMMENTED OUT CODE IN THIS BLOCK
#----------------------------------------
# def PID(color, grey_val):

#     #For example, let's say we want to follow a black line.
#     #We need to find the error of how far the sensor veers from the line
#     #This function passes in our desired color value (for black it is around 300)
#     #Black is sensed by all color values (r, g, b), so we need to add up their total 
#     # and subtract it from our desired color value

#     error = color - (grey_val)
#     #Since r, g, b are high values when reading the white paper, this error value will be a
#     #very large negative value


#     # global integral = integral + error
#     # temp_derivative = error - derivative
#     # global derivative = temp_derivative
#     # PID = error+integral+derivative
#     # return PID


#     print("This is the error:", error)
#     return error
#     #Returns the error value

while(True):
    picam2.start() #must start the camera before taking any images
    time.sleep(0.1)

    img_name = 'image.jpg'
    picam2.capture_file(img_name) #take image 

    img = cv.imread("image.jpg") #read image with open cv, to get the bgr value of one pixel index using print(img[row][col])

    total_pixels = img.shape #returns [2529, 4608] as the shape of the image
    
    print("BGR values are: ",img[1260][2300])

    #create boundary for blue values as two arrays
    lower = np.array([115,0,0]) #lower range of bgr values for blue
    upper = np.array([255,70,70]) #upper range of bgr values for blue

    #determine if the pixel in the image has bgr values within the range
    image_mask = cv.inRange(img,lower,upper) #returns array of 0s & 255s, 255=white=within range, 0=black=not in range
    cv.imwrite("image2.jpg", image_mask) #write the mask to a new file so that it can be viewed 

    in_range = np.count_nonzero(image_mask) #count the number of elements in the array that are not zero (in other words elements that are in the blue range)
    not_in_range = total_pixels[0]*total_pixels[1] - in_range 
    total = total_pixels[0]*total_pixels[1]

    percent_blue = round((in_range/total)*100)
    print(percent_blue, "%")

    picam2.stop() #stop the picam 

    if percent_blue >= 7:
        moveSteps(1, 2)
    elif percent_blue <= 1:
        moveSteps(2, 1)
    else:
        moveSteps(1, 1)
    