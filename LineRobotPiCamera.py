# Core opencv code provided by Einsteinium Studios
# Revisions to work with Pi Camera v3 by Briana Bouchard

from picamera2 import Picamera2 
import cv2 as cv 
import numpy as np
from libcamera import controls
import time
import RPi.GPIO as GPIO
import board
import digitalio

picam2 = Picamera2()

#configure the picamera
capture_config = picam2.create_still_configuration() #automatically 4608x2592 width by height (columns by rows) pixels
picam2.configure(capture_config)
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) #sets auto focus mode



# ---------------------------------------------------------------------
# MOTOR CODE
# Initialize pins using BCM mode (GPIO pin numbers not board numbers)
# ---------------------------------------------------------------------

yellow = digitalio.DigitalInOut(board.D18)
yellow.direction = digitalio.Direction.OUTPUT
red = digitalio.DigitalInOut(board.D17)
red.direction = digitalio.Direction.OUTPUT
gray = digitalio.DigitalInOut(board.D27)
gray.direction = digitalio.Direction.OUTPUT
green = digitalio.DigitalInOut(board.D22)
green.direction = digitalio.Direction.OUTPUT


yellow2 = digitalio.DigitalInOut(board.D16)
yellow2.direction = digitalio.Direction.OUTPUT
red2 = digitalio.DigitalInOut(board.D20)
red2.direction = digitalio.Direction.OUTPUT
gray2 = digitalio.DigitalInOut(board.D21)
gray2.direction = digitalio.Direction.OUTPUT
green2 = digitalio.DigitalInOut(board.D26)
green2.direction = digitalio.Direction.OUTPUT

# Define direction values
cw = 1
ccw = 0

# Define the steps per revolution for the motor 
steps_rev = 200

def setMotor_right(current_step, delay):
# This function provides the step sequence

    if current_step == 0:
        yellow.value = True
        red.value = False
        gray.value = True
        green.value = False
        time.sleep(delay)

    elif current_step == 1:
        yellow.value = False
        red.value = True
        gray.value = True
        green.value = False
        time.sleep(delay)

    elif current_step == 2:
        yellow.value = False
        red.value = True
        gray.value = False
        green.value = True
        time.sleep(delay)
        
    elif current_step == 3:
        yellow.value = True
        red.value = False
        gray.value = False
        green.value = True
        time.sleep(delay)

def setMotor_left(current_step, delay):
# This function provides the step sequence

    if current_step == 0:
        yellow2.value = True
        red2.value = False
        gray2.value = False
        green2.value = True
        time.sleep(delay)

    elif current_step == 1:
        yellow2.value = False
        red2.value = True
        gray2.value = False
        green2.value = True
        time.sleep(delay)

    elif current_step == 2:
        yellow2.value = False
        red2.value = True
        gray2.value = True
        green2.value = False
        time.sleep(delay)
        
    elif current_step == 3:
        yellow2.value = True
        red2.value = False
        gray2.value = True
        green2.value = False
        time.sleep(delay)

def moveSteps(input_steps, speed, increase_right, increase_left):
    current_step = 0
    delay = 60/(steps_rev*speed)

    # Determines the direction based on sign of input_steps 
    if input_steps > 0:
        direction = ccw
    if input_steps < 0:
        direction = cw
    
    for steps_remaining in range (abs(input_steps), 0, -1):
        if direction == cw: 
            if current_step >= 0 and current_step < 3:
                current_step = current_step + 1
            elif current_step == 3:
                current_step = 0
        if direction == ccw: 
            if current_step <= 3 and current_step > 0:
                current_step = current_step - 1
            elif current_step == 0:
                current_step = 3
                
        setMotor_right(current_step, delay)
        
    print("Stepping complete! Your right motor completed " + str(abs(input_steps)) + " steps at " + str(speed)+ " revolutions per minute")

def moveSteps_right(input_steps, speed):    
# This function tracks the number of steps remaining based on the step input and the loop cycles

    current_step = 0
    delay = 60/(steps_rev*speed)
    
    # Determines the direction based on sign of input_steps 
    if input_steps > 0:
        direction = ccw
    if input_steps < 0:
        direction = cw
    
    for steps_remaining in range (abs(input_steps), 0, -1):
        if direction == cw: 
            if current_step >= 0 and current_step < 3:
                current_step = current_step + 1
            elif current_step == 3:
                current_step = 0
        if direction == ccw: 
            if current_step <= 3 and current_step > 0:
                current_step = current_step - 1
            elif current_step == 0:
                current_step = 3
                
        setMotor_right(current_step, delay)
        
    print("Stepping complete! Your right motor completed " + str(abs(input_steps)) + " steps at " + str(speed)+ " revolutions per minute")

def moveSteps_left(input_steps, speed):    
# This function tracks the number of steps remaining based on the step input and the loop cycles

    current_step = 0
    delay = 60/(steps_rev*speed)
    
    # Determines the direction based on sign of input_steps 
    if input_steps > 0:
        direction = ccw
    if input_steps < 0:
        direction = cw
    
    for steps_remaining in range (abs(input_steps), 0, -1):
        if direction == cw: 
            if current_step >= 0 and current_step < 3:
                current_step = current_step + 1
            elif current_step == 3:
                current_step = 0
        if direction == ccw: 
            if current_step <= 3 and current_step > 0:
                current_step = current_step - 1
            elif current_step == 0:
                current_step = 3
                
        setMotor_left(current_step, delay)
        
    print("Stepping complete! Your left motor completed " + str(abs(input_steps)) + " steps at " + str(speed)+ " revolutions per minute")

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

    #create boundary for red values as two arrays
    lower = np.array([115,0,0]) #lower range of bgr values for red
    upper = np.array([255,70,70]) #upper range of bgr values for red

    #determine if the pixel in the image has bgr values within the range
    image_mask = cv.inRange(img,lower,upper) #returns array of 0s & 255s, 255=white=within range, 0=black=not in range
    cv.imwrite("image2.jpg", image_mask) #write the mask to a new file so that it can be viewed 

    in_range = np.count_nonzero(image_mask) #count the number of elements in the array that are not zero (in other words elements that are in the red range)
    not_in_range = total_pixels[0]*total_pixels[1] - in_range 
    total = total_pixels[0]*total_pixels[1]

    percent_red = round((in_range/total)*100)
    print(percent_red, "%")

    picam2.stop() #stop the picam 

    time.sleep(0.1) # wait to give camera time to start up
    