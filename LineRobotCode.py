import RPi.GPIO as GPIO
import time
import board
import digitalio
from adafruit_apds9960.apds9960 import APDS9960
from adafruit_apds9960 import colorutility
#sure
i2c = board.I2C() # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C() # For using the built-in STEMMA QT connector on a microcontroller
apds = APDS9960(i2c)
apds.enable_color = True


# Initialize pins using BCM mode (GPIO pin numbers not board numbers)
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
def PID(color, r, g, b):

    #For example, let's say we want to follow a black line.
    #We need to find the error of how far the sensor veers from the line
    #This function passes in our desired color value (for black it is around 300)
    #Black is sensed by all color values (r, g, b), so we need to add up their total 
    # and subtract it from our desired color value

    error = color - (r+g+b)
    #Since r, g, b are high values when reading the white paper, this error value will be a
    #very large negative value


    # global integral = integral + error
    # temp_derivative = error - derivative
    # global derivative = temp_derivative
    # PID = error+integral+derivative
    # return PID


    print("This is the error:", error)
    return error
    #Returns the error value


try:
    while True:
        # wait for color data to be ready
        while not apds.color_data_ready:
            time.sleep(0.001)

        # get the data and print the different channels
        r, g, b, c = apds.color_data
        print("red: ", r)
        print("green: ", g)
        print("blue: ", b)
        print("clear: ", c)
        print("color temp {}".format(colorutility.calculate_color_temperature(r, g, b)))
        print("light lux {}".format(colorutility.calculate_lux(r, g, b)))
        time.sleep(0.1)

        black = r+g+b
        print('THIS IS BLACK VALUE: ',black)
        #-----------------------------------
        #To control right motor
        #-----------------------------------
        


        #This if statement will add steps to a running motor the color values become too large
        #Again, the rgb values for black are low, so this makes sure the sensor stays on the black line
        #if r > 1400 and g > 800 and b > 600:
        if black > 3000:
            

            #the round() in moveSteps() rounds the equation to a whole number since only
            #integers can be read in moveSteps(). The equation 200 - (PID(300, r, g, b)/200)
            #Will add steps to the typical 200 steps for the motor. Let's say the error is 
            #very large, the PID() function will return a large negative value. Dividing it
            #by 200 proportionalizes the error in terms of steps and subtracting it from 200
            #actually adds it(since it's trying to subtract a negative).
            moveSteps_right(round((-5 + (PID(2800, r, g, b)*0.001))), 20)
            print("Moves ", round((-5 + (PID(2800, r, g, b)*0.001)))," steps")
            print("Color sensor reads black ")



        #-----------------------------------
        #To control left motor
        #-----------------------------------


        #if r < 1400 and g < 800 and b < 600:
        if black < 3000:    

            #the round() in moveSteps() rounds the equation to a whole number since only
            #integers can be read in moveSteps(). The equation 200 - (PID(300, r, g, b)/200)
            #Will add steps to the typical 200 steps for the motor. Let's say the error is 
            #very large, the PID() function will return a large negative value. Dividing it
            #by 200 proportionalizes the error in terms of steps and subtracting it from 200
            #actually adds it(since it's trying to subtract a negative).
            moveSteps_left(round((-5 + (PID(2800, r, g, b)*0.001))), 20)
            print("Moves ", round((-5 + (PID(2800, r, g, b)*0.001)))," steps")
            print("Color sensor reads black ")

        
        # Turn off GPIO pins
        #GPIO.cleanup()
        #break

except KeyboardInterrupt:
    # Turn off GPIO pins
    GPIO.cleanup()