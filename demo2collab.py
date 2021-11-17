from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2 
from cv2 import aruco
import numpy
import numpy as np
from statistics import mean
import math
import time
import smbus
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import smbus
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
angle = 0
new_angle = ""
columns = 16
rows = 2
#for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)
#time.sleep(2)

# Modify this if you have a different sized Character LCD

# Initialise I2C bus.
i2c = busio.I2C(board.SCL,board.SDA)

lcd = character_lcd.Character_LCD_RGB_I2C(i2c, columns, rows)

lcd.clear()

#this is the address we setup in the Arduino Program
address = 0x04

def computerVision():
    global angle
    global tapefound
    global tape
    camera = PiCamera()
    rawCapture= PiRGBArray(camera)
    #camera.resolution = (320,240)#new
   # camera.framerate =24#new
    #image = np.empty((240,320,3), dtype=np.uint8)#new
    camera.iso=100
    time.sleep(2)
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    gain = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = gain
    time.sleep(0.1)
    print("Capturing Image...")
    try:
        camera.capture(rawCapture, format="bgr")
        image = rawCapture.array
        camera.close()
    except:
        print("Failed to capture")
    tapefound = False    
    height = (image.shape[0])/2
    width = (image.shape[1])/2
    Filter = cv2.GaussianBlur(image,(5,5),0) 
    imagehsv = cv2.cvtColor(Filter, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5,5),np.uint8)
    lowthresh = (98,150,100)
    highthresh = (110,255,255)
    mask = cv2.inRange(imagehsv, lowthresh, highthresh)
    mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN, kernel)
    blueiso = cv2.bitwise_and(image,image,mask=mask)
    
    
    
    ret,thresh = cv2.threshold(mask,127,255,0)
    Y,X = np.nonzero(thresh)
    cX = X.mean()
    cY= Y.mean()
    if cX !=0:
        tapefound = True
        tape = "1"
        angle = (27*(cX-width))/width
        angle = round(angle,2)
        angle = str(angle)
        
    else:
        tapefound = False
        tape = "0"
        print("No tape found")
        lcd.color = [100,0,0]
        lcd.text_direction = lcd.LEFT_TO_RIGHT
        lcd.message = "No angle Found"        
        
        
def systemIntegration():
    global angle
    angle = angle.lower()
    number = []
    #lcd.message = "Angle: " + str(angle)  
    for char in angle:
        number.append(int ( ord(char)))
    number.append(32)
    for char in tape:
        number.append(int ( ord(char)))
    if number[0] == 110:
#        tape = 0
        number = []
        number.append(32)
        number.append(48)
        print(number)
        try:
            bus.write_i2c_block_data(address,0,number)
        except IOError:
            lcd.color = [100,0,0]
            lcd.text_direction = lcd.LEFT_TO_RIGHT
            lcd.message = "I2C Error "
        lcd.color = [100,0,0]
        lcd.text_direction = lcd.LEFT_TO_RIGHT
        lcd.message = "No tape found"
        print ("RPI: Nothing was sent to the arduino.")
#        break
    else:
        lcd.color = [100,100,100]
        lcd.text_direction = lcd.LEFT_TO_RIGHT
        lcd.message = "Angle: " + str(angle)
    if number[0] != 110:
        try:
            bus.write_i2c_block_data(address,0,number)
        except IOError:
            lcd.color = [100,0,0]
            lcd.text_direction = lcd.LEFT_TO_RIGHT
            lcd.message = "I2C Error "
        
        print ("RPI: ", number, " was sent to the ardiuno.")
    
       #sleep one second
    #time.sleep(1)
#    for char in tape:
#        dis.append(int ( ord(char)))
#    if dis[0] != 110:
#        try:
#            bus.write_i2c_block_data(address,0,number)
#        except IOError:
#            lcd.color = [100,0,0]
#            lcd.text_direction = lcd.LEFT_TO_RIGHT
#            lcd.message = "I2C Error "
#        
#        print ("RPI: ", number, " was sent to the ardiuno.")
    
       #sleep one second
    #time.sleep(1)
#    number = bus.read_i2c_block_data(address,0,len(number))


while True:
     computerVision()
     if tapefound == True:
         systemIntegration()
         #time.sleep(1)
         lcd.clear()
     else:
#        number = []
#        number.append[32]
#        number.append[48]
#        print(number)
#        try:
#            bus.write_i2c_block_data(address,0,number)
#        except IOError:
#            lcd.color = [100,0,0]
#            lcd.text_direction = lcd.LEFT_TO_RIGHT
#            lcd.message = "I2C Error "
        lcd.clear()
