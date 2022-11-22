import cv2
import numpy as np
import matplotlib.pyplot as plt
#import serial as ser
import time
import math
from pyfirmata2 import Arduino, PWM , SERVO
import threading

board = Arduino(Arduino.AUTODETECT)
print("Communication Successfully started")
board.digital[5].mode = SERVO
turn = 90
previous_I = 0
previous_error=0
P=0
I=0
D=0
old_turn=90

def make_coordinates(image,line_parameters):
    try:
        slope, intercept = line_parameters
    except TypeError:
        slope, intercept = 0.001,0
    y1 = image.shape[0]  # Height
    y2 = int(y1 * (3/5))
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return np.array([x1,y1,x2,y2]) 

def map(data,MIN,MAX):
    d_min = -280    # 当前数据最大值
    d_max = 280    # 当前数据最小值
    return MIN +(MAX-MIN)/(d_max-d_min) * (data - d_min)

def average_slope_intercept(image,lines):
    left_fit = []
    right_fit = []
    global turn
    if lines is not None:
        for line in lines:
            x1,y1,x2,y2 = line.reshape(4)            
            parameters = np.polyfit((x1, x2),(y1, y2),1)
            slope = parameters[0]
            intercept = parameters[1]
            if slope<0:
                left_fit.append((slope,intercept))
            else :
                right_fit.append((slope,intercept))
        left_fit_average = np.average(left_fit,axis=0)
        right_fit_average = np.average(right_fit,axis=0)
        left_line = make_coordinates(image,left_fit_average)
        right_line = make_coordinates(image,right_fit_average)   
        
        mid1 = ((left_line[0]+left_line[2])/2)
        mid2 = ((right_line[0]+right_line[2])/2)        
        left_dis = (320-mid1)
        right_dis = (mid2-320)
        if abs(right_dis - left_dis)<500:
            direction = (right_dis - left_dis) 
            intdir = int(direction)
            cnum = map(intdir,-40,40)            
            turn = int(cnum)
            #se.write(cnum.encode()) 
            #time.sleep(0.1)
            #se.write("\n".encode())  
            #print(turn)
            
        return np.array([left_line,right_line])

def canny(image):
    gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),5)   # Kernel size is 5x5
    canny = cv2.Canny(blur,30,150)
    return canny

def display_lines(image,lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1,y1,x2,y2 = line.reshape(4)  # Reshaping all the lines to a 1D array.
            try:
                cv2.line(line_image,(int(x1), int(y1)), (int(x2), int(y2)),(0,255,0),10)
                
            except OverflowError:
                pass
            except cv2.error:
                pass
            
            
    return line_image

def region_of_interest(image):
    height = image.shape[0]
    width = image.shape[1]
    rectangle=np.array([[(50, 370), (590, 370),(560, 250), (100, 250)]])
    black_image = np.zeros_like(image)
    mask = cv2.fillPoly(black_image, rectangle, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

#cap = cv2.VideoCapture("v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! appsink", cv2.CAP_GSTREAMER)
cap = cv2.VideoCapture('/dev/video0')
cap.set(cv2.CAP_PROP_FPS, 30)
#se = ser.Serial("/dev/ttyTHS1", 38400, timeout=1)

def pid(Kp,Ki,Kd,error):
    global previous_I,previous_error,P,I,D
    P = error
    I = I + previous_I
    D = error-previous_error
    
    PID_value = (Kp*P)+ (Ki*I) + (Kd*D)   
    previous_I=I
    previous_error=error
    return PID_value

def stablize(old,new):
    angle_deviation = new - old
    if abs(angle_deviation) > 1:
        stabilized_steering_angle = int(old + 1 * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = degree_servo
    return stabilized_steering_angle

while(cap.isOpened()):
    _, frame = cap.read()
    canny_image = canny(frame)
    cropped_image = region_of_interest(canny_image)
    lines = cv2.HoughLinesP(cropped_image,1,np.pi/180,100,np.array([]),minLineLength=5,maxLineGap=5)
    averaged_lines = average_slope_intercept(frame,lines)

    line_image = display_lines(frame,averaged_lines)
    combo_image = cv2.addWeighted(frame,0.8,line_image,1,1)
    
    cv2.imshow('region_of_interest',cropped_image)
    cv2.imshow('Result',combo_image)
    turn1=pid(0.2,0,25,turn)
    degree_servo = 88+turn1
    old_turn = stablize(old_turn, degree_servo)
    if old_turn >= 100:
        old_turn = 100
    if old_turn <= 80:
        old_turn = 80
    board.digital[5].write(old_turn)
    print(old_turn)
    cv2.waitKey(1)
   
