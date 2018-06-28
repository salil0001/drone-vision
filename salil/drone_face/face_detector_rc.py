import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import Tkinter as tk





face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_alt.xml')

if face_cascade.empty():
	raise IOError('Unable to load the face cascade classifier xml file')

cap = cv2.VideoCapture(0)
scaling_factor = 0.5


#vehicle = connect('127.0.0.1:14550', wait_ready=True)
vehicle=connect('/dev/ttyUSB0', baud=57600)
#vehicle=connect('/dev/ttyAMA0', baud=57600)
vehicle.channels.overrides = {}
desired_mode = "LOITER"
while vehicle.mode != desired_mode:
    vehicle.mode = VehicleMode(desired_mode)
    print(" Waiting for mode change ...")
    time.sleep(0.5)
print (" Mode: %s" % vehicle.mode.name)

def drone_cntrl(cpx):

    if((cpx>80) &(cpx<240)):
        vehicle.channels.overrides[4] = 1500
        print("center")
    elif(cpx<90):
        print("right move")
        vehicle.channels.overrides[4] = 1400
    elif(cpx>230):
        print("left move")
        vehicle.channels.overrides[4] = 1600



while True:
    #vehicle.channels.overrides[4] = 1500
    #print (" Ch3 override: %s" % vehicle.channels.overrides[4])
    #print " Ch6: %s" % vehicle.channels['6']

    ret, frame = cap.read()
    frame = cv2.resize(frame, None, fx=scaling_factor, fy=scaling_factor, interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #face_rects = face_cascade.detectMultiScale(gray, 1.3, 5)
    face_rects = face_cascade.detectMultiScale(
        gray,
        scaleFactor=1.3,
        minNeighbors=4,
        minSize=(15, 15)
    )    
    for (x,y,w,h) in face_rects:
    #print(face_rects[0])    
            x, y, w, h = face_rects[0]
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,0,255), 2)
            cpx=(x+x+w)/2
            cpy=(y+y+h)/2
            cv2.circle(frame,(cpx,cpy), 5, (0,0,255), -1)
            if(vehicle.channels['7']<1300):
                drone_cntrl(cpx)
            elif(vehicle.channels['7']>1700):
                vehicle.channels.overrides = {}
                print " Ch4: %s" % vehicle.channels['4']





    cv2.line(frame,(80,20),(80,220),(255,0,255),2)
    cv2.line(frame,(240,20),(240,220),(255,0,255),2)

    cv2.imshow('Drone Controller', frame)

    c = cv2.waitKey(1)
    if c == 27:
        break

cap.release()
cv2.destroyAllWindows()
