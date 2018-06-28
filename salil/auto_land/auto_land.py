import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
import math


face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_alt.xml')

if face_cascade.empty():
	raise IOError('Unable to load the face cascade classifier xml file')


cap = cv2.VideoCapture(0)
scaling_factor = 0.5


vehicle = connect('127.0.0.1:14550', wait_ready=True)
#vehicle=connect('/dev/ttyUSB0', baud=57600)
#vehicle=connect('/dev/ttyS0', baud=57600)
#vehicle=connect('/dev/ttyAMA0', baud=57600)
vehicle.channels.overrides = {}

'''
desired_mode = "LOITER"
while vehicle.mode != desired_mode:
    vehicle.mode = VehicleMode(desired_mode)
    print(" Waiting for mode change ...")
    time.sleep(0.5)
print (" Mode: %s" % vehicle.mode.name)
'''
            

while True:
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
    if len(face_rects)>=1:
        for (x,y,w,h) in face_rects:     
        #print(face_rects[0])    
            x, y, w, h = face_rects[0]
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,0,255), 2)
            cpx=(x+x+w)/2
            cpy=(y+y+h)/2
            cv2.circle(frame,(cpx,cpy), 5, (0,0,255), -1)
            #drone_cntrl(cpx,w)
            if(vehicle.channels['7']>1700):
                print("tracking mode")
                while vehicle.mode != "LAND":
                    vehicle.mode = VehicleMode("LAND")
                    print(" Waiting for mode change to LAND ...")
                    time.sleep(0.5)
                print (" Mode: %s" % vehicle.mode.name)
            elif(vehicle.channels['7']<1300):
                print("plz enable tracking mode")    
                
                
    else:
        vehicle.channels.overrides = {}
        print("wating for face")
          
    cv2.imshow('Drone Controller', frame)
    #print('FPS {:.1f}'.format(1 / (time.time() - stime)))   
    c = cv2.waitKey(1)
    if c == 27:
        break

cap.release()
cv2.destroyAllWindows()
