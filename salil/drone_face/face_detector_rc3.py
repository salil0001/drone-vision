import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math


face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_alt.xml')

if face_cascade.empty():
	raise IOError('Unable to load the face cascade classifier xml file')

cap = cv2.VideoCapture(0)
scaling_factor = 0.5


vehicle = connect('127.0.0.1:14550', wait_ready=True)
#vehicle=connect('/dev/ttyUSB0', baud=57600)
#vehicle=connect('/dev/ttyAMA0', baud=57600)
#vehicle=connect('/dev/ttyS0', baud=57600)

vehicle.channels.overrides = {}
desired_mode = "LOITER"
while vehicle.mode != desired_mode:
    vehicle.mode = VehicleMode(desired_mode)
    print(" Waiting for mode change ...")
    time.sleep(0.5)
print (" Mode: %s" % vehicle.mode.name)

def drone_cntrl(cpx,fw):

    if((cpx>100) &(cpx<220)):
        vehicle.channels.overrides[4] = 1500
        #print("center")

        if((fw>45)&(fw<60)):
            vehicle.channels.overrides[2] = 1500    
            print("Normal")            
        elif((fw>60)&(fw<120)):
            vehicle.channels.overrides[2] = 1700    
            print("pitch down")            
        elif((fw>10)&(fw<45)):
            vehicle.channels.overrides[2] = 1300    
            print("pitch up")
        
    elif(cpx<100):
        #print("right yaw")
        vehicle.channels.overrides[4] = 1400
    elif(cpx>220):
        #print("left yaw")
        vehicle.channels.overrides[4] = 1600
    '''    
        
    
        '''
            

while True:
    #vehicle.channels.overrides[4] = 1500
    #print (" Ch3 override: %s" % vehicle.channels.overrides[4])
    #print " Ch6: %s" % vehicle.channels['6']
    stime = time.time()    
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
            
            #if(vehicle.channels['7']>1700):
            if(1):                    
                #print("trackin")                    
                drone_cntrl(cpx,w)
            elif(vehicle.channels['7']<1300):
                vehicle.channels.overrides = {}
                #print(" Ch4: %s" % vehicle.channels['4'])
                #print(" Ch2: %s" % vehicle.channels['2'])
                
    else:
        vehicle.channels.overrides = {}
        print("wating for face")
          
              
              
                

    cv2.line(frame,(100,20),(100,220),(255,0,255),2)
    cv2.line(frame,(220,20),(220,220),(255,0,255),2)

    cv2.imshow('Drone Controller', frame)
    #print('FPS {:.1f}'.format(1 / (time.time() - stime)))   
    c = cv2.waitKey(1)
    if c == 27:
        break

cap.release()
cv2.destroyAllWindows()
