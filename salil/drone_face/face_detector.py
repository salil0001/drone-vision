import cv2
import numpy as np
import time
import math


face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_alt.xml')

if face_cascade.empty():
    raise IOError('Unable to load the face cascade classifier xml file')

cap = cv2.VideoCapture(0)
scaling_factor = 0.5

while True:
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

    if (len(face_rects) == 0):
        print("waiting for target")
    else:
        for (x,y,w,h) in face_rects:
            x, y, w, h = face_rects[0]
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 3)
            print(w)            
    

    cv2.imshow('Face Detector', frame)
    print('FPS {:.1f}'.format(1 / (time.time() - stime)))
    c = cv2.waitKey(1)
    if c == 27:
        break

cap.release()
cv2.destroyAllWindows()
