#!/usr/bin/python
import nrf
import numpy as np
import cv2
import math
import time

nrf=nrf.bridge('/dev/ttyACM0')#'/dev/ttyACM0'
a=nrf.assign_addresses()
for i in a.keys():
    nrf.set_TX_address(i)
    if nrf.get_ID_type()[6]==1: #find first camera in neighbours
        camera_address=i
        break
else: raise RuntimeError('No cameras found')

#nrf.send_packet('\x94'+chr(64)) #overclock PSoC and camera (default is 48MHz, which gives 15fps SXGA, 30fps VGA, 60fps CIF and 120fps QCIF)

class thresholds:
    values=[0x79,0x9a,0xb1,0x5a,0xc6,0x70,0xa3,0x82]
    def __init__(self,i):
        self.i=i
    
    def __call__(self,value):
        old=thresholds.values[self.i]
        thresholds.values[self.i]=value
        try:
            nrf.set_camera_thresholds(thresholds.values)
        except RuntimeError as e: #ACK not received, set trackbar back to original value
            thresholds.values[self.i]=old
            cv2.setTrackbarPos(['Red','Green','Blue','Magenta'][self.i/2]+' '+['U','V'][self.i%2],'Blob camera',old)

cv2.namedWindow('Blob camera')
i=0
for colour in ['Red','Green','Blue','Magenta']:
    for channel in ['U','V']:
        cv2.createTrackbar(colour+' '+channel,'Blob camera',thresholds.values[i],255,thresholds(i))
        i+=1

ts=0
frame_blobs=[]
colours=[(0,0,255),(0,255,0),(255,0,0),(255,0,255)]
frame_times=zip(range(10),range(10))
while True:
    try:
        k=cv2.waitKey(1)
        if chr(k&0xff)=='q': break
        prev_ts=ts
        ts,blobs=nrf.get_blobs()
        if ts!=prev_ts:
            img=np.zeros((480,640,3),np.uint8)
            for b in frame_blobs:
		if b[2]==3:
		  cv2.circle(img,(b[0]/2,b[1]/2),int(b[3]/math.pi**0.5/2),colours[2],-1,cv2.CV_AA)
		else:
		  cv2.circle(img,(b[0]/2,b[1]/2),int(b[3]/math.pi**0.5/2),colours[b[2]],-1,cv2.CV_AA)
		#print 'x, y:',b[0]/2,b[1]/2 
		print b[3]
            old_time=frame_times[0]
            new_time=(ts,time.time())
            frame_times=frame_times[1:]+[new_time]
            fps_camera=1000.*len(frame_times)/(new_time[0]-old_time[0])
            fps_local=len(frame_times)/(new_time[1]-old_time[1])
            cv2.putText(img,'%03.0f'%fps_camera,(0,20),cv2.FONT_HERSHEY_PLAIN,1,(128,128,0),1,cv2.CV_AA)
            cv2.putText(img,'%03.0f'%fps_local,(0,50),cv2.FONT_HERSHEY_PLAIN,1,(128,128,0),1,cv2.CV_AA)
            cv2.imshow('Blob camera',img)
            frame_done=True
            frame_blobs=[]
        
        frame_blobs+=blobs
        
    except RuntimeError as e:
        continue #TODO check what kind of error
