#!/usr/bin/python
import nrf
import numpy as np
import cv2
import math
import time
from sklearn.neighbors import NearestNeighbors
import scipy.sparse.csgraph

import circle as CL
import clustering as CLTR

SHOW_NEIGHBORS = True
LED_DETECTION_THRESHOLD = 6
ix,iy = -1,-1

nrf=nrf.bridge('/dev/ttyACM0')#'/dev/ttyACM0'
a=nrf.assign_addresses()
for i in a.keys():
    nrf.set_TX_address(i)
    if nrf.get_ID_type()[6]==1: #find first camera in neighbours
        camera_address=i
        break
else: raise RuntimeError('No cameras found')

def draw_circle(event,x,y,flags,param):
    global ix,iy
    if event == cv2.EVENT_LBUTTONDBLCLK:
        destination = [x,y]
        print destination


#nrf.send_packet('\x94'+chr(64)) #overclock PSoC and camera (default is 48MHz, which gives 15fps SXGA, 30fps VGA, 60fps CIF and 120fps QCIF)
nrf.camera_write_reg(0x10,20)
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
cv2.setMouseCallback('Blob camera',draw_circle)
i=0
for colour in ['Red','Green','Blue','Magenta']:
    for channel in ['U','V']:
        cv2.createTrackbar(colour+' '+channel,'Blob camera',thresholds.values[i],255,thresholds(i))
        i+=1

ts=0
frame_blobs=[]
colours=[(0,0,255),(0,255,0),(255,0,0),(255,0,255)]
frame_times=zip(range(10),range(10))
flg = True
destination = []#map(int,raw_input().split())

while flg:
    try:
        k=cv2.waitKey(1)
        if chr(k&0xff)=='q': break
        prev_ts=ts
        ts,blobs=nrf.get_blobs()
        LEDs_buf = []
        color_buf = []
        blob_size_buf = []
        if ts!=prev_ts:
            img=np.zeros((480,640,3),np.uint8)
            for b in frame_blobs:
		if b[2]==3:
		  cv2.circle(img,(b[0]/2,b[1]/2),int(b[3]/math.pi**0.5/2)*2,colours[2],-1,cv2.CV_AA)
		else:
		  cv2.circle(img,(b[0]/2,b[1]/2),int(b[3]/math.pi**0.5/2)*2,colours[b[2]],-1,cv2.CV_AA)
		LEDs_buf.append([b[0]/2,b[1]/2])
		color_buf.append(b[2])
		blob_size_buf.append(int(b[3]/math.pi**0.5/2)*2)
		#print 'x, y:',b[0]/2,b[1]/2 
	    LEDs = np.array(LEDs_buf)
	    color = np.array(color_buf)
	    blob_size = np.array(blob_size_buf)
	
	    
	    if len(LEDs)>5:
	      nbrs = NearestNeighbors(n_neighbors = 4,algorithm = 'ball_tree').fit(LEDs)
	      distances,indices = nbrs.kneighbors(LEDs)
	      index_true = []
	      index_false = []
	      
	      for i in range(len(distances)):
		if indices[i][0] in index_false:
		  continue
		bad_group = []
		for j in range(1,4):
		  if distances[i][j]<6:
		    bad_group.append(indices[i][j])
		index_true+=[indices[i][0]]
		index_false+=bad_group
		
	      LEDs = LEDs[list(set(index_true))]
	      color = color[list(set(index_true))]
	      blob_size = blob_size[list(set(index_true))]
	      #print min([i[1] for i in distances])
	      nbrs = NearestNeighbors(n_neighbors = 3,algorithm = 'kd_tree').fit(LEDs)
	      distances,indices = nbrs.kneighbors(LEDs)
	      
	      valid_LEDs, color, blob_size,valid_cluster_index = CLTR.Clusters(LEDs,indices,color,blob_size,img,SHOW_NEIGHBORS)
	      #print valid_cluster_index
	      
	      centers = []
	      LEDs_per_Ebug = []
	      LEDColor_per_Ebug = []
	      BlobSize_per_Ebug = []
	      radius = []
	      visited = []
	      ID = []
	      for i in valid_cluster_index:
		if i in visited:
		  continue
		visited.append(i)
		buf_LEDs = valid_LEDs[np.where(valid_cluster_index==i)]
		circle_LEDs = CL.circle(buf_LEDs)
		center=np.array(map(int,circle_LEDs.LS()))
		bad_est = False
		if 1:#not centers:
		  centers.append(center)
		  LEDs_per_Ebug.append(buf_LEDs)
		  radius.append(int(circle_LEDs.calc_R(*center).mean()))
		  LEDColor_per_Ebug.append(color[np.where(valid_cluster_index==i)])
		  BlobSize_per_Ebug.append(blob_size[np.where(valid_cluster_index==i)])
	       
	        """
		else:
		  for j in range(len(centers)):
		    if ((centers[j]-centers)**2).sum()<10:
		      bad_est = True
		      if len(LEDs_per_Ebug[j])>=len(buf_LEDs):
			break
		      else:
			centers[j] = center
			LEDs_per_Ebug[j] = buf_LEDs
			radius[j] = int(circle_LEDs.calc_R(*center).mean())
			LEDColor_per_Ebug[j] = color[np.where(valid_cluster_index==i)]
			BlobSize_per_Ebug[j] = blob_size[np.where(valid_cluster_index==i)]
			break    
		  if bad_est == False:
		    
		    centers.append(center)
		    LEDs_per_Ebug.append(buf_LEDs)
		    radius.append(int(circle_LEDs.calc_R(*center).mean()))
		    LEDColor_per_Ebug.append(color[np.where(valid_cluster_index==i)])
		    BlobSize_per_Ebug.append(blob_size[np.where(valid_cluster_index==i)])
	        """
		      
			
		
	      for i in range(len(radius)):
		#cv2.circle(img,(centers[i][0],centers[i][1]),radius[i],[255,255,255],1)
		  #print len(LEDs_per_Ebug[i]),len(LEDColor_per_Ebug[i])
		#print LEDColor_per_Ebug[i]
		color_seq, blob_seq = CL.GetSequence(LEDs_per_Ebug[i],LEDColor_per_Ebug[i],BlobSize_per_Ebug[i],centers[i],radius[i],16)
		#print color_seq
		#print '------------------------------------------------'
		  
		EbugID = CL.EbugIdDtection(color_seq,LED_DETECTION_THRESHOLD)
		if EbugID!=-1:
		  ID.append(EbugID)
		  cv2.circle(img,(centers[i][0],centers[i][1]),radius[i],[255,255,255],1)
		  #print EbugID#, len(color_seq1)
	      print "eBug IDs are:", ID
	      #print centers  
	    
		
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
