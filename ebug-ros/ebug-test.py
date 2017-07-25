import nrf
import numpy as np
import cv2
import math
import time
from random import randint

LED_list = [[0x1a02,0x2091,0xc56c],
	    [0xba22,0x4495,0x148],
	    [0x4381,0x3c02,0x807c],#sometimes mess up with original #10 an #11
	    [0xa914,0x100a,0x46e1],
	    [0x440,0xd0a3,0x2b1c],
	    #[0x9061,0x4c16,0x2388],# not good
	    [0x6483,0x8308,0x1874],
	    #[0x14e1,0x600a,0x8b14],# bad
	    #[0x304c,0x8313,0x4ca0],# not good
	    [0x9010,0x6a07,0x5e8],# not good
	    #[0x9f16,0x4080,0x2069],# not good
	    #[0xd122,0x2e59,0x84],# not good
	    #[0xae00,0x41b3,0x104c],# not good
	    #[0x640,0xe08a,0x1935],
	    [0x83d9,0x3406,0x4820]]


n=nrf.bridge('/dev/ttyACM1')#'/dev/ttyACM0'
a=n.assign_addresses()
ID = 0
for i in a.keys():
    n.set_TX_address(i)
    n.LCD_backlight(0)
    n.enable_LEDs(0,1,0,0)
    n.LED_brightness(20)#40
    LED_set = LED_list[ID]#randint(0,14)]
    n.set_LEDs(*LED_set)
    ID+=1
    #if n.get_ID_type()[6]==0: #find first ebug in neighbours
    #    camera_address=i
    
    #    break
else: raise RuntimeError('No ebugs found')



    
n.LCD_backlight(0)
n.enable_LEDs(0,1,0,0)
n.LED_brightness(20)#40
LED_set = LED_list[0]#randint(0,14)]
n.set_LEDs(*LED_set)
#n.set_LEDs(0xffff,0,0xc4d7) #No.1 seq 4 colors
#n.set_LEDs(0x1a02,0x2091,0xc56c) #No.1
#n.set_LEDs(0x0640,0xe08a,0x1935) #No.13
#n.set_LEDs(0x0800,0x0,0x0)