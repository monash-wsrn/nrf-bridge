Python Scripts:

1. "camera.py" : ????
2. "camera-test.py": wakes up the camera
4. "circle.py": used in "camera-test.py" for circle and ebug ID detection.
3. "clustering": used in "camera-test.py" for separate LEDs to different clusters (NN).
3. "discover.py: ????
1. "ebug-test.py": sets LED sequence for ebugs.
5. "flash.py": ????
6. "nrf.py": ????

Firmware:

nrf-bridge.cydsn
nrf-bridge.cywrk


How to use:

First run ebug-test.py, then run camera-test.py. 

Notes:

* In camera-test.py, SHOW_NEIGHBORS is a global variable. If you set
it to "True", the neighbors of each LED will be connected by a line
shown in the video.

* Since the LEDs flash in the bottom of each ebug (we dicussed about
it previously at Monash), if I only recognize an LED sequence when all
the 16 LEDs on an ebug match the dictionary, no LED sequence would be
recognized. I use the voting approach (we discussed about this as
well).  This method can detect LED sequences, but it also introduces
some faults in detection.

* The global variable, 'LED_DETECTION_THRESHOLD', defines the number
of LEDs on each ebug to be matched.

* I only draw a circle on ebug when its LED sequence is recognized.

