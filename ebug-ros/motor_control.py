#!/usr/bin/python
import sys
from PyQt4.QtGui import *
from PyQt4.QtCore import QTimer

app=QApplication(sys.argv)
win=QWidget()
win.setWindowTitle('Motor contoller')

pos_l=QLabel('0')
pos_r=QLabel('0')
speed_l=QLabel('0')
speed_r=QLabel('0')
target_l=QSpinBox()
target_r=QSpinBox()
P_l=QSpinBox()
P_r=QSpinBox()
I_l=QSpinBox()
I_r=QSpinBox()
D_l=QSpinBox()
D_r=QSpinBox()

for i in [target_l,target_r,P_l,P_r,I_l,I_r,D_l,D_r]: i.setRange(-32768,32767)

def target_change():
    n.set_motor_controller_target(target_l.value(),target_r.value())
for i in [target_l,target_r]: i.valueChanged.connect(target_change)

def PID_change():
    n.set_motor_controller_PID(*(i.value() for i in [P_l,I_l,D_l,P_r,I_r,D_r]))
for i in [P_l,P_r,I_l,I_r,D_l,D_r]: i.valueChanged.connect(PID_change)


layout=QGridLayout(win)
layout.addWidget(QLabel('Left'),0,1)
layout.addWidget(QLabel('Right'),0,2)
layout.addWidget(QLabel('Position'),1,0)
layout.addWidget(QLabel('Speed'),2,0)
layout.addWidget(QLabel('Target'),3,0)
layout.addWidget(QLabel('P'),4,0)
layout.addWidget(QLabel('I'),5,0)
layout.addWidget(QLabel('D'),6,0)
layout.addWidget(pos_l,1,1)
layout.addWidget(pos_r,1,2)
layout.addWidget(speed_l,2,1)
layout.addWidget(speed_r,2,2)
layout.addWidget(target_l,3,1)
layout.addWidget(target_r,3,2)
layout.addWidget(P_l,4,1)
layout.addWidget(P_r,4,2)
layout.addWidget(I_l,5,1)
layout.addWidget(I_r,5,2)
layout.addWidget(D_l,6,1)
layout.addWidget(D_r,6,2)


def update():
    l,r=n.get_motor_turn_counts()
    pos_l.setText(str(l))
    pos_r.setText(str(r))
    l,r=n.get_motor_speed() #eBug actually measures period, not speed
    l=(1<<16)/l #this is the value the controller uses
    r=(1<<16)/r
    speed_l.setText(str(l))
    speed_r.setText(str(r))

timer=QTimer()
timer.timeout.connect(update)
timer.start(10)

import nrf
n=nrf.bridge()
if n.assign_addresses():
    win.show()
    sys.exit(app.exec_())
else: print 'No eBugs found'
