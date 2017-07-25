#!/usr/bin/python
import nrf
import sys

if 'all' in sys.argv:
    flash_all=True
    sys.argv.remove('all')
else: flash_all=False

if len(sys.argv)<2:
    raise RuntimeError('usage: %s master|slave|<filename> [all] [<serial port>]'%sys.argv[0])

if len(sys.argv)<3: nrf=nrf.bridge()
else: nrf=nrf.bridge(sys.argv[2])

master='../ebug2014-firmware/eBug2014 Master.cydsn/CortexM3/ARM_GCC_493/Release/eBug2014 Master.cyacd'
slave='../ebug2014-firmware/eBug2014 Slave.cydsn/CortexM3/ARM_GCC_493/Release/eBug2014 Slave.cyacd'

flash_func=nrf.flash_all_ebugs if flash_all else nrf.flash

if sys.argv[1]=='master': flash_func(master)
elif sys.argv[1]=='slave': flash_func(slave)
else: flash_func(sys.argv[1])
