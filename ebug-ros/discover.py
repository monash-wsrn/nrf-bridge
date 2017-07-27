#!/usr/bin/python

from nrf import Bridge

nrf = Bridge()
cameras, ebugs, unknowns = nrf.assign_static_addresses('ebug_tab.json')

# Light up the LED sequences


