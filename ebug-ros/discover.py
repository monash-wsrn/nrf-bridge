#!/usr/bin/python

import nrf

nrf=nrf.bridge()
ebugs=nrf.assign_addresses()

for i,j in ebugs.items():
    print i,j
    nrf.set_TX_address(i)
    nrf.print_bot(repr(j))
