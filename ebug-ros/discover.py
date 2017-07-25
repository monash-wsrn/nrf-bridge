#!/usr/bin/python

from __future__ import absolute_import
import settings
from libraries.nrf import Bridge

nrf=Bridge()
camera, eBugs, unknown = nrf.assign_static_addresses(path=u'../libraries/eBugs_pairing_list.json')

for addr, info in eBugs.items():
    nrf.set_TX_address(addr)
    nrf.print_top(u'-'.join(unicode(element) for element in info[u'psoc_id']))
