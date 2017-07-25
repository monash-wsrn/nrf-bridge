from __future__ import with_statement
from __future__ import division
from __future__ import absolute_import
import codecs, json, re, struct, subprocess, sys, time

import serial
from io import open


class Bridge(object):
    
    def __init__(self, device=u'/dev/ttyACM0'):
        subprocess.check_call([u'stty', u'-F', device, u'raw'])
        self.usb=serial.Serial(device, timeout = 0.5)
    
    def send_packet(self,packet):
        u"""
        Sends a raw packet to the wireless interface, uses the last address
        set. It raises an error if does not receive an ack. 

        Returns the payload of the response packet.
        """
        self.usb.write('\x00' + packet)
        self.usb.flush()
        n = ord(self.usb.read(1))
        if n & 0x80:
            raise RuntimeError(self.usb.read(n & 0x3f))
        if n:
            return self.usb.read(n)
    
    def send_multicast(self, packet):
        self.usb.write('\x03' + packet)
        self.usb.flush()
        n = ord(self.usb.read(1))
        if n & 0x80:
            raise RuntimeError(self.usb.read(n&0x3f))
        if n:
            return self.usb.read(n)

    def set_TX_address(self, address):
        u"""
        Give a number or a 3-byte string.
        """
        if not isinstance(address, str):
            address = str([address]) + '\x21\xc8'
        self.usb.write('\x01' + address)
        self.usb.flush()
        n = ord(self.usb.read(1))
        if n & 0x80:
            raise RuntimeError(self.usb.read(n & 0x3f))
        if n:
            return self.usb.read(n)

    def set_RX_address(self, address):
        u"""
        Used only during the neighbour discovery. 
        """
        self.usb.write('\x02' + address)
        self.usb.flush()
        n = ord(self.usb.read(1))
        if n & 0x80:
            raise RuntimeError(self.usb.read(n & 0x3f))
        if n:
            return self.usb.read(n)

    def send_packet_check_response(self, packet):
        u""" 
        A convenience packet. 
        """
        x = self.send_packet(packet)
        if x is None:
            for i in xrange(10): #retry 10 times with dummy packet
                time.sleep(0.003)
                x = self.send_packet('\x00')
                if x:
                    break
            else:
                raise RuntimeError(u'No Response')
        if x[0] != packet[0]:
            raise RuntimeError(u'Unexpected response: %s'%repr(list(bytearray(x))))
        return x[1:]

    def send_packet_check_response_without_retry(self, packet):
        x = self.send_packet(packet)
        if x is None:
            raise RuntimeError(u'Empty Response')
        if x[0] != packet[0]:
            raise RuntimeError(u'Unexpected response: %s'%repr(list(bytearray(x))))
        return x[1:]
        
    def get_ID_type(self):
        x = self.send_packet_check_response('\x01')
        return struct.unpack(u'<BBBHBBB',x)
    
    def get_version(self):
        return self.send_packet_check_response('\x02')
    
    def get_JTAG_ID(self):
        x = self.send_packet_check_response('\x03')
        return hex(struct.unpack(u'<I', x)[0])
    
    def print_top(self, string):
        self.send_packet('\x10' + string.encode(u'ascii') + '\x00')
    
    def print_bot(self, string):
        self.send_packet('\x11' + string.encode(u'ascii') + '\x00')
    
    def LCD_backlight(self, enable):
        self.send_packet('\x12' + str([enable]))
        
    def LCD_contrast(self, contrast): #set contrast (range is 0--100)
        self.send_packet('\x13' + str([contrast]))
    
    def motor_control(self, speed_L, speed_R, decay_mode_L, decay_mode_R=None):
        if decay_mode_R is None:
            decay_mode_R = decay_mode_L
        dir_L = speed_L < 0
        dir_R = speed_R < 0
        motor_mode = dir_L | (decay_mode_L << 1) | (dir_R << 2) | (decay_mode_R << 3)
        self.send_packet('\x20' + struct.pack(u'<HHB', abs(speed_L), abs(speed_R), motor_mode))
    
    def set_boost_voltage(self, voltage, enable=True):
        if voltage < 4.5 or voltage > 18:
            raise RuntimeError(u'Voltage should be between 4.5 and 18')
        current = (voltage - 12) / 0.08725
        current = int(current)
        self.send_packet('\x21' + struct.pack(u'<BB', (current > 0) << 1 | enable, abs(current)))
    
    def get_motor_turn_counts(self, reset=False):
        u"""
        Gives us two signed 16-bit integers. 
        """
        x = self.send_packet_check_response('\x22' if reset else '\x23')
        return struct.unpack(u'<hh', x)
    
    def calibrate_hall_sensors(self, hysteresis=20): #hysteresis in steps of 1.25mV -- default is +/-25mV
        u"""
        Do this once (results stored in EEPROM). Call it with the magnet holders removed.
        Decrease hysteresis for more sensitivity, increase for more noise immunity.
        """
        self.send_packet('\x24' + str([hysteresis]))
    
    def enable_motor_controller(self):
        self.send_packet('\x25\x01')
    
    def disable_motor_controller(self):
        self.send_packet('\x25\x00')
    
    def set_motor_controller_PID(self, P_l, I_l, D_l, P_r, I_r, D_r):
        self.send_packet('\x25' + struct.pack(u'<hhhhhh', P_l ,I_l, D_l, P_r, I_r, D_r))
    
    def set_motor_controller_target(self, L, R):
        self.send_packet('\x26' + struct.pack(u'<hh', L, R))
    
    def get_motor_speed(self):
        x = self.send_packet_check_response('\x27')
        return struct.unpack(u'<hh', x)
    
    def enable_LEDs(self, side, top, centre_master, centre_slave):
        self.send_packet(
            '\x30' + str([side | (top << 1) | (centre_master << 2) | (centre_slave << 3)])
        )
    
    def set_LEDs(self, red, green, blue):
        self.send_packet('\x31' + struct.pack(u'<HHH',int(red, 16), int(green, 16), int(blue, 16)))
    
    def LED_brightness(self, brightness):
        self.send_packet('\x32' + str([brightness]))
    
    def get_bump_sensors(self):
        u"""
        Gives us 6 boolean values.
        """
        x = self.send_packet_check_response('\x40')
        x = ord(x)
        return tuple(bool(x & (1 << i)) for i in xrange(6))
    
    def get_touch_buttons(self):
        u"""
        Gives us 2 boolean values.
        """
        x = self.send_packet_check_response('\x41')
        x = ord(x)
        return tuple(bool(x & (1 << i)) for i in xrange(2))
        
    def get_light_sensors(self):
        u"""
        Gives us 16 12-bit unsigned values. 
        """
        x = self.send_packet_check_response('\x50')
        LS = []
        for i in xrange(8):
            a = bytearray(x[i * 3:(i + 1) * 3])
            LS.append(a[0] | (a[1] & 0xf) << 8)
            LS.append(a[1] >> 4 | a[2] << 4)
        return LS
    
    def get_power_values(self):
        u"""
        Gives us battery voltage level (mV), current draw (mA), 
        charge current (mA, current flowing in through the USB connection),
        and temperature (C).
        """
        x = self.send_packet_check_response('\x60')
        return struct.unpack(u'<iiih', x)
    
    def calibrate_power_ADC(self):
        u"""
        Do this once. Call it when powered through the USB connection and 
        battery taken out. 
        """
        self.send_packet('\x61')
    
    def increase_charge_current(self, current):
        if current > 0.8:
            raise RuntimeError(u'Additional charge current should be at most 0.8A')
        self.send_packet('\x62' + str([int(current / 8 * 1000)]))
    
    def forget_unicast_address(self):
        u"""
        Mostly used by address assignment.
        """
        self.send_packet('\xb3')
    
    def reset(self, bootloader=False):
        u"""
        reset or reset to bootloader.
        """
        self.send_packet('\xff' if bootloader else '\xfe')

    def flash(self, filename, which=None):
        u"""
        Flash a new firmware. eBug needs to be in the bootloader mode.
        """
        with open(filename) as f:
            h = f.readlines() #read in .cyacd file
        
        if which == u'Master' or (which is None and u'Master' in filename):
            print u'Programming master'
            self.set_TX_address('\x19\x8a\xaf')
        elif which == u'Slave' or (which is None and u'Slave' in filename):
            print u'Programming slave'
            self.set_TX_address('\x1a\x8a\xaf')
        else:
            raise RuntimeError(u'Couldn\'t determine whether to program master or slave')
        
        welcome = struct.pack(u'<HHHI', 0xff00, int(h[1][1:7], 16), len(h)-2, int(h[0][:8], 16)) + '\x00' * 22
        self.send_packet(welcome) #welcome packet
        time.sleep(0.003)
        
        for n,line in enumerate(h[1:-1]): #first line is header; last line is metadata (which we ignore)
            array = int(line[1:3], 16) #array number
            row = int(line[3:7], 16)   #row number
            data = [str([int(line[11 + 2 * i:13 + 2 * i], 16)]) for i in xrange(256)] #256 data bytes for each row
            data = ''.join(data)
            data += '\x00' * 14 #add 14 bytes to make 9 packets of 30 bytes
            
            for i in xrange(9):
                self.send_packet(str([row]) + str([array + (i << 2)]) + data[i * 30:(i + 1) * 30]) #header contains row, array, and seq numbers
                time.sleep(0.003) #sometimes delay is needed, sometimes not
            print u'%d/%d' % (n + 1, len(h) - 2), u'\r', sys.stdout.flush()
        print
        self.send_packet('\xff' * 32) #reset to loaded app
        time.sleep(0.3)
    
    def neighbour_discovery(self, index=0, only_new=False):
        u"""
        Usually we don't use this function directly. We use "assign_addresses()"
        instead. 
        """
        self.set_TX_address(0xff)
        self.set_RX_address('\x00\x21\xc8')
        self.usb.write(('\xb0' if only_new else '\xb1') + str([index]))
        self.usb.flush()
        n = ord(self.usb.read(1))
        if n & 0x80:
            raise RuntimeError(self.usb.read(n&0x3f))
        if n != 2:
            raise RuntimeError(u'Unexpected response from bridge')
        num_responses = struct.unpack(u'<H',self.usb.read(2))[0]
        responses = []
        for i in xrange(num_responses):
            n = ord(self.usb.read(1))
            if n & 0x80:
                raise RuntimeError(self.usb.read(n&0x3f))
            if n != 8:
                raise RuntimeError(u'Unexpected response from bridge')
            r = self.usb.read(8)
            if str([r[0]]) != ('\xb0' if only_new else '\xb1'):
                raise RuntimeError(u'Unexpected response from bridge')
            responses.append(struct.unpack(u'<BBBHBB', r[1:]))
        return responses
    
    def set_unicast_address(self, serial, address):
        u"""
        We don't use this directly. 
        """
        self.set_TX_address(0xff)
        if type(serial) is not unicode: serial = struct.pack(u'<BBBHBB',*serial)
        if type(address) is not unicode: address = str([address])
        self.send_multicast('\xb2' + serial + address)
    
    def forget_unicast_address(self, everyone=True):
        if everyone:
            self.set_TX_address(0xff)
            self.send_multicast('\xb3')
        else:
            self.send_packet('\xb3')

    def load_device_info_from_file(self, file):
        with open(file,u'r') as saved_device_info_file:
            device_pairing_list = json.load(saved_device_info_file)

        device_pairing_list = dict((int(key), value) for key, value in device_pairing_list.items())
        for device in device_pairing_list:
            if device_pairing_list[device].get(u'led_sequence'):
                device_pairing_list[device][u'led_sequence'] = [codecs.encode(value) for value in device_pairing_list[device][u'led_sequence']]
            device_pairing_list[device][u'psoc_id'] = tuple(device_pairing_list[device][u'psoc_id'])
        return device_pairing_list

    def assign_static_addresses(self, path = u'eBugs_pairing_list.json'):
        u'''
        consults a table to always assign the same 1-byte address and RGB led sequence to the same eBug
        returns the information on every conected devices : cameras, eBugs and unknown devices
        Ex : camera, eBug, unknown = nrf.assign_static_addresses('../nrf-bridge/eBugs_pairing_list.json')
        '''
        self.unknown = set()
        self.camera = dict()
        self.eBug = dict()

        eBugs_pairing_list = self.load_device_info_from_file(path)

        eBugs_psoc_id_list = dict((value[u'psoc_id'], key) for key, value in eBugs_pairing_list.items())

        self.forget_unicast_address() #everyone should forget their current addresses
        for j in xrange(3): #repeat a few times in case of collisions
            for i in xrange(7):
                neighbours = self.neighbour_discovery(i,True) #find all neighbours that haven't been assigned an address
                for x in neighbours:
                    if x in eBugs_psoc_id_list:
                        for t in xrange(10):
                            try:
                                address = eBugs_psoc_id_list[x]
                                self.set_unicast_address(x, address)
                                self.set_TX_address(address)
                                self.send_packet('\x00')
                                if(eBugs_pairing_list[address][u'type'] == 1):
                                    self.camera[address] = eBugs_pairing_list[address]
                                else:
                                    self.eBug[address] = eBugs_pairing_list[address]
                                break
                            except:
                                pass
                    else:
                        self.unknown.add(x)

        self.display_devices()
        return self.camera, self.eBug, self.unknown

    def assign_addresses(self):
        self.forget_unicast_address() #everyone should forget their current addresses
        #TODO this is not ACKed, so some may still have an address assigned
        #TODO instead, we can send a forget_unicast_address to each of the 254 possible addresses (takes about 8ms for each address if no one is listening)
        #TODO alternative: use a `session ID' and include in ND request and the set address request. Nodes set their own session ID when setting their address and only respond to ND requests if session ID differs.
        devices={}
        n=0
        for j in xrange(3): #repeat a few times in case of collisions
            for i in xrange(7):
                neighbours=self.neighbour_discovery(i,True) #find all neighbours that haven't been assigned an address
                for x in neighbours:
                    n+=1
                    for t in xrange(10):
                        try:
                            self.set_unicast_address(x,n)
                            self.set_TX_address(n)
                            self.send_packet('\x00')
                            devices[n]=x
                            break
                        except: pass
                    else: n-=1
        return devices

    def display_devices(self):
        print u'Addr\tPSoC ID             \tType       \tLED Sequence'
        print u'----\t--------------------\t-----------\t-------------------------------'
        for addr, info in self.camera.items():
            print addr,
                u'\t',
                u'-'.join([unicode(element) for element in info[u'psoc_id']]),
                u'\t',
                u'camera (0)'
        for addr, info in self.eBug.items():
            print addr,
                u'\t',
                u'-'.join([unicode(element) for element in info[u'psoc_id']]),
                u'\t',
                u'eBug (1)',
                u'\t',
                unicode(info[u'led_sequence'])
        for psoc_id in self.unknown:
            print u'\t',
                u'-'.join([unicode(element) for element in psoc_id]),
                u'\t',
                u'UNKNOWN'

    def flash_all_ebugs(self, filename, which=None):
        u"""
        Used by flash.py
        """
        for i,j in self.assign_addresses().items():
            self.set_TX_address(i)
            if self.get_ID_type()[6] == 0: #only flash eBugs (not cameras)
                print u'Flashing eBug with ID ', j
                self.reset(True)
                time.sleep(0.3)
                self.flash(filename,which)
    
    def get_blobs(self):
        u"""
        We keep calling this. Look at camera.py for usage.
        
        Camera firmware processes a frame, records the blobs into points[256]
        array. Each element of the array is a "blob". 

        Each blob is packed into 32-bits  - x, y, color, size 
        size: square root of the blob.
        double resolution (half pixels)

        We get up to 6 blobs per packet. 

        Timestamps are in miliseconds since the start of the
        program (32 bits long)

        The points[] array only gets updated when all the blobs have
        been read out. If you don't request all of them in time, then
        the next frame will be discarded. This way you only get a
        complete frame's worth of blobs at a time. Each packet is
        timestamped with the frame it corresponds to so you can tell
        when you've missed a frame.
        """
        x = self.send_packet_check_response_without_retry('\x90')
        n = len(x)//4
        z = struct.unpack('<' + 'I' * n,x)
        unpack = lambda i: tuple(i >> offset & (1 << length) - 1 for offset,length in [(0, 11), (11, 11), (22, 2), (24, 8)])
        return z[0], [unpack(i) for i in z[1:]]
    
    def set_camera_thresholds(self, thresholds):
        u"""
        When we adjust the sliders, it sends this. 
        """
        self.send_packet('\x93' + struct.pack(u'<' + u'B' * 8, *thresholds))

    def set_camera_exposure(self, n):
        self.camera_write_reg(4,n&3)
        self.camera_write_reg(0x10,(n>>2)&0xff)
        self.camera_write_reg(0xa1,(n>>10)&0x3f)
    
    def set_camera_gain(self, n):
        self.camera_write_reg(0, n)
    
    def set_camera_blue_gain(self, n):
        self.camera_write_reg(1, n)
    
    def set_camera_red_gain(self, n):
        self.camera_write_reg(2, n)
    
    def camera_write_reg(self, reg, value):
        self.send_packet('\x91' + struct.pack(u'<BB',reg,value))
        
    def get_laser_event(self):
        x = self.send_packet_check_response('\x80')
        if x:
            return struct.unpack(u'<HHBB', x)

    def print_laser_event(self):
        e = self.get_laser_event()
        if e:
            pos,ebug_id,length,sensor_id = e
            print u'[%u] %3u: %5u - %5u (%u)' % (sensor_id, length, (pos - length) & 0xffff, pos, ebug_id)
    
    def laser_motor_enable(self, enable_laser, enable_motor):
        self.send_packet('\x81' + str([enable_laser + (enable_motor << 1)]))
    
    def set_laser_id(self, id_no):
        self.send_packet('\x82' + struct.pack(u'<H', id_no))
    
    def set_laser_motor_speed(self, speed):
        self.send_packet('\x83' + struct.pack(u'<B', speed))
    
    def test_laser(self, clk_divider):
        self.send_packet('\x84' + struct.pack(u'<B', clk_divider))
    
    def get_laser_motor_feedback(self):
        x = self.send_packet_check_response('\x90')
        if x:
            return struct.unpack(u'<' + u'I' * (len(x) / 4), x)
