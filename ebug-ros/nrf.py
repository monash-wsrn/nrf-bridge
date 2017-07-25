import serial
import subprocess
import struct
import time
import sys

class bridge:
    def __init__(self,device='/dev/ttyACM0'):
        subprocess.check_call(['stty','-F',device,'raw'])
        self.usb=serial.Serial(device,timeout=0.5)
    
    def send_packet(self,packet):
        self.usb.write('\x00'+packet)
        self.usb.flush()
        n=ord(self.usb.read(1))
        if n&0x80: raise RuntimeError(self.usb.read(n&0x3f))
        return self.usb.read(n) if n else None
    
    def send_multicast(self,packet):
        self.usb.write('\x03'+packet)
        self.usb.flush()
        n=ord(self.usb.read(1))
        if n&0x80: raise RuntimeError(self.usb.read(n&0x3f))
        return self.usb.read(n) if n else None
    
    def set_TX_address(self,address):
        if type(address) is not str: address=chr(address)+'\x21\xc8'
        self.usb.write('\x01'+address)
        self.usb.flush()
        n=ord(self.usb.read(1))
        if n&0x80: raise RuntimeError(self.usb.read(n&0x3f))
        return self.usb.read(n) if n else None
    
    def set_RX_address(self,address):
        self.usb.write('\x02'+address)
        self.usb.flush()
        n=ord(self.usb.read(1))
        if n&0x80: raise RuntimeError(self.usb.read(n&0x3f))
        return self.usb.read(n) if n else None
        
    def send_packet_check_response(self,packet):
        x=self.send_packet(packet)
        if x is None:
            for i in range(10): #retry 10 times with dummy packet
                time.sleep(0.003)
                x=self.send_packet('\x00')
                if x is not None: break
            else: raise RuntimeError('No response')
        if x[0]!=packet[0]: raise RuntimeError('Unexpected response: %s'%repr(list(bytearray(x))))
        return x[1:]
        
    def get_ID_type(self):
        x=self.send_packet_check_response('\x01')
        return struct.unpack('<BBBHBBB',x)
    
    def get_version(self):
        return self.send_packet_check_response('\x02')
    
    def get_JTAG_ID(self):
        x=self.send_packet_check_response('\x03')
        return hex(struct.unpack('<I',x)[0])
    
    def print_top(self,string):
        self.send_packet('\x10'+string+'\x00')
    
    def print_bot(self,string):
        self.send_packet('\x11'+string+'\x00')
    
    def LCD_backlight(self,enable):
        self.send_packet('\x12'+chr(enable))
    
    def motor_control(self,speed_L,speed_R,decay_mode_L,decay_mode_R=None):
        if decay_mode_R is None: decay_mode_R=decay_mode_L
        dir_L=speed_L<0
        dir_R=speed_R<0
        motor_mode=dir_L|(decay_mode_L<<1)|(dir_R<<2)|(decay_mode_R<<3)
        self.send_packet('\x20'+struct.pack('<HHB',abs(speed_L),abs(speed_R),motor_mode))
    
    def set_boost_voltage(self,voltage,enable=True):
        if voltage<4.5 or voltage>18: raise RuntimeError('Voltage should be between 4.5 and 18')
        current=(voltage-12)/0.08725
        current=int(current)
        self.send_packet('\x21'+struct.pack('<BB',(current>0)<<1|enable,abs(current)))
    
    def get_motor_turn_counts(self,reset=False):
        x=self.send_packet_check_response('\x22' if reset else '\x23')
        return struct.unpack('<hh',x)
    
    def calibrate_hall_sensors(self):
        self.send_packet('\x24')
    
    def enable_motor_controller(self):
        self.send_packet('\x25\x01')
    
    def disable_motor_controller(self):
        self.send_packet('\x25\x00')
    
    def set_motor_controller_PID(self,P_l,I_l,D_l,P_r,I_r,D_r):
        self.send_packet('\x25'+struct.pack('<hhhhhh',P_l,I_l,D_l,P_r,I_r,D_r))
    
    def set_motor_controller_target(self,L,R):
        self.send_packet('\x26'+struct.pack('<hh',L,R))
    
    def get_motor_speed(self):
        x=self.send_packet_check_response('\x27')
        return struct.unpack('<hh',x)
    
    def enable_LEDs(self,side,top,centre_master,centre_slave):
        self.send_packet('\x30'+chr(side|(top<<1)|(centre_master<<2)|(centre_slave<<3)))
    
    def set_LEDs(self,red,green,blue):
        self.send_packet('\x31'+struct.pack('<HHH',red,green,blue))
    
    def LED_brightness(self,brightness):
        self.send_packet('\x32'+chr(brightness))
    
    def get_bump_sensors(self):
        x=self.send_packet_check_response('\x40')
        x=ord(x)
        return tuple(bool(x&(1<<i)) for i in range(6))
    
    def get_touch_buttons(self):
        x=self.send_packet_check_response('\x41')
        x=ord(x)
        return tuple(bool(x&(1<<i)) for i in range(2))
        
    def get_light_sensors(self):
        x=self.send_packet_check_response('\x50')
        LS=[]
        for i in range(8):
            a=bytearray(x[i*3:(i+1)*3])
            LS.append(a[0]|(a[1]&0xf)<<8)
            LS.append(a[1]>>4|a[2]<<4)
        return LS
    
    def get_power_values(self):
        x=self.send_packet_check_response('\x60')
        return struct.unpack('<iiih',x)
    
    def calibrate_power_ADC(self):
        self.send_packet('\x61')
    
    def increase_charge_current(self,current):
        if current>0.8: raise RuntimeError('Additional charge current should be at most 0.8A')
        self.send_packet('\x62'+chr(int(current/8*1000)))
    
    def forget_unicast_address(self):
        self.send_packet('\xb3')
    
    def reset(self,bootloader=False):
        self.send_packet('\xff' if bootloader else '\xfe')
    
    def flash(self,filename,which=None):
        with open(filename) as f: h=f.readlines() #read in .cyacd file
        
        if which=='Master' or (which is None and 'Master' in filename):
            print 'Programming master'
            self.set_TX_address('\x19\x8a\xaf')
        elif which=='Slave' or (which is None and 'Slave' in filename):
            print 'Programming slave'
            self.set_TX_address('\x1a\x8a\xaf')
        else: raise RuntimeError('Couldn\'t determine whether to program master or slave')
        
        welcome=struct.pack('<HHHI',0xff00,int(h[1][1:7],16),len(h)-2,int(h[0][:8],16))+'\x00'*22
        self.send_packet(welcome) #welcome packet
        time.sleep(0.003)
        
        for n,line in enumerate(h[1:-1]): #first line is header; last line is metadata (which we ignore)
            array=int(line[1:3],16) #array number
            row=int(line[3:7],16)   #row number
            data=[chr(int(line[11+2*i:13+2*i],16)) for i in range(256)] #256 data bytes for each row
            data=''.join(data)
            data+='\0'*14 #add 14 bytes to make 9 packets of 30 bytes
            
            for i in range(9):
                self.send_packet(chr(row)+chr(array+(i<<2))+data[i*30:(i+1)*30]) #header contains row, array, and seq numbers
                time.sleep(0.003) #sometimes delay is needed, sometimes not
            print '%d/%d'%(n+1,len(h)-2),'\r',
            sys.stdout.flush()
        print
        self.send_packet('\xff'*32) #reset to loaded app
        time.sleep(0.3)
    
    def neighbour_discovery(self,index=0,only_new=False):
        self.set_TX_address(0xff)
        self.set_RX_address('\x00\x21\xc8')
        self.usb.write(('\xb0' if only_new else '\xb1')+chr(index))
        self.usb.flush()
        n=ord(self.usb.read(1))
        if n&0x80: raise RuntimeError(self.usb.read(n&0x3f))
        if n!=2: raise RuntimeError('Unexpected response from bridge')
        num_responses=struct.unpack('<H',self.usb.read(2))[0]
        responses=[]
        for i in range(num_responses):
            n=ord(self.usb.read(1))
            if n&0x80: raise RuntimeError(self.usb.read(n&0x3f))
            if n!=8: raise RuntimeError('Unexpected response from bridge')
            r=self.usb.read(8)
            if r[0]!=('\xb0' if only_new else '\xb1'): raise RuntimeError('Unexpected response from bridge')
            responses.append(struct.unpack('<BBBHBB',r[1:]))
        return responses
    
    def set_unicast_address(self,serial,address):
        self.set_TX_address(0xff)
        if type(serial) is not str: serial=struct.pack('<BBBHBB',*serial)
        if type(address) is not str: address=chr(address)
        self.send_multicast('\xb2'+serial+address)
    
    def forget_unicast_address(self,everyone=True):
        if everyone:
            self.set_TX_address(0xff)
            self.send_multicast('\xb3')
        else: self.send_packet('\xb3')
    
    def assign_addresses(self):
        self.forget_unicast_address() #everyone should forget their current addresses
        #TODO this is not ACKed, so some may still have an address assigned
        #TODO instead, we can send a forget_unicast_address to each of the 254 possible addresses (takes about 8ms for each address if no one is listening)
        #TODO alternative: use a `session ID' and include in ND request and the set address request. Nodes set their own session ID when setting their address and only respond to ND requests if session ID differs.
        devices={}
        n=0
        for j in range(3): #repeat a few times in case of collisions
            for i in range(7):
                neighbours=self.neighbour_discovery(i,True) #find all neighbours that haven't been assigned an address
                for x in neighbours:
                    n+=1
                    for t in range(10):
                        try:
                            self.set_unicast_address(x,n)
                            self.set_TX_address(n)
                            self.send_packet('\x00')
                            devices[n]=x
                            break
                        except: pass
                    else: n-=1
        self.set_TX_address(n)
        return devices

    def flash_all_ebugs(self,filename,which=None):
        for i,j in self.assign_addresses().items():
            self.set_TX_address(i)
            if self.get_ID_type()[6]==0: #only flash eBugs (not cameras)
                print 'Flashing eBug with ID '+repr(j)
                self.reset(True)
                time.sleep(0.3)
                self.flash(filename,which)
    
    def get_blobs(self):
        x=self.send_packet_check_response('\x90')
        n=len(x)/4
        z=struct.unpack('<'+'I'*n,x)
        unpack=lambda i: tuple(i>>offset & (1<<length)-1 for offset,length in [(0,11),(11,11),(22,2),(24,8)])
        return z[0],[unpack(i) for i in z[1:]]
    
    def set_camera_thresholds(self,thresholds):
        self.send_packet('\x93'+struct.pack('<'+'B'*8,*thresholds))
    
    def camera_write_reg(self,reg,value):
	self.send_packet('\x91'+struct.pack('<BB',reg,value))
        
    def get_laser_event(self):
        x=self.send_packet_check_response('\x80')
        return struct.unpack('<HHBB',x) if x else None
        
    def print_laser_event(self):
        e=self.get_laser_event()
        if e:
            pos,ebug_id,length,sensor_id=e
            print '[%u] %3u: %5u - %5u (%u)'%(sensor_id,length,(pos-length)&0xffff,pos,ebug_id)
    
    def laser_motor_enable(self,enable_laser,enable_motor):
        self.send_packet('\x81'+chr(enable_laser+(enable_motor<<1)))
    
    def set_laser_id(self,id_no):
        self.send_packet('\x82'+struct.pack('<H',id_no))
    
    def set_laser_motor_speed(self,speed):
        self.send_packet('\x83'+struct.pack('<B',speed))
    
    def test_laser(self,clk_divider):
        self.send_packet('\x84'+struct.pack('<B',clk_divider))
    
    def get_laser_motor_feedback(self):
        x=self.send_packet_check_response('\x90')
        return struct.unpack('<'+'I'*(len(x)/4),x) if x else None
