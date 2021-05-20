import platform
import serial
import serial.tools.list_ports
import time
import numpy as np

from control._def import *

# add user to the dialout group to avoid the need to use sudo

class Microcontroller():
    def __init__(self,parent=None):
        self.serial = None
        self.platform_name = platform.system()
        self.tx_buffer_length = MCU.CMD_LENGTH
        self.rx_buffer_length = MCU.MSG_LENGTH

        arduino_ports = [
                p.device
                for p in serial.tools.list_ports.comports()
                if 'RaspberryPi Pico' in p.description]
        if not arduino_ports:
            raise IOError("No RaspberryPi Pico found")
        if len(arduino_ports) > 1:
            print('Multiple RaspberryPi Pico found - using the first')
        else:
            print('Using RaspberryPi Pico found at : {}'.format(arduino_ports[0]))

        # establish serial communication
        self.serial = serial.Serial(arduino_ports[0],2000000)
        time.sleep(0.2)
        print('Serial Connection Open')

    def close(self):
        self.serial.close()

    def set_threshold_start(self,value):
        value = ((value-_comm_pressure_min)/(_comm_pressure_max-_comm_pressure_min))*65535
        cmd = bytearray(3)
        cmd[0] = 0
        cmd[1] = int((value)*65535) >> 8
        cmd[2] = int((value)*65535) & 0xff
        self.serial.write(cmd)

    def set_threshold_stop(self,value):
        value = ((value-_comm_pressure_min)/(_comm_pressure_max-_comm_pressure_min))*65535
        cmd = bytearray(3)
        cmd[0] = CMD_SET.SET_ILLUMINATION
        cmd[1] = int((value)*65535) >> 8
        cmd[2] = int((value)*65535) & 0xff
        self.serial.write(cmd)

    def read_received_packet(self):
        # wait to receive data
        while self.serial.in_waiting==0:
            pass
        while self.serial.in_waiting % self.rx_buffer_length != 0:
            pass

        num_bytes_in_rx_buffer = self.serial.in_waiting

        # get rid of old data
        if num_bytes_in_rx_buffer > self.rx_buffer_length:
            print('getting rid of old data')
            for i in range(num_bytes_in_rx_buffer-self.rx_buffer_length):
                self.serial.read()
        
        # read the buffer
        data=[]
        for i in range(self.rx_buffer_length):
            data.append(ord(self.serial.read()))

        return data

    def read_received_packet_nowait(self):
        # wait to receive data
        if self.serial.in_waiting==0:
            return None
        if self.serial.in_waiting % self.rx_buffer_length != 0:
            print(self.serial.in_waiting)
            # self.serial.reset_input_buffer()
            num_bytes_in_rx_buffer = self.serial.in_waiting
            for i in range(num_bytes_in_rx_buffer):
                self.serial.read()
            print('reset input buffer')
            return None
        
        # get rid of old data
        num_bytes_in_rx_buffer = self.serial.in_waiting
        if num_bytes_in_rx_buffer > self.rx_buffer_length:
            print('getting rid of old data')
            for i in range(num_bytes_in_rx_buffer-self.rx_buffer_length):
                self.serial.read()
        
        # read the buffer
        data=[]
        for i in range(self.rx_buffer_length):
            data.append(ord(self.serial.read()))
        return data

class Microcontroller_Simulation():
    def __init__(self,parent=None):
        self.tx_buffer_length = MCU.CMD_LENGTH
        self.rx_buffer_length = MCU.MSG_LENGTH

    def close(self):
        pass

    def read_received_packet(self):
        pass

    def read_received_packet_nowait(self):
        return None