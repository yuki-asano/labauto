#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial


class SerialInterface():
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE, timeout=1.0):
        self.port =  port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout

        self.ser = None
        self.is_open = False


    def __del__(self):
        self.close()


    def open(self):
        self.ser = serial.Serial(self.port, self.baudrate, self.bytesize, self.parity, self.stopbits, self.timeout)
        self.is_open = True


    def close(self):
        if(self.ser):
            self.ser.close()
