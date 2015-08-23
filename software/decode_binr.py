#!/usr/bin/python

from __future__ import division

import sys
import struct

DLE = 0x10
ETX = 0x03
CRC = 0xFF

prom = None

def handle_packet(id_, payload):
    global prom
    if id_ == 0:
        if ord(payload[0]) == 1: # barometer prom
            prom = struct.unpack('>8H', payload[1:])
        elif ord(payload[0]) == 2: # barometer measurement
            assert len(payload) == 7
            dd = map(ord, payload[1:])
            
            D1 = (dd[0]<<16)|(dd[1]<<8)|dd[2]
            D2 = (dd[3]<<16)|(dd[4]<<8)|dd[5]
            
            dT = D2 - prom[5] * 2**8
            
            TEMP = 2000 + dT * prom[6] / 2**23
            OFF = prom[2] * 2**16 + (prom[4] * dT) / 2**7
            SENS = prom[1] * 2**15 + (prom[3] * dT) / 2**8
            
            if TEMP < 2000:
                T2 = dT**2 / 2**31
                OFF2 = 5 * (TEMP - 2000)**2 / 2**1
                SENS2 = 5 * (TEMP - 2000)**2 / 2**2
                if TEMP < -1500:
                    OFF2 = OFF2 + 7 * (TEMP + 1500)**2
                    SENS2 = SENS2 + 11 * (TEMP + 1500)**2 / 2**1
            else:
                T2 = 0
                OFF2 = 0
                SENS2 = 0
            TEMP = TEMP - T2
            OFF = OFF - OFF2
            SENS = SENS - SENS2
            
            P = (D1 * SENS / 2**21 - OFF) / 2**15
            
            temperature = TEMP * .01
            pressure = P
            
            p0 = 101325
            L = 0.0065
            T_0 = 288.15
            g = 9.80665
            M = 0.0289644
            R = 8.31447
            
            h = (1 - pow(pressure / p0, R*L/g/M))/L*T_0
            
            print temperature, pressure, h
        elif ord(payload[0]) == 3: # ahrs measurement
            #print payload[1:].encode('hex')
            data = payload[1:]
            quat_wxyz = [x*2**-14 for x in struct.unpack('<4h', data[0x20-0x8:0x20-0x8+8])]
            print quat_wxyz
        else:
            assert False, ord(payload[0])
    else:
        pass # ignore all standard BINR messages

def parser():
    while True:
        assert ord((yield)) == DLE
        id_ = ord((yield)); assert id not in [DLE, ETX, CRC]
        payload = []
        while True:
            b = yield
            if ord(b) == DLE:
                b = yield
                if ord(b) == DLE:
                    payload.append(chr(DLE))
                elif ord(b) == ETX:
                    handle_packet(id_, ''.join(payload))
                    break
                else:
                    assert False
            else:
                payload.append(b)

with open(sys.argv[1]) as f:
    p = parser(); p.next()
    while True:
        buf = f.read(1024)
        if not buf: break
        for b in buf:
            p.send(b)
