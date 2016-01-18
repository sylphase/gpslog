#!/usr/bin/python

from __future__ import division

import csv
import sys
import struct
import math
import time
import calendar
import bisect

class TimeTracker(object):
    def __init__(self, nominal_dt):
        self._nominal_dt = nominal_dt
        
        self._countdown = 10
        self._last_time = None
    
    def update(self, last_gps_time):
        if last_gps_time is None:
            return
        
        if self._countdown:
            self._countdown -= 1
            return None
        
        measured = last_gps_time + 1/10 / 2
        
        if self._last_time is None:
            self._last_time = measured
        else:
            self._last_time += self._nominal_dt
            error = measured - self._last_time
            if abs(error) > 1:
                print 'time jump'
                self._last_time = measured
            self._last_time += .01 * error
        
        return self._last_time

def norm(xs):
    return math.sqrt(sum(x**2 for x in xs))

DLE = 0x10
ETX = 0x03
CRC = 0xFF

def write_packet(f, id_, msg):
    assert id_ != DLE and id_ != ETX and id_ != CRC
    res = []
    res.append(chr(DLE))
    res.append(chr(id_))
    for b in msg:
        if b == chr(DLE):
            res.append(chr(DLE))
            res.append(chr(DLE))
        else: res.append(b)
    res.append(chr(DLE))
    res.append(chr(ETX))
    f.write(''.join(res))

def packet_handler():
    altitude_time_tracker = TimeTracker(1/10)
    
    def handle_baro(data):
        assert len(data) == 6
        dd = map(ord, data)
        
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
        
        #print temperature, pressure, h
        
        t = altitude_time_tracker.update(last_gps_time)
        if t is not None:
            i = bisect.bisect_left(pos, (t,))
            if i == 0:
                gpsh = ''
            elif i == len(pos):
                gpsh = ''
            else:
                assert pos[i-1][0] <= t <= pos[i][0], (i, len(pos), t, pos[i][0], pos[i+1][0])
                gpsh = pos[i-1][1][2]
            altitude_writer.writerow([t, temperature, pressure, h, gpsh])
    
    def handle_mpu(i, data):
        accx, accy, accz, temp, gyrox, gyroy, gyroz = struct.unpack('>7h', data);
        acc = accx, accy, accz
        acc = [a/16384 * 9.80665 for a in acc]
        temp = temp/333.87 + 21
        gyro = gyrox, gyroy, gyroz
        gyro = [g/131 * math.pi/180 for g in gyro]
        #print 'MPU', i, acc, temp, gyro
    
    with open(sys.argv[1].rsplit('.', 1)[0] + '_altitude.csv', 'wb') as altitude_file, \
        open(sys.argv[1].rsplit('.', 1)[0] + '_fixed.binr', 'wb') as binr_file, \
        open(sys.argv[1].rsplit('.', 1)[0] + '_unprocessed_pos.csv', 'wb') as pos_file, \
        open(sys.argv[1].rsplit('.', 1)[0] + '_unprocessed_pos.kml', 'wb') as kml_file:
        
        altitude_writer = csv.writer(altitude_file)
        altitude_writer.writerow(['GPS Time/s', 'Temperature/C', 'Pressure/Pa', 'Altitude/m', 'GPS height/m'])
        
        pos_writer = csv.writer(pos_file)
        pos_writer.writerow(['GPS Time/s', 'Latitude/deg', 'Longitude/deg', 'Height/m'])
        
        kml_file.write('''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://earth.google.com/kml/2.1">
<Document>
<Style id="P0">
  <IconStyle>
    <color>ffffffff</color>
    <scale>0.3</scale>
    <Icon><href>http://maps.google.com/mapfiles/kml/pal2/icon18.png</href></Icon>
  </IconStyle>
</Style>
<Style id="P1">
  <IconStyle>
    <color>ff008800</color>
    <scale>0.2</scale>
    <Icon><href>http://maps.google.com/mapfiles/kml/pal2/icon18.png</href></Icon>
  </IconStyle>
</Style>
<Style id="P2">
  <IconStyle>
    <color>ff00aaff</color>
    <scale>0.2</scale>
    <Icon><href>http://maps.google.com/mapfiles/kml/pal2/icon18.png</href></Icon>
  </IconStyle>
</Style>
<Style id="P3">
  <IconStyle>
    <color>ff0000ff</color>
    <scale>0.2</scale>
    <Icon><href>http://maps.google.com/mapfiles/kml/pal2/icon18.png</href></Icon>
  </IconStyle>
</Style>
<Style id="P4">
  <IconStyle>
    <color>ff00ffff</color>
    <scale>0.2</scale>
    <Icon><href>http://maps.google.com/mapfiles/kml/pal2/icon18.png</href></Icon>
  </IconStyle>
</Style>
<Style id="P5">
  <IconStyle>
    <color>ffff00ff</color>
    <scale>0.2</scale>
    <Icon><href>http://maps.google.com/mapfiles/kml/pal2/icon18.png</href></Icon>
  </IconStyle>
</Style>
<Placemark>
<name>Rover Track</name>
<Style>
<LineStyle>
<color>ff00ffff</color>
</LineStyle>
</Style>
<LineString>
<altitudeMode>absolute</altitudeMode>
<coordinates>
''')
        kml_lines = []
        kml_wrote = False
        
        prom = None
        last_gps_time = None
        countdown = 10
        try:
            while True:
                id_, payload = yield
                if id_ == 0:
                    if ord(payload[0]) == 1: # barometer prom
                        prom = struct.unpack('>8H', payload[1:])
                    elif ord(payload[0]) == 4: # baro + 4x IMU
                        assert len(payload) == 1 + 6 + 4 * 14, len(payload)
                        handle_baro(payload[1:7])
                        for i in xrange(4):
                            handle_mpu(i, payload[7+i*14:7+i*14+14])
                    else:
                        assert False, ord(payload[0])
                elif id_ == 0xF5: # raw GPS measurements
                    tow, week = struct.unpack('<dH', payload[:10])
                    s = 315964800 + 24*60*60*7 * (1024+week) + tow/1000
                    if last_gps_time is not None and abs(s-last_gps_time - .1) > .11:
                        print s - last_gps_time
                    last_gps_time = s
                    
                    res = payload[:27]
                    
                    if len(payload[27:]) % 30 != 0:
                        print 'misshapen'
                        continue
                    assert len(payload[27:]) % 30 == 0, len(payload[27:])
                    sats = []
                    for i in xrange(len(payload[27:]) // 30):
                        d = payload[27+30*i:27+30*i+30]
                        (sig_type, sat_num, carrier_num, snr,
                            carrier_phase, pseudo_range, doppler_freq,
                            flags, _reserved) = struct.unpack('<BBbBdddBB', d)
                        if sig_type != 0x02: continue
                        assert flags & 0x01
                        if not flags & 0x02:
                            pseudo_range = doppler_freq = None
                        if not flags & 0x08:
                            carrier_phase = None
                        if not flags & 0x10:
                            pseudo_range = None
                            doppler_freq = None
                        
                        if carrier_phase is not None:
                            res += d
                            sats.append(snr)
                    if countdown:
                        countdown -= 1
                    else:
                        write_packet(binr_file, id_, res)
                        #print repr(s), time.asctime(time.gmtime(s)), sorted(sats)
                elif id_ == 0x88:
                    latitude_rad, longitude_rad, height_m, rms_pos_error_m, tow_ms_IM, tow_ms_se, WN, lat_vel, lon_vel, alt_vel, dev_m, status = struct.unpack('<dddfQHhdddfB', payload)
                    tow_ms_M = tow_ms_IM & (2**63-1)
                    tow_ms_s = tow_ms_se>>15
                    tow_ms_e = tow_ms_se & 0x7fff
                    if 0 < tow_ms_e < 32767: tow_ms = -1**(tow_ms_s) * 2**(tow_ms_e-16383.) * (1 + tow_ms_M/2**63)
                    else: tow_ms = -1**(tow_ms_s) * 2**-16382 * (tow_ms_M/2**63)
                    
                    if status & 0b1:
                        pos_writer.writerow([WN * (24*60*60*7) + tow_ms*1e-3, math.degrees(latitude_rad), math.degrees(longitude_rad), height_m])
                        kml_file.write('%f,%f,%f\n' % (math.degrees(longitude_rad), math.degrees(latitude_rad), height_m))
                        kml_lines.append('%f,%f,%f' % (math.degrees(longitude_rad), math.degrees(latitude_rad), height_m))
                        kml_wrote = True
                    elif kml_wrote:
                        kml_file.write('''</coordinates>
</LineString>
</Placemark>
<Placemark>
<name>Rover Track</name>
<Style>
<LineStyle>
<color>ff00ffff</color>
</LineStyle>
</Style>
<LineString>
<altitudeMode>absolute</altitudeMode>
<coordinates>
''')
                        kml_wrote = False
                else:
                    pass # ignore all other standard BINR messages
                    #write_packet(binr_file, id_, payload)
        finally:
            kml_file.write('''</coordinates>
</LineString>
</Placemark>
''')
            if 1:
                kml_file.write('''<Folder>
  <name>Rover Position</name>
''')
                for l in kml_lines[::10]:
                    kml_file.write('''
<Placemark>
<styleUrl>#P2</styleUrl>
<Point>
<coordinates>%s</coordinates>
</Point>
</Placemark>
''' % (l,))
                kml_file.write('''</Folder>
''')
            kml_file.write('''</Document>
</kml>
''')

def parser():
    handler = packet_handler(); handler.next()
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
                    handler.send((id_, ''.join(payload)))
                    break
                else:
                    assert False
            else:
                payload.append(b)


pos = []
if len(sys.argv) == 3:
    with open(sys.argv[2], 'rb') as f:
        for line in f:
            if line.startswith('%'): continue
            GPST_date, GPST_time, latitude, longitude, height, Q, ns, sdn, sde, sdu, sdne, sdeu, sdun, age, ratio = line.split()
            gpst = calendar.timegm(time.strptime(GPST_date + " " + GPST_time.split('.', 1)[0], '%Y/%m/%d %H:%M:%S')) + float('0.' + GPST_time.split('.', 1)[1])
            pos.append((gpst, (latitude, longitude, height)))
    is_sorted = lambda x: sorted(x) == x
    assert is_sorted([gpst for gpst, p in pos])

with open(sys.argv[1], 'rb') as f:
    p = parser(); p.next()
    while True:
        buf = f.read(1024)
        if not buf: break
        for b in buf:
            p.send(b)
