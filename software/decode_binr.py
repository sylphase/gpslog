#!/usr/bin/python

from __future__ import division

import csv
import sys
import struct
import math
import time
import calendar
import bisect

sinc = lambda x: math.sin(x)/x if x else 1
sincsqrt = lambda x: math.sin(math.sqrt(x))/math.sqrt(x) if x else 1
cossqrt = lambda x: math.cos(math.sqrt(x))
sign_never_0 = lambda x: 1 if x >= 0 else -1
sincacos_clipping = lambda x: sinc(math.acos(min(x, 1)))
if_positive = lambda x, y, z: y if x >= 0 else z

# epsilon for testing whether a number is close to zero
_EPS = 8.8817841970012523e-16

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

def euler_from_matrix(matrix, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> numpy.allclose(R0, R1)
    True
    >>> angles = (4*math.pi) * (numpy.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not numpy.allclose(R0, R1): print(axes, "failed")

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # validation
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = matrix
    if repetition:
        sy = math.sqrt(M[i][j]*M[i][j] + M[i][k]*M[i][k])
        if sy > _EPS:
            ax = math.atan2( M[i][j],  M[i][k])
            ay = math.atan2( sy,       M[i][i])
            az = math.atan2( M[j][i], -M[k][i])
        else:
            ax = math.atan2(-M[j][k],  M[j][j])
            ay = math.atan2( sy,       M[i][i])
            az = 0.0
    else:
        cy = math.sqrt(M[i][i]*M[i][i] + M[j][i]*M[j][i])
        if cy > _EPS:
            ax = math.atan2( M[k][j],  M[k][k])
            ay = math.atan2(-M[k][i],  cy)
            az = math.atan2( M[j][i],  M[i][i])
        else:
            ax = math.atan2(-M[j][k],  M[j][j])
            ay = math.atan2(-M[k][i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az

class V(tuple):
    def __init__(self, x):
        tuple.__init__(x)
        for y in x: assert isinstance(y, (int, float))
    
    def __neg__(self): return V(-x for x in self)
    def __pos__(self): return self
    def __add__(self, other):
        if isinstance(other, V) and len(other) == len(self):
            return V(a+b for a, b in zip(self, other))
        else:
            return NotImplemented
    def __sub__(self, other):
        if isinstance(other, V) and len(other) == len(self):
            return V(a-b for a, b in zip(self, other))
        else:
            return NotImplemented
    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return V(a*other for a in self)
        elif isinstance(other, V) and len(other) == len(self):
            return sum(a*b for a, b in zip(self, other))
        elif isinstance(other, Matrix) and other.rows == 1:
            return Matrix([[a*b for b in other._contents[0]] for a in self])
        else:
            return NotImplemented
    def __rmul__(self, other):
        if isinstance(other, (int, float)):
            return V(other*a for a in self)
        elif isinstance(other, Matrix) and other.cols == len(self):
            return self.__class__(V(row)*self for row in other)
        else:
            return NotImplemented
    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            return V(a/other for a in self)
        else:
            return NotImplemented
    
    def __mod__(self, other): # cross product
        if isinstance(other, self.__class__) and len(self) == 3 and len(other) == 3:
            (x, y, z), (X, Y, Z) = self, other
            return self.__class__([y*Z-z*Y, z*X-x*Z, x*Y-y*X])
        else:
            return NotImplemented
    
    @property
    def as_row_matrix(self):
        return Matrix([list(self)])
    @property
    def as_diagonal_matrix(self):
        return Matrix([[self[i] if i == j else 0 for j in xrange(len(self))] for i in xrange(len(self))])
    @property
    def as_scalar(self):
        assert len(self) == 1
        return self[0]
    
    def __repr__(self):
        return 'v%s' % tuple.__repr__(self)
    
    def __getitem__(self, item):
        if isinstance(item, slice):
            return V(tuple.__getitem__(self, item))
        else:
            return tuple.__getitem__(self, item)
    def __getslice__(self, i, j):
        return self.__getitem__(slice(i, j))
    
    def norm(self):
        return math.sqrt(self*self)
    def norm2(self):
        return self*self
    def unit(self):
        return self/self.norm()
def v(*args):
    return V(args)

class Quaternion(object):
    def __init__(self, **kwargs):
        assert set(kwargs.keys()) == {'w', 'x', 'y', 'z'}
        for x in kwargs.values(): assert isinstance(x, (int, float))
        self.__dict__.update(kwargs)
    
    def __repr__(self):
        return 'Quaternion(w=%r, x=%r, y=%r, z=%r)' % (self.w, self.x, self.y, self.z)
    
    @classmethod
    def from_axisangle(cls, axis, angle):
        assert isinstance(axis, V) and len(axis) == 3
        v = math.sin(angle/2) * axis.unit()
        return cls(w=math.cos(angle/2), x=v[0], y=v[1], z=v[2])

    @classmethod
    def from_rotvec(cls, r):
        assert isinstance(r, V) and len(r) == 3
        angle2 = r.norm2()
        v = r * (sincsqrt(angle2*.25)*.5)
        return cls(w=cossqrt(angle2*.25), x=v[0], y=v[1], z=v[2])
    
    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return self.__class__(**{k: v*other for k, v in self.__dict__.iteritems()})
        else:
            return NotImplemented
    def __rmul__(self, other):
        if isinstance(other, (int, float)):
            return self.__class__(**{k: other*v for k, v in self.__dict__.iteritems()})
        else:
            return NotImplemented
    
    def __mod__(self, other):
        if isinstance(other, self.__class__):
            return self.__class__(
                w=other.w * self.w - other.x * self.x - other.y * self.y -  other.z * self.z,
                x=other.x * self.w + other.w * self.x + other.z * self.y -  other.y * self.z,
                y=other.y * self.w - other.z * self.x + other.w * self.y +  other.x * self.z,
                z=other.z * self.w + other.y * self.x - other.x * self.y +  other.w * self.z,
            )
        else:
            return NotImplemented
    
    @property
    def norm_squared(self):
        return self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z
    
    @property
    def conj(self):
        return self.__class__(w=self.w, x=-self.x, y=-self.y, z=-self.z)
    @property
    def normalized(self):
        norm = math.sqrt(sum(x**2 for x in self.__dict__.itervalues()))
        return self * (1/norm)
    
    @property
    def as_matrix(self):
        return [
            [1 - 2*self.y*self.y - 2*self.z*self.z,     2*self.x*self.y - 2*self.z*self.w,     2*self.x*self.z + 2*self.y*self.w],
            [    2*self.x*self.y + 2*self.z*self.w, 1 - 2*self.x*self.x - 2*self.z*self.z,     2*self.y*self.z - 2*self.x*self.w],
            [    2*self.x*self.z - 2*self.y*self.w,     2*self.y*self.z + 2*self.x*self.w, 1 - 2*self.x*self.x - 2*self.y*self.y],
        ]
    
    @property
    def as_axisangle(self):
        self = self.unit()
        if self.w < 0:
            self = -self
        return v(self.x, self.y, self.z).unit(), math.acos(self.w) * 2
    @property
    def as_rotvec(self): # q must be normalized already
        self *= sign_never_0(self.w) # make sure that q.w is positive
        return 2/sincacos_clipping(self.w) * v(self.x, self.y, self.z)
    
    def rotate(self, v):
        assert isinstance(v, V) and len(v) == 3
        r = self % self.__class__(w=0, x=v[0], y=v[1], z=v[2]) % self.conj
        return V((r.x, r.y, r.z))

def triad(v1_world, v2_world, v1_body, v2_body):
    # from "Fast Quaternion Attitude Estimation From Two Vector Measurements"
    # requires all vectors to be normalized
    # XXX has singularity at 180 deg rotation TODO: work around by rotating everything
    
    b1, b2 = v1_body, v2_body
    r1, r2 = v1_world, v2_world
    
    mu = (1 + b1 * r1) * ((b1 % b2) * (r1 % r2)) - (b1 * (r1 % r2)) * (r1 * (b1 % b2))
    v = (b1 + r1) * ((b1 % b2) % (r1 % r2))
    
    rho = math.sqrt(mu**2 + v**2)
    
    q_scale = 1/(2*math.sqrt(rho * (rho + abs(mu)) * (1 + b1 * r1)))
    
    q_vector_if_mu_pos = q_scale * ((rho + mu)*(b1 % r1) + v*(b1 + r1))
    q_scalar_if_mu_pos = q_scale * (rho + mu) * (1 + b1 * r1)
    
    q_vector_if_mu_neg = q_scale * (v*(b1 % r1) + (rho - mu)*(b1 + r1))
    q_scalar_if_mu_neg = q_scale * v * (1 + b1 * r1)
    
    q_scalar = if_positive(mu, q_scalar_if_mu_pos, q_scalar_if_mu_neg)
    q_vector = V(if_positive(mu, a, b) for a, b in zip(q_vector_if_mu_pos, q_vector_if_mu_neg))
    
    return Quaternion(w=q_scalar, **dict(zip('xyz', q_vector)))

class IMUSolver(object):
    def __init__(self, dt):
        self.dt = dt
        self.orientation = None
        self.mag_min = None
    def handle(self, acc, gyro, mag):
        if self.mag_min is None:
            self.mag_min = mag
            self.mag_max = mag
        self.mag_min = map(min, self.mag_min, mag)
        self.mag_max = map(max, self.mag_max, mag)
        if any(a == b for a, b in zip(self.mag_min, self.mag_max)): return None
        mag_corrected = [(m - lo)/(hi - lo) - 0.5 for m, lo, hi in zip(mag, self.mag_min, self.mag_max)]
        measurement = triad(v(0, 0, 1), v(0, 1, 0), V(acc).unit(), V(mag_corrected).unit())
        print mag, mag_corrected
        if self.orientation is None:
            self.orientation = measurement
            res = self.orientation
        else:
            self.orientation = self.orientation % Quaternion.from_rotvec(V(gyro) * (self.dt/2))
            error = (measurement % self.orientation.conj).as_rotvec
            self.orientation = Quaternion.from_rotvec(error * (self.dt/3)) % self.orientation
            res = self.orientation
            self.orientation = self.orientation % Quaternion.from_rotvec(V(gyro) * (self.dt/2))
        self.orientation = self.orientation.normalized
        if res.w < 0: res *= -1
        return res

class TimeTracker(object):
    def __init__(self):
        self._offset = None
    
    def update(self, gps_time, device_time):
        offset = gps_time - device_time
        
        if self._offset is None:
            self._offset = offset
        
        if abs(offset - self._offset) > 1:
            print 'time jump'
        
        self._offset = .99 * self._offset + .01 * offset
    
    def get(self, device_time):
        if self._offset is None:
            return None
        return self._offset + device_time

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
    time_tracker = TimeTracker()
    imu_solvers = [IMUSolver(.1) for i in xrange(4)]
    
    def handle_baro(ts, data):
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
        
        t = time_tracker.get(ts)
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
    
    def handle_mpu(ts, i, data):
        if data.encode('hex') == 'ffffffffffffffffffffffffffffffffffffffffffff': return
        t = time_tracker.get(ts)
        accx, accy, accz, temp, gyrox, gyroy, gyroz = struct.unpack('>7h', data[:14])
        mag_st1, magx, magy, magz, mag_st2 = struct.unpack('<B3hB', data[14:])
        acc = accx, accy, accz
        acc = [a/16384 * 9.80665 for a in acc]
        temp = temp/333.87 + 21
        gyro = gyrox, gyroy, gyroz
        gyro = [g/131 * math.pi/180 for g in gyro]
        mag = magy, magx, -magz
        mag = [m*0.15e-6 for m in mag]
        q = imu_solvers[i].handle(acc, gyro, mag)
        t = time_tracker.get(ts)
        if t is not None:
            #print 'MPU', i, t, acc, temp, gyro, mag
            imu_writers[i].writerow([t] + acc + gyro + mag + ([q.w, q.x, q.y, q.z] + list(map(math.degrees, euler_from_matrix(q.as_matrix))) if q is not None else ['']*7))
    
    altitude_file = open(sys.argv[1].rsplit('.', 1)[0] + '_altitude.csv', 'wb')
    binr_file = open(sys.argv[1].rsplit('.', 1)[0] + '_fixed.binr', 'wb')
    pos_file = open(sys.argv[1].rsplit('.', 1)[0] + '_unprocessed_pos.csv', 'wb')
    kml_file = open(sys.argv[1].rsplit('.', 1)[0] + '_unprocessed_pos.kml', 'wb')
    imu_files = [open(sys.argv[1].rsplit('.', 1)[0] + 'imu%i.csv' % (i,), 'wb') for i in xrange(4)]
        
    altitude_writer = csv.writer(altitude_file)
    altitude_writer.writerow(['GPS Time/s', 'Temperature/C', 'Pressure/Pa', 'Altitude/m', 'GPS height/m'])
    
    pos_writer = csv.writer(pos_file)
    pos_writer.writerow(['GPS Time/s', 'Latitude/deg', 'Longitude/deg', 'Height/m'])
    
    imu_writers = map(csv.writer, imu_files)
    for w in imu_writers:
        w.writerow(['GPS Time/s', 'X acceleration/(m/s^2)', 'Y acceleration/(m/s^2)', 'Z acceleration/(m/s^2)', 'X angular velocity/(rad/s)', 'Y angular velocity/(rad/s)', 'Z angular velocity/(rad/s)', 'X magnetic field/(tesla)', 'Y magnetic field/(tesla)', 'Z magnetic field/(tesla)', 'Quaternion w', 'Quaternion x', 'Quaternion y', 'Quaternion z', 'Roll/deg', 'Pitch/deg', 'Yaw/deg'])
    
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
    countdown = 10
    last_ts = None
    try:
        while True:
            ts = last_ts
            last_ts = None
            id_, payload = yield
            if id_ == 0:
                if ord(payload[0]) == 1: # barometer prom
                    prom = struct.unpack('>8H', payload[1:])
                elif ord(payload[0]) == 4: # baro + 4x IMU
                    assert len(payload) == 1 + 6 + 4 * (14+8), len(payload)
                    handle_baro(ts, payload[1:7])
                    for i in xrange(4):
                        handle_mpu(ts, i, payload[7+i*(14+8):7+(i+1)*(14+8)])
                elif ord(payload[0]) == 5:
                    last_ts, = struct.unpack('<Q', payload[1:9])
                    last_ts *= 1e-6
                else:
                    assert False, ord(payload[0])
            elif id_ == 0xF5: # raw GPS measurements
                tow, week = struct.unpack('<dH', payload[:10])
                gps_time = 315964800 + 24*60*60*7 * (1024+week) + tow/1000
                
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
                if 0 < tow_ms_e < 32767: tow_ms = (-1)**(tow_ms_s) * 2**(tow_ms_e-16383.) * (1 + tow_ms_M/2**63)
                else: tow_ms = (-1)**(tow_ms_s) * 2**-16382 * (tow_ms_M/2**63)
                
                if status & 0b1:
                    gps_time = 315964800 + (1024 + WN) * (24*60*60*7) + tow_ms*1e-3
                    time_tracker.update(gps_time, ts)
                    pos_writer.writerow([gps_time, math.degrees(latitude_rad), math.degrees(longitude_rad), height_m])
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
