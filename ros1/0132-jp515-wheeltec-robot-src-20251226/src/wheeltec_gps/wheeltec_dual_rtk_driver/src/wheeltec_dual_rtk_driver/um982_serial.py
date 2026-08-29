# coding=utf-8
from __future__ import print_function

import threading
import serial
import time
import math

try:
    from pyproj import CRS, Transformer
    HAS_PYPROJ = True
except ImportError:
    HAS_PYPROJ = False


def crc_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
        table.append(crc)
    return table


NMEA_EXPEND_CRC_TABLE = crc_table()


def open_serial_with_retry(port, baudrate, retry=5, delay=1):
    for i in range(retry):
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            print("Serial {} open successfully!".format(port))
            return ser
        except serial.SerialException as e:
            print("Serial {} failed to open: {}".format(port, e))
            time.sleep(delay)
    raise serial.SerialException("Serial {} could not be opened".format(port))


def nmea_expend_crc(sentence):
    def calculate_crc32(data):
        crc = 0
        for b in data:
            if isinstance(b, str):   # Python2
                b = ord(b)
            crc = NMEA_EXPEND_CRC_TABLE[(crc ^ b) & 0xFF] ^ (crc >> 8)
        return crc & 0xFFFFFFFF

    try:
        body, crc = sentence[1:].split("*")
        crc = crc[:8]
    except Exception:
        return False

    calc = calculate_crc32(body.encode('ascii'))
    return crc.lower() == format(calc, '08x')


def nmea_crc(sentence):
    try:
        body, crc = sentence[1:].split("*")
        crc = crc[:2]
    except Exception:
        return False

    cs = 0
    for c in body:
        cs ^= ord(c)
    return format(cs, '02X') == crc.upper()


def msg_seperate(msg):
    return msg[1:msg.find('*')].split(',')


def PVTSLN_solver(msg):
    p = msg_seperate(msg)
    return (
        float(p[10]),  # hgt
        float(p[11]),  # lat
        float(p[12]),  # lon
        float(p[13]),  # hgt std
        float(p[14]),  # lat std
        float(p[15])   # lon std
    )


def GNHPR_solver(msg):
    p = msg_seperate(msg)
    return float(p[2]), float(p[3]), float(p[4])


def BESTNAV_solver(msg):
    p = msg_seperate(msg)
    vel_hor = float(p[-5])
    heading = float(p[-4])
    vel_ver = float(p[-3])
    vel_hor_std = float(p[-1])
    vel_ver_std = float(p[-2])

    ve = vel_hor * math.sin(math.radians(heading))
    vn = vel_hor * math.cos(math.radians(heading))
    return ve, vn, vel_ver, vel_hor_std, vel_hor_std, vel_ver_std


def create_utm_trans(lat, lon):
    if not HAS_PYPROJ:
        return None

    zone = int((lon + 180) / 6) + 1
    epsg = "epsg:326{}".format(zone) if lat >= 0 else "epsg:327{}".format(zone)
    return Transformer.from_crs(CRS("epsg:4326"), CRS(epsg), always_xy=True)


def utm_trans(transformer, lon, lat):
    return transformer.transform(lon, lat)


class UM982Serial(threading.Thread):
    def __init__(self, port, baud):
        super(UM982Serial, self).__init__()
        self.daemon = True

        self.ser = open_serial_with_retry(port, baud)

        self.isRUN = True
        self.fix = None
        self.orientation = None
        self.vel = None
        self.utmpos = None
        self.transformer = None

        print("GNSS init, waiting for fix...")

    def stop(self):
        self.isRUN = False
        time.sleep(0.1)
        self.ser.close()

    def read_frame(self):
        try:
            line = self.ser.readline()
            if not line:
                return
            frame = line.decode('utf-8', errors='ignore').strip()

            if frame.startswith("#PVTSLNA") and nmea_expend_crc(frame):
                self.fix = PVTSLN_solver(frame)
            elif frame.startswith("$GNHPR") and nmea_crc(frame):
                self.orientation = GNHPR_solver(frame)
            elif frame.startswith("#BESTNAVA") and nmea_expend_crc(frame):
                self.vel = BESTNAV_solver(frame)

        except Exception as e:
            print("Serial parse error:", e)

    def run(self):
        while self.isRUN:
            self.read_frame()
            if self.fix is not None and self.transformer is None:
                h, lat, lon, _, _, _ = self.fix
                self.transformer = create_utm_trans(lat, lon)
                if self.transformer:
                    print("GNSS fix acquired, UTM initialized")
            if self.fix is not None and self.transformer is not None:
                _, lat, lon, _, _, _ = self.fix
                self.utmpos = utm_trans(self.transformer, lon, lat)


if __name__ == "__main__":
    um = UM982Serial("/dev/wheeltec_gnss", 115200)
    um.start()
    while True:
        time.sleep(1)
