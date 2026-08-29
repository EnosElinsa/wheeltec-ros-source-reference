# coding=utf-8
import threading
import serial
import time
import math
import pyproj
# ================= CRC =================
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

# ================= Serial =================
def open_serial_with_retry(port, baudrate, retry=5, delay=1):
    for i in range(retry):
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            print("Serial %s open successfully!" % port)
            return ser
        except serial.SerialException as e:
            print("Serial %s failed to open: %s" % (port, e))
            time.sleep(delay)

    raise serial.SerialException("Serial %s could not be opened after %d attempts" % (port, retry)    )

# ================= NMEA =================
def nmea_expend_crc(sentence):
    # def calc_crc32(data):
    #     crc = 0
    #     for b in data:
    #         crc = NMEA_EXPEND_CRC_TABLE[(crc ^ b) & 0xFF] ^ (crc >> 8)
    #     return crc & 0xFFFFFFFF
    def calc_crc32(data):
        crc = 0
        for byte in data:
            if isinstance(byte, str):
                byte = ord(byte)
                crc = NMEA_EXPEND_CRC_TABLE[(crc ^ byte) & 0xFF] ^ (crc >> 8)
        return crc & 0xFFFFFFFF

    try:
        body, crc = sentence[1:].split("*")
        crc = crc[:8]
    except:
        return False
    calc = calc_crc32(body.encode('utf-8'))
    return crc.lower() == format(calc, '08x')

def nmea_crc(sentence):
    try:
        body, crc = sentence[1:].split("*")
        crc = crc[:2]
    except:
        return False
    checksum = 0
    for c in body:
        checksum ^= ord(c)
    return format(checksum, '02X') == crc.upper()

def msg_seperate(msg):
    return msg[1:msg.find('*')].split(',')

# ================= Solvers =================

def PVTSLN_solver(msg):
    p = msg_seperate(msg)
    return (
        float(p[10]),  # height
        float(p[11]),  # lat
        float(p[12]),  # lon
        float(p[13]),
        float(p[14]),
        float(p[15])
    )

def GNHPR_solver(msg):
    p = msg_seperate(msg)
    return float(p[2]), float(p[3]), float(p[4])


def BESTNAV_solver(msg):
    p = msg_seperate(msg)
    vel_hor = float(p[-5])
    heading = float(p[-4])

    vel_north = vel_hor * math.cos(math.radians(heading))
    vel_east  = vel_hor * math.sin(math.radians(heading))

    return (
        vel_east,
        vel_north,
        float(p[-3]),
        float(p[-1]),
        float(p[-1]),
        float(p[-2])
    )


# ================= UTM =================

def create_utm_trans(lat, lon):
    zone = int((lon + 180) / 6) + 1
    if lat >= 0:
        utm = pyproj.Proj(proj='utm', zone=zone, ellps='WGS84')
    else:
        utm = pyproj.Proj(proj='utm', zone=zone, south=True, ellps='WGS84')

    wgs84 = pyproj.Proj(proj='latlong', datum='WGS84')
    return wgs84, utm


def utm_trans(trans, lon, lat):
    wgs84, utm = trans
    return pyproj.transform(wgs84, utm, lon, lat)


# ================= Thread =================

class UM982Serial(threading.Thread):

    def __init__(self, port, baud):
        threading.Thread.__init__(self)

        self.ser = open_serial_with_retry(port, baud)
        self.isRUN = True

        self.fix = None
        self.orientation = None
        self.vel = None
        self.utmpos = None
        self.transformer = None

    def stop(self):
        self.isRUN = False
        time.sleep(0.1)
        self.ser.close()

    def read_frame(self):
        try:
            raw = self.ser.readline()
            frame = raw.decode('utf-8', 'ignore').strip()

            if frame.startswith("#PVTSLNA") and nmea_expend_crc(frame):
                self.fix = PVTSLN_solver(frame)

            elif frame.startswith("$GNHPR") and nmea_crc(frame):
                self.orientation = GNHPR_solver(frame)

            elif frame.startswith("#BESTNAVA") and nmea_expend_crc(frame):
                self.vel = BESTNAV_solver(frame)

        except Exception as e:
            print("Serial error:", e)

    def run(self):
        # 等待首次定位
        while self.isRUN and self.fix is None:
            self.read_frame()

        if not self.isRUN:
            return

        h, lat, lon, _, _, _ = self.fix
        self.transformer = create_utm_trans(lat, lon)
        self.utmpos = utm_trans(self.transformer, lon, lat)

        print("GNSS init OK:", self.fix)

        while self.isRUN:
            self.read_frame()
            if self.fix:
                _, lat, lon, _, _, _ = self.fix
                self.utmpos = utm_trans(self.transformer, lon, lat)


# ================= Main =================

if __name__ == "__main__":
    um982 = UM982Serial("/dev/wheeltec_gnss", 115200)
    um982.start()
