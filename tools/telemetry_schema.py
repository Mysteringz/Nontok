"""MEGA3 wire contract shared by the recorder and local dashboard."""
import csv
import math

FIELDS = 'ms,mode,armed,left45_mm,right45_mm,gyro_deg,gyro_dps,imu_ok,light_left,light_middle,light_right,light_ok,red,green,blue,clear,color_ok,color,decision,x_mm,y_mm,yaw_deg,pose_ok,heading_ok,beacon_residual_mm,us_fused,us_residual_mm,reference,target,vx,vy,wz,calibrated,hardware,fault'.split(',')
TEXT_FIELDS = {'mode', 'color', 'decision', 'reference', 'target', 'fault'}
MODES = {'DISARMED','IDLE','MANUAL','DEMO','TO_CENTER','READ_COLOR','TURN','TO_BOX','DOCK','FINISHED','FAULT'}
POINTS = {'NONE','CENTER','TURN_LEFT','TURN_RIGHT','BOX_LEFT','BOX_RIGHT'}
BOOL_FIELDS = {'armed','imu_ok','light_ok','color_ok','pose_ok','heading_ok','us_fused','calibrated'}


def parse_line(line):
    if not line or line.startswith('#') or len(line) > 1024:
        return None
    row = next(csv.reader([line]))
    if len(row) != len(FIELDS) or not row[0].isdigit():
        return None
    data = {}
    try:
        for key, value in zip(FIELDS, row):
            if key in TEXT_FIELDS:
                if not value or len(value)>48 or not all(c.isupper() or c.isdigit() or c=='_' for c in value):
                    return None
                data[key] = value
            else:
                number = float(value)
                if not math.isfinite(number) or abs(number)>1e12:
                    return None
                data[key] = int(number) if number.is_integer() else number
        if not 0 <= data['ms'] <= 0xffffffff or data['mode'] not in MODES:
            return None
        if data['color'] not in {'UNKNOWN','RED','GREEN'} or data['decision'] not in {'UNKNOWN','RED','GREEN'}:
            return None
        if data['reference'] not in POINTS or data['target'] not in POINTS:
            return None
        limits = {'left45_mm':(-1,4000), 'right45_mm':(-1,4000), 'x_mm':(0,10000), 'y_mm':(-10000,10000),
                  'yaw_deg':(-180,180), 'gyro_deg':(-1000000,1000000), 'gyro_dps':(-1000,1000),
                  'vx':(-150,150), 'vy':(-150,150), 'wz':(-45,45), 'hardware':(0,15),
                  **{k:(0,1023) for k in ('light_left','light_middle','light_right')},
                  **{k:(0,65535) for k in ('red','green','blue','clear')}}
        if any(not lo<=data[k]<=hi for k,(lo,hi) in limits.items()):
            return None
        if any(data[k] not in (0,1) for k in BOOL_FIELDS):
            return None
    except (ValueError, OverflowError):
        return None
    return data
