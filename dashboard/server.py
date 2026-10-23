#!/usr/bin/env python3
"""Local Mega telemetry, SQLite recordings and browser UI. No navigation runs here."""
import argparse
import csv
import io
import json
import math
import queue
import re
import sqlite3
import sys
import threading
import time
import uuid
from collections import deque
from contextlib import closing
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from telemetry_schema import FIELDS, parse_line

STATIC = Path(__file__).parent / 'static'
POINT = r'(?:CENTER|TURN_LEFT|TURN_RIGHT|BOX_LEFT|BOX_RIGHT)'
NUM = r'-?(?:\d+(?:\.\d*)?|\.\d+)'
COMMANDS = [r'(?:STOP|ARM|START|STATUS|PING|CALIB|PROBE|SAVE|LOAD|DEFAULTS|DEMO)',
            rf'HEADING {NUM}', rf'COLOR (?:RED|GREEN)',
            rf'TEACH {POINT} {NUM} {NUM} {NUM}', rf'SAMPLE {POINT} (?:X|Y|YAW) {NUM}',
            rf'BEACON (?:0|1|2)(?: {NUM}){{7}}',
            r'(?:FWD|BACK|LEFT|RIGHT|CW|CCW) (?:[1-9]\d{0,3})',
            r'SURVEY (?:STATUS|DARK|SOLVE|APPLY|DUMP|CLEAR|END|CANCEL)',
            rf'SURVEY (?:MOUNTS|OPTICS)(?: {NUM}){{4}}',
            rf'SURVEY BEGIN(?: {NUM}){{3}}',rf'SURVEY POSE(?: {NUM}){{2}}',
            r'SURVEY TAKE (?:WALL|POINT)']


def utc():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


def survey_line(line):
    for prefix,kind in (('# SURVEY_SAMPLE ','sample'),('# SURVEY ','state')):
        if line.startswith(prefix):
            obj=json.loads(line[len(prefix):],parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite survey value')))
            if not isinstance(obj,dict): raise ValueError('Expected survey object')
            if kind=='sample' and (type(obj.get('index')) is not int or not 0<=obj['index']<40):
                raise ValueError('Invalid survey sample index')
            return kind,obj
    return None,None


def survey_record(events):
    state=None
    samples={}
    for line in events:
        try: kind,obj=survey_line(line)
        except ValueError: continue
        if kind=='state':
            state={**(state or {}),**obj}
            if obj.get('count')==0: samples.clear()
        elif kind=='sample': samples[obj['index']]=obj
    return {'survey':state,'samples':[samples[i] for i in sorted(samples)]}


class Store:
    def __init__(self, path):
        self.lock = threading.RLock()
        self.path = path
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.executescript('''CREATE TABLE IF NOT EXISTS runs
          (id TEXT PRIMARY KEY, started TEXT, ended TEXT, source TEXT, port TEXT, count INTEGER DEFAULT 0);
          CREATE TABLE IF NOT EXISTS samples (id INTEGER PRIMARY KEY,run TEXT,elapsed REAL,data TEXT);
          CREATE INDEX IF NOT EXISTS samples_run ON samples(run,id);
          CREATE TABLE IF NOT EXISTS events (id INTEGER PRIMARY KEY,run TEXT,at TEXT,message TEXT);
          CREATE INDEX IF NOT EXISTS events_run ON events(run,id);''')
        # A previous Mac/server crash must leave an honest, recoverable recording.
        self.db.execute('UPDATE runs SET ended=? WHERE ended IS NULL', (utc(),))
        self.db.commit()

    def start(self, source, port):
        with self.lock:
            rid = uuid.uuid4().hex
            self.db.execute('INSERT INTO runs (id,started,source,port) VALUES (?,?,?,?)', (rid,utc(),source,port))
            self.db.commit()
            return rid

    def end(self, rid):
        with self.lock:
            self.db.execute('UPDATE runs SET ended=? WHERE id=?', (utc(),rid))
            self.db.commit()

    def sample(self, rid, sample):
        with self.lock:
            self.db.execute('INSERT INTO samples (run,elapsed,data) VALUES (?,?,?)', (rid,sample['t'],json.dumps(sample)))
            self.db.execute('UPDATE runs SET count=count+1 WHERE id=?', (rid,))
            self.db.commit()

    def event(self, rid, message):
        with self.lock:
            self.db.execute('INSERT INTO events (run,at,message) VALUES (?,?,?)',(rid,utc(),message))
            self.db.commit()

    def runs(self):
        with self.lock:
            rows = self.db.execute("SELECT r.id,r.started,r.ended,r.source,r.port,r.count,EXISTS(SELECT 1 FROM events e WHERE e.run=r.id AND e.message LIKE ?) FROM runs r ORDER BY r.started DESC LIMIT 200",('# SURVEY {%',)).fetchall()
        return [dict(zip(('id','started','ended','source','port','count','survey'),r)) for r in rows]

    def data(self, rid, max_points=6000):
        with self.lock:
            count = self.db.execute('SELECT COUNT(*) FROM samples WHERE run=?',(rid,)).fetchone()[0]
            stride = max(1, math.ceil(count/max_points))
            rows = self.db.execute('''SELECT data FROM (SELECT data,ROW_NUMBER() OVER(ORDER BY id) n FROM samples WHERE run=?)
                                      WHERE (n-1)%?=0 OR n=? ORDER BY n''',(rid,stride,count)).fetchall()
            events = self.db.execute('SELECT at,message FROM events WHERE run=? ORDER BY id LIMIT 2000',(rid,)).fetchall()
        return {'samples':[json.loads(r[0]) for r in rows], 'count':count,'stride':stride,
                'events':[{'at':a,'message':m} for a,m in events]}

    def export(self, rid):
        # File-backed temporary output avoids loading a long recording into memory.
        import tempfile
        output = tempfile.SpooledTemporaryFile(max_size=1024*1024, mode='w+', newline='')
        writer = csv.writer(output)
        writer.writerow(['received_at','elapsed_s','sequence']+FIELDS)
        with closing(sqlite3.connect(self.path)) as reader:
            reader.execute('PRAGMA query_only=ON')
            for (raw,) in reader.execute('SELECT data FROM samples WHERE run=? ORDER BY id',(rid,)):
                row=json.loads(raw)
                writer.writerow([row['received_at'],row['t'],row['seq']]+[row[k] for k in FIELDS])
        output.seek(0)
        return output

    def survey(self, rid):
        with closing(sqlite3.connect(self.path)) as reader:
            rows=reader.execute('SELECT message FROM events WHERE run=? AND message LIKE ? ORDER BY id',(rid,'# SURVEY%'))
            data=survey_record(r[0] for r in rows)
        data.update(recording=rid,exported_at=utc())
        return data

    def close(self):
        with self.lock:
            self.db.close()


class Bridge:
    def __init__(self, store):
        self.store=store
        self.lock=threading.RLock()
        self.samples=deque(maxlen=6000)
        self.events=deque(maxlen=200)
        self.latest=None
        self.profile=None
        self.survey=None
        self.survey_samples={}
        self.serial=None
        self.thread=None
        self.halt=threading.Event()
        self.connected=False
        self.source=None
        self.port=None
        self.baud=115200
        self.run=None
        self.seq=0
        self.started=time.monotonic()
        self.record_start=self.started
        self.last_ms=None
        self.demo_mode='DISARMED'
        self.demo_green=True
        self.demo_start=0
        self.pending=queue.Queue(maxsize=32)
        self.operations=threading.Lock()

    def event(self, message):
        with self.lock:
            item={'at':utc(),'message':str(message)[:4096]}
            self.events.append(item)
            if self.run:
                self.store.event(self.run,item['message'])

    def ingest(self, line):
        if line.startswith('#'):
            self.event(line)
            if line.startswith('# SURVEY ' ) or line.startswith('# SURVEY_SAMPLE '):
                try:
                    kind,obj=survey_line(line)
                    with self.lock:
                        if kind=='state':
                            self.survey={**(self.survey or {}),**obj}
                            if obj.get('count')==0: self.survey_samples.clear()
                        elif kind=='sample': self.survey_samples[obj['index']]=obj
                except ValueError:
                    self.event('Invalid survey response')
            if line.startswith('# PROFILE '):
                try:
                    profile=json.loads(line[10:])
                    if isinstance(profile,dict):
                        with self.lock:
                            self.profile=profile
                except ValueError:
                    self.event('Invalid calibration response')
            return
        data=parse_line(line)
        if data is None:
            return
        with self.lock:
            if self.last_ms is not None and data['ms']<self.last_ms and not (self.last_ms>0xffff0000 and data['ms']<65536):
                self.event('Mega restarted: heading must be referenced again; motors boot stopped.')
            self.last_ms=data['ms']
            self.seq+=1
            data.update(seq=self.seq,received_at=utc(),t=round(time.monotonic()-self.record_start,3))
            self.latest=data
            self.samples.append(data)
            if self.run:
                self.store.sample(self.run,data)

    def connect(self, port, baud, demo=False):
        with self.operations:
            self.disconnect_unlocked()
            device=None
            if not demo:
                if not isinstance(port,str) or not port.startswith('/dev/') or len(port)>256:
                    raise ValueError('Choose a /dev/ serial port from this Mac')
                if baud not in (9600,19200,38400,57600,115200):
                    raise ValueError('Unsupported baud rate')
                try:
                    import serial
                except ImportError:
                    raise ValueError('Install tools/requirements.txt in the dashboard virtual environment') from None
                device=serial.Serial(port,baud,timeout=0.1,write_timeout=0.25)
            with self.lock:
                self.serial=device
                self.source='SIMULATION' if demo else 'MEGA'
                self.port='demo' if demo else port
                self.baud=baud
                self.connected=True
                self.samples.clear()
                self.latest=None
                self.profile=None
                self.survey=None
                self.survey_samples={}
                self.last_ms=None
                self.demo_mode='DISARMED'
                self.run=self.store.start(self.source,self.port)
                self.started=self.record_start=time.monotonic()
                self.halt.clear()
                self.pending=queue.Queue(maxsize=32)
                self.event('Connected — simulation only' if demo else 'Serial opened; USB may reset the Mega. No motion command sent.')
                self.thread=threading.Thread(target=self.reader,daemon=True)
                self.thread.start()

    def disconnect_unlocked(self):
        self.halt.set()
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=2)
            if self.thread.is_alive():
                raise ValueError('Serial reader is still closing; try again')
        with self.lock:
            if self.serial:
                self.serial.close()
            if self.run:
                self.store.end(self.run)
            self.serial=None
            self.run=None
            self.connected=False
            self.thread=None

    def disconnect(self):
        with self.operations:
            self.disconnect_unlocked()

    def reader(self):
        buffer=bytearray()
        try:
            while not self.halt.is_set():
                if self.source=='SIMULATION':
                    self.simulate()
                    self.halt.wait(0.1)
                else:
                    # Only this worker writes commands, so STOP can jump ahead of queued setup commands.
                    try:
                        command=self.pending.get_nowait()
                        self.serial.write((command+'\n').encode('ascii'))
                        self.serial.flush()
                    except queue.Empty:
                        pass
                    chunk=self.serial.read(512)
                    buffer.extend(chunk)
                    if len(buffer)>16384:
                        buffer.clear()
                        self.event('Oversized serial line discarded')
                    while b'\n' in buffer:
                        raw,_,tail=buffer.partition(b'\n')
                        buffer=bytearray(tail)
                        self.ingest(raw.decode('ascii','replace').strip())
        except Exception as exc:
            self.event('Connection lost: '+str(exc))
        finally:
            with self.lock:
                self.connected=False
                if self.serial:
                    self.serial.close()
                if self.run:
                    self.store.end(self.run)
                    self.run=None

    def record(self, enabled):
        with self.lock:
            if not self.connected:
                raise ValueError('Connect a Mega or start the simulation first')
            if enabled and not self.run:
                self.run=self.store.start(self.source,self.port)
                self.record_start=time.monotonic()
                self.samples.clear()
            elif not enabled and self.run:
                self.store.end(self.run)
                self.run=None

    def command(self, value):
        if not isinstance(value,str) or len(value)>140 or not any(re.fullmatch(p,value) for p in COMMANDS):
            raise ValueError('Command is not supported; see docs/serial_protocol.md')
        with self.lock:
            if not self.connected:
                raise ValueError('No connection')
            if self.source=='SIMULATION':
                if value.startswith('SURVEY '): raise ValueError('Field surveying needs a real Mega; the simulation cannot measure a field')
                if value=='STOP': self.demo_mode='DISARMED'
                elif value=='ARM' and self.demo_mode in ('DISARMED','IDLE'): self.demo_mode='IDLE'
                elif value=='START' and self.demo_mode=='IDLE': self.demo_mode='TO_CENTER';self.demo_start=time.monotonic()
                else: self.event('Simulation: calibration/manual command has no hardware effect')
            elif value=='STOP':
                while not self.pending.empty():
                    try: self.pending.get_nowait()
                    except queue.Empty: break
                self.pending.put_nowait('!STOP')
            else:
                try: self.pending.put_nowait(value)
                except queue.Full: raise ValueError('Command queue full; wait for replies') from None
            self.event('TX '+value)

    def simulate(self):
        with self.lock:
            t=time.monotonic()-self.started
            age=time.monotonic()-self.demo_start
            if self.demo_mode not in ('DISARMED','IDLE','FINISHED'):
                self.demo_mode='TO_CENTER' if age<5 else 'READ_COLOR' if age<7 else 'TURN' if age<11 else 'TO_BOX' if age<16 else 'DOCK' if age<18 else 'FINISHED'
            mode=self.demo_mode
            turn=max(0,min(1,(age-7)/4))*90*(1 if self.demo_green else -1) if mode not in ('DISARMED','IDLE') else 0
            x=300+max(0,5-age)*80 if mode=='TO_CENTER' else 300
            y=max(0,min(1,(age-11)/6))*450*(-1 if self.demo_green else 1) if mode in ('TO_BOX','DOCK','FINISHED') else 0
            color='GREEN' if self.demo_green else 'RED'
            data=dict.fromkeys(FIELDS,0)
            data.update(ms=int(t*1000),mode=mode,armed=int(mode not in ('DISARMED','FINISHED')),left45_mm=420+18*math.sin(t),right45_mm=425+15*math.cos(t),
                        gyro_deg=180+turn,gyro_dps=22.5 if mode=='TURN' else 0,imu_ok=1,light_left=190+20*math.sin(t),light_middle=450+30*math.cos(t),light_right=200+15*math.cos(t),light_ok=1,
                        red=90 if self.demo_green else 320,green=300 if self.demo_green else 100,blue=75,clear=600,color_ok=1,color=color,decision=color if age>=7 and mode not in ('DISARMED','IDLE') else 'UNKNOWN',
                        x_mm=x,y_mm=y,yaw_deg=((180+turn+180)%360)-180,pose_ok=1,heading_ok=1,beacon_residual_mm=8+3*math.sin(t),us_fused=int(mode not in ('TO_CENTER','TO_BOX')),us_residual_mm=3,
                        reference='CENTER' if mode=='READ_COLOR' else 'NONE',target='BOX_LEFT' if self.demo_green else 'BOX_RIGHT',vx=40 if mode in ('TO_CENTER','TO_BOX') else 0,wz=22.5 if mode=='TURN' else 0,
                        calibrated=1,hardware=15,fault='NONE')
        self.ingest(','.join(str(data[k]) for k in FIELDS))

    def status(self, after=0):
        with self.lock:
            age=None if not self.latest else round(time.monotonic()-self.started-self.latest['ms']/1000,2) if self.source=='SIMULATION' else None
            return dict(connected=self.connected,source=self.source,port=self.port,baud=self.baud,recording=self.run,latest=self.latest,profile=self.profile,
                        samples=[s for s in self.samples if s['seq']>after][-1000:],events=list(self.events),sequence=self.seq,age=age,
                        survey=self.survey,survey_samples=[self.survey_samples[i] for i in sorted(self.survey_samples)])


class Handler(BaseHTTPRequestHandler):
    server_version='NontokLocal/1'

    def log_message(self, *_):
        pass

    def valid_origin(self):
        # Local processes may call the API without Origin. Browser requests must originate here.
        host=self.headers.get('Host','')
        allowed={'127.0.0.1:'+str(self.server.server_port),'localhost:'+str(self.server.server_port)}
        origin=self.headers.get('Origin')
        return host in allowed and (origin is None or origin in {'http://'+h for h in allowed})

    def send(self, status, content, mime='application/json; charset=utf-8', download=None):
        raw=json.dumps(content,allow_nan=False).encode() if mime.startswith('application/json') else content
        self.send_response(status)
        self.send_header('Content-Type',mime)
        self.send_header('Content-Length',str(len(raw)))
        if download: self.send_header('Content-Disposition','attachment; filename="'+download+'"')
        self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff')
        self.send_header('Content-Security-Policy',"default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        if not self.valid_origin():
            return self.send(403,{'error':'Local origin required'})
        uri=urlsplit(self.path)
        query=parse_qs(uri.query)
        bridge=self.server.bridge
        try:
            if uri.path=='/api/status':
                return self.send(200,bridge.status(max(0,int(query.get('after',['0'])[0]))))
            if uri.path=='/api/ports':
                try:
                    from serial.tools import list_ports
                    ports=[{'path':p.device,'name':p.description} for p in list_ports.comports()]
                except ImportError:
                    ports=[]
                return self.send(200,{'ports':ports})
            if uri.path=='/api/runs':
                return self.send(200,bridge.store.runs())
            if uri.path=='/api/survey/export.json':
                with bridge.lock:
                    data=dict(survey=bridge.survey,samples=[bridge.survey_samples[i] for i in sorted(bridge.survey_samples)],source=bridge.source,exported_at=utc())
                return self.send(200,data,download='field-survey.json')
            saved=re.fullmatch(r'/api/runs/([a-f0-9]{32})/survey.json',uri.path)
            if saved: return self.send(200,bridge.store.survey(saved[1]),download='survey-'+saved[1]+'.json')
            match=re.fullmatch(r'/api/runs/([a-f0-9]{32})(/export.csv)?',uri.path)
            if match:
                rid=match[1]
                if match[2]:
                    with bridge.store.export(rid) as data:
                        self.send_response(200)
                        self.send_header('Content-Type','text/csv; charset=utf-8')
                        self.send_header('Content-Disposition','attachment; filename="mega-'+rid+'.csv"')
                        self.send_header('X-Content-Type-Options','nosniff')
                        self.end_headers()
                        for chunk in iter(lambda:data.read(65536),''):
                            self.wfile.write(chunk.encode())
                    return
                return self.send(200,bridge.store.data(rid))
            files={'/':'index.html','/app.js':'app.js','/style.css':'style.css','/field-reference.jpg':'field-reference.jpg'}
            if uri.path in files:
                mime={'index.html':'text/html; charset=utf-8','app.js':'text/javascript; charset=utf-8','style.css':'text/css; charset=utf-8','field-reference.jpg':'image/jpeg'}
                name=files[uri.path]
                return self.send(200,(STATIC/name).read_bytes(),mime[name])
            self.send(404,{'error':'Not found'})
        except (ValueError,KeyError) as exc:
            self.send(400,{'error':str(exc)})

    def do_POST(self):
        if not self.valid_origin() or self.headers.get('Content-Type','').split(';')[0]!='application/json':
            return self.send(403,{'error':'Local JSON request required'})
        try:
            size=int(self.headers.get('Content-Length','0'))
            if not 0<size<=2048:
                raise ValueError('Invalid request size')
            data=json.loads(self.rfile.read(size))
            if not isinstance(data,dict): raise ValueError('Expected an object')
            bridge=self.server.bridge
            if self.path=='/api/connect':
                bridge.connect(data.get('port'),int(data.get('baud',115200)))
            elif self.path=='/api/demo':
                bridge.demo_green=data.get('color','GREEN')=='GREEN'
                bridge.connect(None,115200,demo=True)
            elif self.path=='/api/disconnect':
                bridge.disconnect()
            elif self.path=='/api/command':
                bridge.command(data.get('command'))
            elif self.path=='/api/record':
                if type(data.get('enabled')) is not bool: raise ValueError('enabled must be boolean')
                bridge.record(data['enabled'])
            else:
                return self.send(404,{'error':'Not found'})
            self.send(200,{'ok':True})
        except (ValueError,TypeError,OSError) as exc:
            self.send(400,{'error':str(exc)})


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=8765,help='HTTP port; server binds only 127.0.0.1')
    parser.add_argument('--data',type=Path,default=Path(__file__).resolve().parents[1]/'.local-data'/'telemetry.sqlite3')
    args=parser.parse_args()
    args.data.parent.mkdir(parents=True,exist_ok=True)
    store=Store(args.data)
    bridge=Bridge(store)
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    server.bridge=bridge
    print('Dashboard: http://127.0.0.1:'+str(args.port),flush=True)
    print('Recordings: '+str(args.data),flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.disconnect()
        server.server_close()
        store.close()


if __name__=='__main__':
    main()
