import importlib.util
import json
import sys
import tempfile
import threading
import time
import unittest
import http.client
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('mega_dashboard',ROOT/'dashboard/server.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
from test_recorder import sample_line


class DashboardTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.path=Path(self.temp.name)/'telemetry.sqlite3'
        self.store=module.Store(self.path)
        self.bridge=module.Bridge(self.store)

    def tearDown(self):
        self.bridge.disconnect()
        self.store.close()
        self.temp.cleanup()

    def test_saved_samples_events_csv_and_restart(self):
        self.bridge.run=self.store.start('MEGA','/dev/test')
        self.bridge.ingest('# READY MEGA3')
        self.bridge.ingest(sample_line(ms=1000))
        self.bridge.ingest(sample_line(ms=20))
        self.bridge.ingest('bad row')
        rid=self.bridge.run
        self.assertEqual(self.store.runs()[0]['count'],2)
        self.assertEqual(len(self.store.data(rid)['samples']),2)
        self.assertIn('restarted',self.store.data(rid)['events'][-1]['message'])
        with self.store.export(rid) as output:
            self.assertEqual(len(output.readlines()),3)
        self.bridge.disconnect()
        self.store.close()
        self.store=module.Store(self.path)
        self.bridge.store=self.store
        self.assertEqual(self.store.runs()[0]['count'],2)
        self.assertIsNotNone(self.store.runs()[0]['ended'])

    def test_downsampling_preserves_ends_export_preserves_all(self):
        rid=self.store.start('MEGA','/dev/test')
        for i in range(37):
            data=module.parse_line(sample_line(ms=i))
            data.update(t=i,received_at=module.utc(),seq=i)
            self.store.sample(rid,data)
        result=self.store.data(rid,max_points=10)
        self.assertEqual(result['count'],37)
        self.assertEqual(result['samples'][0]['ms'],0)
        self.assertEqual(result['samples'][-1]['ms'],36)
        self.assertLessEqual(len(result['samples']),11)
        with self.store.export(rid) as output:
            self.assertEqual(len(output.readlines()),38)

    def test_commands_are_explicit_bounded_and_stop_has_priority(self):
        self.bridge.connected=True
        self.bridge.source='MEGA'
        for command in ('ARM','START','TEACH CENTER 300 0 180','SAMPLE CENTER YAW 5','BEACON 0 0 750 0 10 50 1000 2','SURVEY BEGIN 300 0 180','SURVEY MOUNTS 90 70 90 -70','SURVEY TAKE WALL','SURVEY SOLVE'):
            self.bridge.command(command)
        for command in ('ARM\nSTART','rm -rf /','TEACH CENTER nan 0 1','DRIVE 100 0 0 999999'):
            with self.assertRaises(ValueError): self.bridge.command(command)
        self.bridge.command('STOP')
        self.assertEqual(self.bridge.pending.get_nowait(),'!STOP')
        self.assertTrue(self.bridge.pending.empty())

    def test_profile_and_simulation_are_separate_from_real_hardware(self):
        self.bridge.ingest('# PROFILE {"valid": 0, "points": [], "beacon_position_mask": 2, "field_width_mm": 1500}')
        self.assertEqual(self.bridge.profile['valid'],0)
        self.assertEqual(self.bridge.profile['beacon_position_mask'],2)
        self.assertEqual(self.bridge.profile['field_width_mm'],1500)
        self.bridge.connect(None,115200,demo=True)
        self.bridge.command('ARM')
        self.bridge.command('START')
        self.bridge.demo_start=time.monotonic()-19
        self.bridge.simulate()
        status=self.bridge.status()
        self.assertEqual(status['source'],'SIMULATION')
        self.assertEqual(status['latest']['mode'],'FINISHED')
        self.assertEqual(status['latest']['armed'],0)
        self.assertIsNone(self.bridge.serial)
        with self.assertRaises(ValueError): self.bridge.command('SURVEY SOLVE')

    def test_survey_status_samples_persist_and_export(self):
        self.bridge.run=self.store.start('MEGA','/dev/test')
        self.bridge.ingest('# SURVEY {"phase":"READY","active":1,"count":0,"map":{"valid":0}}')
        self.bridge.ingest('# SURVEY_SAMPLE {"index":0,"x_mm":600,"y_mm":0,"yaw_deg":180,"optical":0,"ranges":[400,410],"lights":[100,200,300]}')
        self.bridge.ingest('# SURVEY {"phase":"READY","active":1,"count":1,"map":{"valid":1,"width_mm":1498}}')
        self.bridge.ingest('# SURVEY {"phase":"CAPTURE_WALL","active":1,"count":1}')
        self.assertEqual(self.bridge.status()['survey']['map']['width_mm'],1498)
        self.assertEqual(len(self.bridge.status()['survey_samples']),1)
        rid=self.bridge.run
        self.assertEqual(self.store.runs()[0]['survey'],1)
        saved=self.store.survey(rid)
        self.assertEqual(saved['samples'][0]['ranges'],[400,410])
        self.assertEqual(saved['survey']['phase'],'CAPTURE_WALL')
        self.assertEqual(saved['survey']['map']['width_mm'],1498)
        self.bridge.ingest('# SURVEY_SAMPLE {"index":99}')
        self.assertEqual(len(self.bridge.survey_samples),1)
        self.bridge.ingest('# SURVEY {"phase":"READY","count":0}')
        self.assertEqual(self.bridge.status()['survey_samples'],[])
        self.assertEqual(self.store.survey(rid)['samples'],[])

    def test_real_serial_worker_with_virtual_board(self):
        try:
            import serial
        except ImportError:
            self.skipTest('pyserial is optional; run in the dashboard environment for the PTY test')
        import os
        import select
        master,slave=os.openpty()
        try:
            path=os.ttyname(slave)
            self.bridge.connect(path,115200)
            os.write(master,(sample_line(ms=200)+'\n').encode())
            deadline=time.monotonic()+2
            while not self.bridge.latest and time.monotonic()<deadline:
                time.sleep(.01)
            self.assertEqual(self.bridge.latest['ms'],200)
            self.assertEqual(self.bridge.source,'MEGA')
            self.bridge.command('ARM')
            ready,_,_=select.select([master],[],[],2)
            self.assertTrue(ready)
            self.assertIn(b'ARM\n',os.read(master,1024))
            self.bridge.command('STOP')
            ready,_,_=select.select([master],[],[],2)
            self.assertTrue(ready)
            self.assertIn(b'!STOP\n',os.read(master,1024))
        finally:
            self.bridge.disconnect()
            os.close(master)
            os.close(slave)


class HTTPTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.store=module.Store(Path(self.temp.name)/'test.sqlite3')
        self.bridge=module.Bridge(self.store)
        self.server=module.ThreadingHTTPServer(('127.0.0.1',0),module.Handler)
        self.server.bridge=self.bridge
        self.worker=threading.Thread(target=self.server.serve_forever,daemon=True)
        self.worker.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.bridge.disconnect()
        self.store.close()
        self.temp.cleanup()

    def request(self,path,body=None,origin=None):
        client=http.client.HTTPConnection('127.0.0.1',self.server.server_port,timeout=3)
        headers={'Content-Type':'application/json'}
        if origin: headers['Origin']=origin
        client.request('GET' if body is None else 'POST',path,body=None if body is None else json.dumps(body),headers=headers)
        response=client.getresponse()
        content=response.read()
        status=response.status
        client.close()
        return status,content

    def test_complete_local_flow_and_origin_checks(self):
        self.assertEqual(self.request('/')[0],200)
        status,photo=self.request('/field-reference.jpg')
        self.assertEqual(status,200)
        self.assertTrue(photo.startswith(b'\xff\xd8'))
        self.assertEqual(self.request('/api/demo',{},origin='https://untrusted.example')[0],403)
        self.assertEqual(self.request('/api/demo',{'color':'RED'})[0],200)
        self.assertEqual(self.request('/api/command',{'command':'ARM'})[0],200)
        self.assertEqual(self.request('/api/command',{'command':'START'})[0],200)
        self.bridge.demo_start=time.monotonic()-19
        self.bridge.simulate()
        status,raw=self.request('/api/status')
        self.assertEqual(status,200)
        self.assertEqual(json.loads(raw)['latest']['decision'],'RED')
        runs=json.loads(self.request('/api/runs')[1])
        self.assertEqual(runs[0]['source'],'SIMULATION')
        self.assertIn(b'received_at,elapsed_s',self.request('/api/runs/'+runs[0]['id']+'/export.csv')[1])
        self.assertEqual(self.request('/api/command',{'command':'ARM\nSTART'})[0],400)
        self.assertEqual(self.request('/api/record',{'enabled':'false'})[0],400)
        self.assertEqual(self.request('/../../README.md')[0],404)
        self.bridge.ingest('# SURVEY {"phase":"READY","count":0}')
        self.bridge.ingest('# SURVEY_SAMPLE {"index":0,"x_mm":300,"y_mm":0}')
        self.bridge.ingest('# SURVEY {"phase":"READY","count":1}')
        export=json.loads(self.request('/api/survey/export.json')[1])
        self.assertEqual(len(export['samples']),1)
        saved=json.loads(self.request('/api/runs/'+runs[0]['id']+'/survey.json')[1])
        self.assertEqual(saved['samples'][0]['x_mm'],300)
        self.assertEqual(self.request('/api/disconnect',{})[0],200)

if __name__=='__main__':
    unittest.main()
