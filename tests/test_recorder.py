import csv
import io
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from record_telemetry import FIELDS, record
from telemetry_schema import parse_line


def sample_line(**overrides):
    values=dict.fromkeys(FIELDS,0)
    values.update(ms=120,mode='IDLE',color='UNKNOWN',decision='UNKNOWN',reference='NONE',target='NONE',fault='NONE')
    values.update(overrides)
    return ','.join(str(values[k]) for k in FIELDS)


class RecorderTest(unittest.TestCase):
    def test_ignores_resets_commands_and_damaged_lines(self):
        data=iter([b'# READY MEGA3\n',(','.join(FIELDS)+'\n').encode(),b'garbled\n',
                   (sample_line()+'\n').encode(),(sample_line(x_mm='nan')+'\n').encode()])
        class Input:
            def readline(self):
                try: return next(data)
                except StopIteration: raise KeyboardInterrupt
        output=io.StringIO()
        self.assertEqual(record(Input(),output),1)
        rows=list(csv.reader(io.StringIO(output.getvalue())))
        self.assertEqual(rows[0],FIELDS)
        self.assertEqual(len(rows),2)

    def test_schema_rejects_nonfinite_invalid_flags_and_modes(self):
        for overrides in ({'mode':'MADE_UP'},{'armed':3},{'gyro_deg':'inf'},{'ms':2**32},{'reference':'INVALID'}):
            self.assertIsNone(parse_line(sample_line(**overrides)))
        self.assertEqual(parse_line(sample_line(left45_mm=-1))['left45_mm'],-1)

if __name__=='__main__':
    unittest.main()
