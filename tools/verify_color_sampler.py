"""Execute the sampler's AVR instruction subset with cycle-timed input waveforms.

Checks actual assembly source control flow, balanced branch timings and pulse counts.
This does not replace an oscilloscope check on the physical Mega/module.
"""
from pathlib import Path
import math
import re

source=Path(__file__).resolve().parents[1]/'firmware/core_robot_mega/src/color_sampler.S'
body=source.read_text().split('colorPulseBurst:',1)[1].split('.size',1)[0]
ops=[]
labels={}
for raw in body.splitlines():
    line=raw.strip()
    if not line: continue
    if line.endswith(':'): labels[line[:-1]]=len(ops);continue
    op,*args=re.split(r'[\s,]+',line)
    ops.append((op,args))

def execute(signal):
    r=[0]*32
    pc=cycles=0
    z=False
    observations=[]
    while True:
        assert cycles<2000,'Sampler did not terminate'
        op,args=ops[pc]
        pc+=1
        reg=lambda x:int(x[1:])
        if op=='clr': r[reg(args[0])]=0;cycles+=1
        elif op=='ldi': r[reg(args[0])]=int(args[1]);cycles+=1
        elif op=='in':
            assert args[1]=='_SFR_IO_ADDR(PINC)'
            observations.append(cycles)
            # Unrelated port bits vary independently of PC5.
            r[reg(args[0])]=(32 if signal(cycles) else 0)|((cycles*17)&0xdf)
            cycles+=1
        elif op=='nop': cycles+=1
        elif op=='eor': r[reg(args[0])]^=r[reg(args[1])];cycles+=1
        elif op=='and': r[reg(args[0])]&=r[reg(args[1])];cycles+=1
        elif op=='sbrc':
            assert args[1]=='PC5'
            if not (r[reg(args[0])]&32): pc+=1;cycles+=2
            else: cycles+=1
        elif op=='inc': r[reg(args[0])]=(r[reg(args[0])]+1)&255;cycles+=1
        elif op=='mov': r[reg(args[0])]=r[reg(args[1])];cycles+=1
        elif op=='dec':
            k=reg(args[0]);r[k]=(r[k]-1)&255;z=r[k]==0;cycles+=1
        elif op=='brne':
            if not z: pc=labels[args[0]];cycles+=2
            else: cycles+=1
        elif op=='ret': cycles+=4;break
        else: raise AssertionError('Unmodelled instruction '+op)
    assert len(observations)==129
    assert all(b-a==9 for a,b in zip(observations,observations[1:]))
    assert cycles==1166
    return r[24],observations[0],observations[-1]

assert execute(lambda t:False)[0]==0
assert execute(lambda t:True)[0]==0
for hz in (100,500,1000,10000,12345,100000,400000,600000):
    period=16000000/hz
    for duty in (.45,.5,.55):
        for phase in range(31):
            offset=period*phase/31
            count,start,end=execute(lambda t:(t+offset)%period<period*duty)
            # Every continuous-wave rising edge must be counted, across all phases.
            expected=math.floor((end+offset)/period)-math.floor((start+offset)/period)
            assert count==expected,(hz,duty,phase,count,expected)
print('Colour sampler: 9-cycle spacing, 1166-cycle bounded body, stuck inputs and 744 waveform cases passed')
