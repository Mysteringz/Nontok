'use strict';
const $=id=>document.getElementById(id);
const groups={
 ranges:{unit:'mm',fields:['left45_mm','right45_mm'],names:['Left +45°','Right −45°']},
 light:{unit:'ADC',fields:['light_left','light_middle','light_right'],names:['Left beacon','Middle beacon','Right beacon']},
 color:{unit:'raw',fields:['red','green','blue','clear'],names:['Red','Green','Blue','Clear']},
 position:{unit:'mm',fields:['x_mm','y_mm'],names:['World X','World Y']},
 heading:{unit:'°',fields:['gyro_deg','yaw_deg'],names:['Gyroscope','Fused yaw']},
 rate:{unit:'°/s',fields:['gyro_dps','wz'],names:['Measured','Commanded']},
 motion:{unit:'mm/s',fields:['vx','vy'],names:['Forward','Left']},
 residual:{unit:'mm',fields:['beacon_residual_mm','us_residual_mm'],names:['Optical fit','Ultrasonic fit']}
};
const colors=['#087e80','#da8d43','#607ed4','#a0b0bb'];
let state={},history=[],lastSequence=0,paused=false,replay=null,replayEvents=null,replayMeta='',replayProfile=null,lastRecording=null,lastRuns='',lastEvents='',toastTimer,busy=false,lastGood=0;
function fmt(v,d=1){return Number.isFinite(v)?v.toFixed(d):'—';}
function toast(message){$('toast').textContent=message;$('toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').hidden=true,6500);}
async function api(path,data){const response=await fetch(path,data===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});const body=await response.json();if(!response.ok)throw Error(body.error||'Request failed');return body;}
async function action(path,data){try{return await api(path,data);}catch(e){toast(e.message);return null;}}
async function command(value){const result=await action('/api/command',{command:value});if(result)toast('Command queued: '+value+' · check the board reply in the event log');return result;}
function selectedData(){return replay||history;}
function currentSample(){const a=selectedData();return replay?a.at(-1):state.latest;}
function validValue(s,key){
 if((key==='left45_mm'||key==='right45_mm')&&s[key]<0)return false;
 if(['x_mm','y_mm','beacon_residual_mm'].includes(key)&&!s.pose_ok)return false;
 if(key==='yaw_deg'&&!s.heading_ok)return false;
 if(key==='gyro_deg'||key==='gyro_dps')return !!s.imu_ok;
 if(key==='us_residual_mm'&&!s.us_fused)return false;
 if(['light_left','light_middle','light_right'].includes(key)&&!s.light_ok)return false;
 if(['red','green','blue','clear'].includes(key)&&!s.color_ok)return false;
 return Number.isFinite(s[key]);
}
function prepareCanvas(id){const canvas=$(id),box=canvas.getBoundingClientRect(),ratio=window.devicePixelRatio||1;canvas.width=Math.round(box.width*ratio);canvas.height=Math.round(box.height*ratio);const ctx=canvas.getContext('2d');ctx.scale(ratio,ratio);return {ctx,w:box.width,h:box.height};}
function graph(canvasId,group,legendId){
 const {ctx,w,h}=prepareCanvas(canvasId),g=groups[group];
 $(legendId).replaceChildren(...g.names.map((n,i)=>{const el=document.createElement('span'),dot=document.createElement('i');dot.style.background=colors[i];el.append(dot,document.createTextNode(n));return el;}));
 const all=selectedData(),seconds=Number($('window').value),end=all.at(-1)?.t||0,start=seconds?Math.max(all[0]?.t||0,end-seconds):(all[0]?.t||0);
 const data=all.filter(s=>s.t>=start);const values=data.flatMap(s=>g.fields.filter(k=>validValue(s,k)).map(k=>s[k]));
 let min=values.length?Math.min(...values):0,max=values.length?Math.max(...values):100;
 const pad=(max-min)*0.13||10;min-=pad;max+=pad;if(group==='ranges'||group==='light'||group==='color')min=Math.max(0,min);
 const left=48,right=12,top=20,bottom=24,pw=w-left-right,ph=h-top-bottom,span=Math.max(1,end-start);
 ctx.font='10px -apple-system, sans-serif';ctx.lineWidth=1;ctx.fillStyle='#71818c';ctx.fillText(g.unit,4,10);
 for(let i=0;i<=3;i++){const y=top+ph*i/3;ctx.strokeStyle='#e9eef1';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(w-right,y);ctx.stroke();ctx.fillStyle='#81909a';ctx.textAlign='right';ctx.fillText(fmt(max-(max-min)*i/3,Math.abs(max-min)<8?1:0),left-9,y+3);}
 for(let i=0;i<=4;i++){const x=left+pw*i/4;ctx.textAlign='center';ctx.fillText(fmt(start+span*i/4,1)+'s',x,h-4);}
 if(!data.length){ctx.textAlign='center';ctx.fillStyle='#95a3ad';ctx.fillText('Waiting for telemetry',w/2,top+ph/2);return;}
 const stride=Math.max(1,Math.floor(data.length/Math.max(200,pw*2)));
 g.fields.forEach((key,index)=>{ctx.strokeStyle=colors[index];ctx.lineWidth=1.7;ctx.beginPath();let active=false,prevT=null,prevV=null;
 for(let i=0;i<data.length;i+=stride){const s=data[i];if(!validValue(s,key)){active=false;continue;}const x=left+(s.t-start)/span*pw,y=top+(max-s[key])/(max-min)*ph;
 const discontinuity=prevT!==null&&(s.t-prevT>2||s.t<prevT||(key==='yaw_deg'&&Math.abs(s[key]-prevV)>180));
 if(!active||discontinuity)ctx.moveTo(x,y);else ctx.lineTo(x,y);active=true;prevT=s.t;prevV=s[key];}ctx.stroke();
 });
}
function field(){
 const {ctx,w,h}=prepareCanvas('fieldMap'),data=selectedData().filter(s=>s.pose_ok),sample=currentSample(),profile=replay?replayProfile:state.profile,refs=profile?.points||[];
 const positionMask=profile?.beacon_position_mask??2;
 const beacons=(profile?.beacons||[[0,0]]).flatMap((b,i)=>profile?.beacons?((positionMask&(1<<i))?[{x:b[0],y:b[1],name:['L','M','R'][i]}]:[]):[{x:0,y:0,name:'M'}]);
 const map=replay?profile?.survey_map:(state.survey?.map?.valid?state.survey.map:profile?.survey_map);
 const segments=(map?.valid?map.walls:[]).flatMap((wall,i)=>{
  if(!wall.ok||![wall.intercept_mm,wall.slope,wall.from_mm,wall.to_mm].every(Number.isFinite)||Math.abs(wall.from_mm)>10000||Math.abs(wall.to_mm)>10000)return [];
  return [{a:i===0?{x:wall.intercept_mm+wall.slope*wall.from_mm,y:wall.from_mm}:{x:wall.from_mm,y:wall.intercept_mm+wall.slope*wall.from_mm},
           b:i===0?{x:wall.intercept_mm+wall.slope*wall.to_mm,y:wall.to_mm}:{x:wall.to_mm,y:wall.intercept_mm+wall.slope*wall.to_mm}}];
 });
 const extents=[...data,...refs,...beacons,...segments.flatMap(s=>[s.a,s.b])];let xmax=Math.max(1000,...extents.map(s=>s.x_mm??s.x??0)),ymax=Math.max(900,...extents.map(s=>Math.abs(s.y_mm??s.y??0)));
 const scale=Math.min((w-42)/(2*ymax),(h-65)/xmax),sx=y=>w/2-y*scale,sy=x=>h-35-x*scale;
 ctx.strokeStyle='#e8eef1';ctx.lineWidth=1;for(let x=0;x<=xmax;x+=250){ctx.beginPath();ctx.moveTo(12,sy(x));ctx.lineTo(w-12,sy(x));ctx.stroke();}
 ctx.setLineDash([3,4]);ctx.beginPath();ctx.moveTo(w/2,25);ctx.lineTo(w/2,h-12);ctx.stroke();ctx.setLineDash([]);
 ctx.strokeStyle='#607ed4';ctx.lineWidth=2;for(const segment of segments){ctx.beginPath();ctx.moveTo(sx(segment.a.y),sy(segment.a.x));ctx.lineTo(sx(segment.b.y),sy(segment.b.x));ctx.stroke();}
 ctx.font='9px -apple-system,sans-serif';ctx.textAlign='center';
 beacons.forEach(b=>{ctx.fillStyle='#da8d43';ctx.beginPath();ctx.arc(sx(b.y),sy(b.x),4,0,Math.PI*2);ctx.fill();ctx.fillStyle='#71818c';ctx.fillText(b.name,sx(b.y),sy(b.x)+17);});
 if(positionMask!==7){ctx.fillStyle='#71818c';ctx.fillText('Side-light coordinates not measured',w/2,14);}
 ctx.strokeStyle='#087e8055';ctx.lineWidth=2;ctx.beginPath();data.forEach((s,i)=>{if(i===0||s.t-data[i-1].t>2)ctx.moveTo(sx(s.y_mm),sy(s.x_mm));else ctx.lineTo(sx(s.y_mm),sy(s.x_mm));});ctx.stroke();
 const targets=refs.filter(r=>r.samples&1);if(!targets.some(r=>r.name==='CENTER'))targets.push({x:300,y:0,name:'CENTER',samples:0});
 for(const r of targets){ctx.strokeStyle=r.samples?'#607ed4':'#a4b2be';ctx.strokeRect(sx(r.y)-5,sy(r.x)-5,10,10);if(r.name==='CENTER'){ctx.fillStyle='#71818c';ctx.fillText('Centre',sx(r.y)+27,sy(r.x)+3);}}
 if(sample?.pose_ok){ctx.save();ctx.translate(sx(sample.y_mm),sy(sample.x_mm));ctx.rotate(-(sample.yaw_deg||0)*Math.PI/180);ctx.fillStyle='#087e80';ctx.beginPath();ctx.moveTo(0,-11);ctx.lineTo(-6,6);ctx.lineTo(6,6);ctx.closePath();ctx.fill();ctx.restore();}
}
function render(){
 const s=currentSample(),online=state.connected,stale=!s||(!replay&&Date.now()-Date.parse(s.received_at)>1500),sim=state.source==='SIMULATION';
 $('clock').textContent=new Date().toLocaleTimeString('en-GB',{hour12:false});
 $('sourceBadge').textContent=replay?'RECORDING':online?(sim?'SIMULATION · NO HARDWARE':'MEGA CONNECTED'):'OFFLINE';$('sourceBadge').className='badge '+(sim&&online?'sim':online?'connected':'');
 $('recordDot').className='dot'+(state.recording?' active':'');$('recordLabel').textContent=state.recording?(sim?'Recording simulation':'Recording live telemetry'):'Recording paused';
 $('connect').disabled=online;$('disconnect').disabled=!online;$('recordToggle').disabled=!online;$('recordToggle').textContent=state.recording?'Pause recording':'Resume recording';
 $('mode').textContent=s?.mode||'DISCONNECTED';$('dataHealth').textContent=replay?'REPLAY':stale?'STALE / NO DATA':'LIVE';$('dataHealth').className='small-badge '+(stale?'bad':'good');
 $('fault').textContent=s?.fault&&s.fault!=='NONE'?s.fault:s?.mode==='FINISHED'?'Run finished · motors stopped':s?.armed?'Mega armed':'Motors disarmed';$('fault').className='metric-foot '+(s?.fault&&s.fault!=='NONE'?'bad':'');
 $('gyro').textContent=s?.imu_ok?fmt(s.gyro_deg):'—';$('yaw').textContent=s?.heading_ok?fmt(s.yaw_deg):'—';$('rate').textContent=s?.imu_ok?fmt(s.gyro_dps):'—';
 $('leftRange').textContent=s?.left45_mm>=0?fmt(s.left45_mm,0):'—';$('rightRange').textContent=s?.right45_mm>=0?fmt(s.right45_mm,0):'—';
 $('color').textContent=s?.color_ok?s.color:'UNKNOWN';$('decision').textContent=s?.decision||'—';$('colorRaw').textContent=s?.color_ok?`R ${s.red} G ${s.green} B ${s.blue}`:'R — G — B —';$('colorSwatch').style.background=s?.color==='GREEN'?'#25aaa3':s?.color==='RED'?'#bc4545':'#c5ced3';
 $('posX').textContent=s?.pose_ok?fmt(s.x_mm,0):'—';$('posY').textContent=s?.pose_ok?fmt(s.y_mm,0):'—';$('poseBadge').textContent=s?.pose_ok?(s.us_fused?'OPTICAL + US':'OPTICAL'):'NO FIX';
 $('beaconFit').textContent=s?.pose_ok?fmt(s.beacon_residual_mm)+' mm':'Invalid';$('fusion').textContent=s?.us_fused?fmt(s.us_residual_mm)+' mm residual':'Awaiting local reference';$('target').textContent=s?`${s.reference} / ${s.target}`:'—';
 const ready=online&&!stale&&!replay&&s?.calibrated&&s.hardware===15&&s.pose_ok&&s.heading_ok&&s.imu_ok&&s.color_ok&&s.left45_mm>=80&&s.right45_mm>=80;
 $('arm').disabled=!!state.survey?.active||!online||stale||!!replay||!s||!['DISARMED','IDLE'].includes(s.mode)||(s.hardware&1)===0;
 $('start').disabled=!!state.survey?.active||!ready||s?.mode!=='IDLE';$('stop').disabled=!online;
 const missing=[];if(!s||s.hardware!==15)missing.push('hardware confirmation');if(!s?.calibrated)missing.push('calibration');if(state.profile&&state.profile.beacon_position_mask!==7)missing.push('measured beacon coordinates');if(!s?.heading_ok)missing.push('heading reference');if(!s?.pose_ok)missing.push('position fix');if(!s?.color_ok)missing.push('colour sensor');if(stale)missing.push('fresh telemetry');
 $('readiness').textContent=replay?'Viewing a saved recording. Return to live to operate the Mega.':s?.mode==='FINISHED'?'Finished is latched on the Mega. Stop/reset is required before another run.':missing.length?'Still needed: '+missing.join(', ')+'.':'Ready. Arm, then start. The Mega executes the complete mission, including when the dashboard disconnects.';
 const stages=['TO_CENTER','READ_COLOR','TURN','TO_BOX','FINISHED'],stage=stages.indexOf(s?.mode==='DOCK'?'TO_BOX':s?.mode);document.querySelectorAll('.steps li').forEach((el,i)=>el.className=i===stage?'active':stage>i?'done':'');
 $('notice').textContent=replay?replayMeta:sim&&online?'SIMULATION — generated telemetry only. No board is connected or controlled.':online?'USB telemetry is recording locally. Watch for “OK” / “ERR” board replies after commands. Disconnecting leaves Mega autonomy running.':'Connect the Mega over USB, or explore a clearly labelled simulation. Opening USB may reset the board.';
 $('viewTitle').textContent=replay?'Recorded time series':paused?'Live view paused':'Live time series';$('live').hidden=!replay;$('freeze').disabled=!!replay;$('freeze').textContent=paused?'Resume view':'Pause view';$('freeze').setAttribute('aria-pressed',String(paused));
 const ev=replayEvents||state.events||[],signature=JSON.stringify(ev);if(signature!==lastEvents){lastEvents=signature;$('events').replaceChildren(...ev.slice(-50).reverse().map(e=>{const li=document.createElement('li'),t=document.createElement('time');t.textContent=new Date(e.at).toLocaleTimeString('en-GB',{hour12:false});li.append(t,document.createTextNode(e.message));return li;}));}
 $('calibBadge').textContent=s?.calibrated?'COMPLETE':'INCOMPLETE';$('calibBadge').className='badge '+(s?.calibrated?'connected':'');
 document.querySelectorAll('.calibration button').forEach(b=>b.disabled=!online||s?.mode!=='DISARMED'||!!replay||!!state.survey?.active);
 renderCalibration();renderSurvey();
 if(!paused||replay){graph('chartA',$('plotA').value,'legendA');graph('chartB',$('plotB').value,'legendB');field();$('chartNote').textContent=`Time in seconds · ${selectedData().length.toLocaleString()} loaded samples${replay?' · '+replayMeta:''}`;}
}
let calibSignature='';
function renderCalibration(){const p=state.profile,signature=JSON.stringify(p);if(signature===calibSignature)return;calibSignature=signature;const measured=[0,1,2].filter(i=>(p?.beacon_position_mask??2)&(1<<i)).length;$('beaconGeometry').textContent=p?`${measured}/3 beacon positions confirmed by the Mega. Side-wall spacing: ${p.field_width_mm??1500} mm.`:'Side beacon positions await measurement. Read calibration from the Mega.';$('colorCalib').textContent='Colour references: '+(p?.color_mask===3?'red + green captured':p?.color_mask===1?'red only':p?.color_mask===2?'green only':'none');const refs=p?.points||['CENTER','TURN_LEFT','TURN_RIGHT','BOX_LEFT','BOX_RIGHT'].map(name=>({name,samples:0}));$('calibTable').replaceChildren(...refs.map(r=>{const tr=document.createElement('tr');[r.name,...[1,2,4,8].map(bit=>(r.samples&bit)?'Captured':'Needed'),r.samples?`${fmt(r.x,0)}, ${fmt(r.y,0)} / ${fmt(r.yaw)}°`:'—'].forEach((v,i)=>{const td=document.createElement('td');td.textContent=v;if(i>0&&i<5)td.className=v==='Captured'?'good':'muted';tr.append(td);});return tr;}));}
let surveySignature='';
function renderSurvey(){
 const v=state.survey,phase=v?.phase||'NOT STARTED',capturing=['DARK','CAPTURE_POINT','CAPTURE_WALL','SOLVING'].includes(phase);
 const available=state.connected&&state.source==='MEGA'&&state.latest?.mode==='DISARMED'&&!replay;
 $('surveyBadge').textContent=phase.replaceAll('_',' ');$('surveyProgress').textContent=`${v?.count||0} / ${v?.capacity||40} captures`;
 $('surveyMessage').textContent=state.source==='SIMULATION'?'Field measurement requires a real Mega.':v?.message?.replaceAll('_',' ')||'Set measured mounts, then start a survey at a known pose.';
 document.querySelectorAll('.survey-panel button').forEach(b=>{const cmd=b.dataset.survey||'';b.disabled=!available||phase==='SOLVING'||(capturing&&!['SURVEY STATUS','SURVEY CANCEL','SURVEY END'].includes(cmd));});
 $('surveyApply').disabled=!available||capturing||!v?.map?.valid;
 $('surveyDownload').hidden=!v;
 const signature=JSON.stringify(v);if(signature===surveySignature)return;surveySignature=signature;
 $('surveyWidth').textContent=v?.map?.valid?`Measured width ${fmt(v.map.width_mm)} mm · reference 1,500 mm. Segments cover observed surface only.`:'Width awaiting a valid fit. Known reference: 1,500 mm.';
 const walls=v?.map?.walls||[{}, {}, {}],beacons=v?.beacons||[{}, {}, {}];
 function rows(target,items,build){$(target).replaceChildren(...items.map((item,i)=>{const row=document.createElement('tr');for(const [text,cls] of build(item,i)){const cell=document.createElement('td');cell.textContent=text;if(cls)cell.className=cls;row.append(cell);}return row;}));}
 rows('surveyWalls',walls,(w,i)=>[[['Back','+Y side','−Y side'][i]],[w.ok?fmt(w.intercept_mm):'—'],[w.ok?fmt(w.rms_mm):'—'],[String(w.points||0)],[w.ok?'Accepted':'Need data',w.ok?'good':'muted']]);
 rows('surveyBeacons',beacons,(b,i)=>[[['Left','Middle','Right'][i]],[b.ok?`${fmt(b.x_mm)} / ${fmt(b.y_mm)}`:'—'],[b.points?fmt(b.rms_mm):'—'],[b.points?fmt(b.sensitivity_mm):'—'],[b.ok?'Accepted':'Need data',b.ok?'good':'muted']]);
}
async function refreshPorts(){const data=await action('/api/ports');if(!data)return;const prev=$('port').value;$('port').replaceChildren(new Option('Select a device',''),...data.ports.map(p=>new Option(`${p.path} · ${p.name}`,p.path)));if(data.ports.some(p=>p.path===prev))$('port').value=prev;else if(data.ports.length===1)$('port').value=data.ports[0].path;}
async function refreshRuns(){const runs=await action('/api/runs');if(!runs)return;const signature=JSON.stringify(runs);if(signature===lastRuns)return;lastRuns=signature;if(!runs.length)return;$('runs').replaceChildren(...runs.map(r=>{const row=document.createElement('div');row.className='run-row';const info=document.createElement('div');info.className='run-info';const title=document.createElement('strong');title.textContent=new Date(r.started).toLocaleString('en-GB',{hour12:false});const sub=document.createElement('small');sub.textContent=`${r.source} · ${r.count.toLocaleString()} samples · ${r.ended?'Saved':'Recording'}`;info.append(title,sub);const buttons=document.createElement('div');buttons.className='run-actions';const view=document.createElement('button');view.textContent='Replay';view.onclick=async()=>{const data=await action('/api/runs/'+r.id);if(!data)return;replay=data.samples;replayEvents=data.events;replayProfile=null;for(const event of data.events){if(event.message.startsWith('# PROFILE ')){try{replayProfile=JSON.parse(event.message.slice(10));}catch{}}}paused=false;replayMeta=`${r.source} · ${new Date(r.started).toLocaleString()} · ${data.count.toLocaleString()} stored samples${data.stride>1?` · chart stride ${data.stride}`:''}`;lastEvents='';render();$('viewTitle').scrollIntoView({behavior:'smooth',block:'center'});};const download=document.createElement('a');download.href='/api/runs/'+r.id+'/export.csv';download.textContent='CSV ↓';download.setAttribute('download','');buttons.append(view,download);if(r.survey){const surveyLink=document.createElement('a');surveyLink.href='/api/runs/'+r.id+'/survey.json';surveyLink.textContent='Survey ↓';surveyLink.setAttribute('download','');buttons.append(surveyLink);}row.append(info,buttons);return row;}));}
async function poll(){if(busy)return;busy=true;try{state=await api('/api/status?after='+lastSequence);if(state.recording&&state.recording!==lastRecording){history=[];}lastRecording=state.recording;if(state.samples.length){history.push(...state.samples);if(history.length>12000)history.splice(0,history.length-12000);}lastSequence=state.sequence;lastGood=Date.now();render();}catch(e){if(Date.now()-lastGood>3000){$('sourceBadge').textContent='SERVER UNAVAILABLE';$('dataHealth').textContent='STALE';$('start').disabled=$('arm').disabled=true;}}finally{busy=false;}}
$('refreshPorts').onclick=refreshPorts;
$('connect').onclick=async()=>{if(!$('port').value){toast('Select a serial device first.');return;}if(await action('/api/connect',{port:$('port').value,baud:Number($('baud').value)})){history=[];lastSequence=0;replay=null;replayEvents=null;await poll();refreshRuns();}};
$('disconnect').onclick=async()=>{await action('/api/disconnect',{});await poll();refreshRuns();};
$('demo').onclick=async()=>{if(state.connected){toast('Disconnect the current session before starting a simulation.');return;}if(await action('/api/demo',{color:$('demoColor').value})){history=[];lastSequence=0;replay=null;replayEvents=null;await poll();refreshRuns();}};
$('arm').onclick=()=>command('ARM');$('start').onclick=()=>command('START');$('stop').onclick=()=>command('STOP');$('status').onclick=()=>command('STATUS');
$('recordToggle').onclick=async()=>{await action('/api/record',{enabled:!state.recording});await poll();refreshRuns();};
$('freeze').onclick=()=>{paused=!paused;render();};$('live').onclick=()=>{replay=null;replayEvents=null;lastEvents='';render();};
['plotA','plotB','window'].forEach(id=>$(id).onchange=()=>{const old=paused;paused=false;render();paused=old;});
$('headingForm').onsubmit=e=>{e.preventDefault();command('HEADING '+new FormData(e.target).get('heading'));};
$('beaconForm').onsubmit=e=>{e.preventDefault();const d=new FormData(e.target);command('BEACON '+['id','x','y','height','ambient','signal','distance','exponent'].map(k=>d.get(k)).join(' '));};
$('beaconForm').elements.id.onchange=e=>{
 const i=Number(e.target.value),form=$('beaconForm').elements,p=state.profile,b=p?.beacons?.[i],known=(p?.beacon_position_mask??2)&(1<<i);
 form.x.value=known?(b?.[0]??0):'';form.y.value=known?(b?.[1]??0):[750,0,-750][i];
 form.height.value=b&&b[4]>=5?b[2]:'';form.signal.value=b&&b[4]>=5?b[4]:'';
 form.ambient.value=b?.[3]??0;form.distance.value=b?.[5]??1000;form.exponent.value=b?.[6]??2;
};
$('pointForm').onsubmit=e=>{e.preventDefault();const d=new FormData(e.target);command('TEACH '+['point','x','y','yaw'].map(k=>d.get(k)).join(' '));};
$('pointForm').elements.point.onchange=e=>{const r=state.profile?.points?.find(r=>r.name===e.target.value);if(r?.samples){['x','y','yaw'].forEach(k=>$('pointForm').elements[k].value=r[k]);}else if(e.target.value.startsWith('TURN')){const center=state.profile?.points?.[0];$('pointForm').elements.x.value=center?.x||300;$('pointForm').elements.y.value=center?.y||0;let yaw=(center?.yaw??180)+(e.target.value==='TURN_LEFT'?90:-90);$('pointForm').elements.yaw.value=((yaw+180)%360+360)%360-180;}};
$('pointForm').elements.axis.onchange=e=>$('pointForm').elements.offset.value=e.target.value==='YAW'?5:50;
$('sample').onclick=()=>{const d=new FormData($('pointForm'));command(`SAMPLE ${d.get('point')} ${d.get('axis')} ${d.get('offset')}`);};
$('commandForm').onsubmit=e=>{e.preventDefault();command($('command').value.trim());};
 document.querySelectorAll('[data-command]').forEach(b=>b.onclick=()=>command(b.dataset.command));
$('surveyMounts').onsubmit=e=>{e.preventDefault();const d=new FormData(e.target);command('SURVEY MOUNTS '+['lf','ll','rf','rl'].map(k=>d.get(k)).join(' '));};
$('surveyBegin').onsubmit=e=>{e.preventDefault();const d=new FormData(e.target);command('SURVEY BEGIN '+['x','y','yaw'].map(k=>d.get(k)).join(' '));};
$('surveyOptics').onsubmit=e=>{e.preventDefault();const d=new FormData(e.target);command('SURVEY OPTICS '+['hl','hm','hr','n'].map(k=>d.get(k)).join(' '));};
$('surveyPose').onsubmit=e=>{e.preventDefault();const d=new FormData(e.target);command(`SURVEY POSE ${d.get('x')} ${d.get('y')}`);};
$('surveyApply').onclick=async()=>{if(await command('SURVEY APPLY'))await command('CALIB');};
document.querySelectorAll('[data-survey]').forEach(b=>b.onclick=()=>command(b.dataset.survey));
window.addEventListener('resize',()=>{const old=paused;paused=false;render();paused=old;});
refreshPorts();refreshRuns();poll();setInterval(poll,300);setInterval(refreshRuns,4000);
