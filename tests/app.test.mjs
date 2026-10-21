import {test} from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import {readFileSync} from 'node:fs';
import * as model from '../webapp/dist/model.js';
import {Player,preflight} from '../webapp/dist/player.js';
import {SimulatedLink} from '../webapp/dist/simulator.js';
import {SceneController} from '../webapp/src/scene-controller.js';
import {balance} from '../webapp/src/motion-geometry.js';
function setup({beforeSend=async()=>{},savedStorage=new Map(),storageFails=false,boardExtra={}}={}) {
 const links=[],elements=new Map(),all=[],commands=[],storage=savedStorage,downloads=[];let nextTimer=0,scene;const rafs=new Map();
 class Element {
  constructor(tag='div'){this.tag=tag;this.value='';this.disabled=false;this.dataset={};this.className='';this.children=[];this.textContent='';this.classList={toggle:(name,on)=>{const set=new Set(this.className.split(' ').filter(Boolean));const enabled=on??!set.has(name);if(enabled)set.add(name);else set.delete(name);this.className=[...set].join(' ');},add:name=>this.classList.toggle(name,true),remove:name=>this.classList.toggle(name,false)};all.push(this);}
  set id(value){this._id=value;elements.set(value,this);}get id(){return this._id;}
  append(...items){this.children.push(...items);}replaceChildren(...items){this.children=[...items];}
  setAttribute(){}click(){}
 }
 const get=id=>{if(!elements.has(id)){const element=new Element();element.id=id;}return elements.get(id);};
 for(const [id,value] of Object.entries({stride:15,lift:12,height:70,duration:700}))get(id).value=String(value);
 const docListeners={};const document={addEventListener:(type,fn)=>{docListeners[type]=fn;},hidden:false,getElementById:get,createElement:tag=>new Element(tag),querySelectorAll:selector=>all.filter(element=>selector.split(',').some(s=>s.startsWith('#')?element.id===s.slice(1):s.startsWith('.')?element.className.split(' ').includes(s.slice(1)):s==='[data-motion]'?!!element.dataset.motion:false))};
 let board={active:false,moving:false,positions:null,offsets:model.clone(model.DEFAULT_OFFSETS),limits:model.clone(model.LIMITS),joint_limits:model.clone(model.JOINT_LIMITS),calibration:{leg:null,holding:false,reference:[0,175,70],max_adjustment:15,targets:null},...boardExtra};
 class Link {
  constructor(event,lost){this.event=event;this.lost=lost;links.push(this);}
  async open(){return model.clone(board);}async close(){}
  async send(op,args={},guard=()=>true){if(!guard())throw new Error('cancelled');commands.push({op,...args});await beforeSend(op);if(op==='arm'){board.active=true;board.positions=model.home();}if(op==='pose')board.positions=model.clone(args.positions);if(op==='off')board.active=false;return model.clone(board);}
 }
 const context={...model,Player,preflight,SerialLink:Link,SimulatedLink,balance,mountMotionScene:(container,options)=>{scene=new SceneController({...options,request:cb=>{const id=++nextTimer;rafs.set(id,cb);return id;},cancel:id=>rafs.delete(id)});return scene;},document,navigator:{serial:{}},window:{addEventListener(){}},localStorage:{getItem:key=>storage.get(key)||null,setItem:(key,value)=>{if(storageFails)throw new Error('Storage full');storage.set(key,value);}},setInterval(){},setTimeout:()=>++nextTimer,clearTimeout(){},Date,console,Blob,URL:{createObjectURL:blob=>{downloads.push(blob);return 'blob:test';},revokeObjectURL(){}}};
 const source=readFileSync(new URL('../webapp/dist/app.js',import.meta.url),'utf8').replace(/^import .*;\n/gm,'');
 vm.createContext(context);vm.runInContext(source,context);return {context,document,docListeners,links,get,all,commands,storage,downloads,scene,advance:time=>{const entries=[...rafs];rafs.clear();for(const [id,callback] of entries)callback(time);}};
}
test('offline editor builds six motions and exports pose edits',async()=>{
 const {get,all,commands,downloads}=setup();
 assert.equal(get('timeline').children.length,20);assert.equal(all.filter(el=>el.dataset.motion).length,8);assert.equal(get('xyz-0-1').value,175);
 get('xyz-0-0').value='10';get('xyz-0-0').onchange();get('export').onclick();const exported=JSON.parse(await downloads.at(-1).text());
 assert.equal(exported.frames[0].positions[0][0],10);assert.equal(exported.frames[0].angles.length,6);assert.equal(commands.length,0);
});
test('duplicate, remove and local save preserve the edited sequence',()=>{
 const {get,storage}=setup();get('duplicate').onclick();assert.equal(get('timeline').children.length,21);get('remove').onclick();assert.equal(get('timeline').children.length,20);
 get('program-name').value='My taught walk';get('program-name').onchange();get('save').onclick();const library=JSON.parse(storage.get('hexapod-motion-library-v1'));assert.equal(library['My taught walk'].frames.length,20);
});
test('hardware playback requires enable and sends exactly one sequence',async()=>{
 const {get,commands}=setup();assert.equal(get('play').disabled,true);await get('connect').onclick();assert.equal(get('play').disabled,true);await get('arm').onclick();assert.equal(get('play').disabled,false);
 await get('play').onclick();assert.equal(commands.filter(c=>c.op==='pose').length,20);assert.equal(get('hardware-state').textContent,'Motors enabled · holding pose');await get('off').onclick();assert.equal(get('play').disabled,true);assert.equal(get('hardware-state').textContent,'Motors disabled');
});

test('React preview clock interpolates without USB and supports pause/resume',()=>{
 const {get,commands,scene,advance}=setup();
 get('preview').onclick();advance(0);advance(700);advance(1050);
 assert.equal(scene.getSnapshot().pose[0][2],64);
 assert.equal(get('preview').textContent,'Ⅱ Pause preview');
 get('preview').onclick();assert.equal(scene.getSnapshot().playback,'paused');
 const held=scene.getSnapshot().elapsed;advance(9000);assert.equal(scene.getSnapshot().elapsed,held);
 get('preview').onclick();advance(10000);advance(10100);
 assert.equal(scene.getSnapshot().elapsed,held+100);
 assert.equal(commands.length,0);
});

test('pending pause locks playback and finishes at the acknowledged pose',async()=>{
 let release;
 const {get,commands,scene}=setup({beforeSend:op=>op==='pause'?new Promise(resolve=>{release=resolve;}):Promise.resolve()});
 await get('connect').onclick();await get('arm').onclick();
 const stopping=get('pause').onclick();
 assert.equal(get('play').disabled,true);
 assert.equal(get('preview').disabled,true);
 await get('play').onclick();get('preview').onclick();
 assert.equal(commands.filter(c=>c.op==='pose').length,0);
 assert.notEqual(scene.getSnapshot().playback,'playing');
 release();await stopping;
 assert.equal(get('play').disabled,false);
 assert.deepEqual(scene.getSnapshot().pose,model.home());
});
test('file import finishing during playback cannot replace the active timeline',async()=>{
 let finishRead,finishPose;
 const {get}=setup({beforeSend:op=>op==='pose'?new Promise(resolve=>{finishPose=resolve;}):Promise.resolve()});
 await get('connect').onclick();await get('arm').onclick();
 const imported=model.generate('backward');imported.name='Unexpected replacement';
 const target={files:[{size:100,text:()=>new Promise(resolve=>{finishRead=resolve;})}],value:'file'};
 const importing=get('import').onchange({target});
 const playing=get('play').onclick();
 finishRead(JSON.stringify(imported));await importing;
 assert.notEqual(get('program-name').value,'Unexpected replacement');
 assert.match(get('notice').textContent,/Stop playback/);
 await get('pause').onclick();finishPose();await playing;
});

test('posture templates preview offline and rebuild without becoming a walk',()=>{
 const {get,all,commands,scene,advance}=setup();
 for(const [id,height] of [['sit',30],['stand',70]]){
  all.find(el=>el.dataset.motion===id).onclick();
  assert.equal(get('timeline').children.length,2);
  get('generate').onclick();assert.equal(get('timeline').children.length,2);
  get('preview').onclick();advance(0);advance(2500);
  assert.deepEqual(scene.getSnapshot().pose,model.home(height));
 }
 assert.equal(commands.length,0);
});
test('Stand and Sit require enable and send only the destination pose',async()=>{
 const {get,commands}=setup();
 assert.equal(get('stand').disabled,true);assert.equal(get('sit').disabled,true);
 await get('sit').onclick();assert.equal(commands.length,0);
 await get('connect').onclick();await get('stand').onclick();
 assert.equal(commands.filter(c=>c.op==='pose').length,0);
 await get('arm').onclick();
 await get('sit').onclick();await get('stand').onclick();
 const poses=commands.filter(c=>c.op==='pose');assert.equal(poses.length,2);
 assert.deepEqual(poses[0].positions,model.home(30));assert.deepEqual(poses[1].positions,model.home(70));
 assert.equal(poses[0].duration_ms,2000);
 assert.equal(commands.filter(c=>c.op==='off').length,0);
});

test('disconnect locks Stand, Sit and reconnect until shutdown is acknowledged',async()=>{
 let release;
 const {get,commands,links}=setup({beforeSend:op=>op==='off'?new Promise(resolve=>{release=resolve;}):Promise.resolve()});
 await get('connect').onclick();await get('arm').onclick();
 const closing=get('connect').onclick();
 assert.equal(get('stand').disabled,true);
 assert.equal(get('sit').disabled,true);
 assert.equal(get('connect').disabled,true);
 await get('stand').onclick();await get('connect').onclick();
 assert.equal(commands.filter(c=>c.op==='pose').length,0);
 assert.equal(links.length,1);
 release();await closing;
 assert.equal(get('connection').textContent,'Offline planning');
 assert.equal(get('connect').disabled,false);
});
test('callbacks from a closed USB connection do not affect a new connection',async()=>{
 const {get,links}=setup();
 await get('connect').onclick();const old=links[0];
 await get('connect').onclick();await get('connect').onclick();await get('arm').onclick();
 old.event({event:'timeout',state:{active:false,moving:false}});
 assert.equal(get('stand').disabled,false);
 old.lost(new Error('Old port closed'));await Promise.resolve();
 assert.equal(get('connection').textContent,'Servo 2040 connected');
});

test('undo and redo restore edits and template replacements without commands',()=>{
 const {get,all,commands}=setup();
 get('xyz-0-0').value='10';get('xyz-0-0').onchange();
 all.find(el=>el.dataset.motion==='sit').onclick();assert.equal(get('timeline').children.length,2);
 get('undo').onclick();assert.equal(get('timeline').children.length,20);assert.equal(get('xyz-0-0').value,10);
 get('undo').onclick();assert.equal(get('xyz-0-0').value,0);assert.equal(get('undo').disabled,true);
 get('redo').onclick();assert.equal(get('xyz-0-0').value,10);
 get('xyz-0-0').value='12';get('xyz-0-0').onchange();assert.equal(get('redo').disabled,true);
 assert.equal(commands.length,0);
});
test('draft recovers after reload and storage failure does not prevent undo',()=>{
 const first=setup();first.get('xyz-0-0').value='11';first.get('xyz-0-0').onchange();
 const recovered=setup({savedStorage:first.storage});
 assert.equal(recovered.get('xyz-0-0').value,11);
 assert.match(recovered.get('draft-status').textContent,/Recovered/);assert.equal(recovered.commands.length,0);
 const failed=setup({storageFails:true});failed.get('xyz-0-0').value='8';failed.get('xyz-0-0').onchange();
 assert.equal(failed.get('xyz-0-0').value,8);assert.match(failed.get('draft-status').textContent,/unavailable/);
 failed.get('undo').onclick();assert.equal(failed.get('xyz-0-0').value,0);
});
test('corrupt draft is rejected without preventing normal editing',()=>{
 const {get,commands}=setup({savedStorage:new Map([['hexapod-motion-draft-v1','{bad']])});
 assert.equal(get('timeline').children.length,20);assert.match(get('draft-status').textContent,/could not be recovered/);
 get('duplicate').onclick();assert.equal(get('timeline').children.length,21);assert.equal(commands.length,0);
});
test('posture settings persist and control templates and physical destinations',async()=>{
 const {get,all,commands,storage}=setup();
 for(const [id,value] of [['stand-height',85],['sit-height',45],['posture-duration',3000]]){get(id).value=String(value);get(id).onchange();}
 assert.equal(commands.length,0);
 all.find(el=>el.dataset.motion==='sit').onclick();get('timeline').children[1].onclick();
 assert.equal(get('xyz-0-2').value,45);assert.equal(get('duration').value,3000);
 await get('connect').onclick();await get('arm').onclick();await get('stand').onclick();
 const sent=commands.filter(c=>c.op==='pose').at(-1);assert.deepEqual(sent.positions,model.home(85));assert.equal(sent.duration_ms,3000);
 const recovered=setup({savedStorage:storage});assert.equal(recovered.get('stand-height').value,85);
 assert.equal(recovered.get('sit-height').value,45);assert.equal(recovered.get('posture-duration').value,3000);
});
test('invalid settings are reverted and do not send commands',()=>{
 const {get,commands}=setup();
 for(const [id,value] of [['sit-height',90],['stand-height',500],['posture-duration',0]]){
  get(id).value=String(value);get(id).onchange();assert.notEqual(get(id).value,value);
 }
 assert.equal(commands.length,0);assert.equal(get('sit-height').value,30);
});
test('reported pose stays separate from edited targets and clears on disconnect',async()=>{
 const {get}=setup();get('xyz-0-0').value='10';get('xyz-0-0').onchange();
 await get('connect').onclick();await get('arm').onclick();await get('sit').onclick();
 assert.equal(get('xyz-0-0').value,10);assert.equal(get('xyz-0-2').value,70);
 assert.equal(get('reported-values').children.length,6);
 assert.equal(get('reported-values').children[0].children[3].textContent,'30.0');
 assert.equal(get('scene-source').textContent,'BOARD-REPORTED COMMAND');
 await get('connect').onclick();assert.equal(get('reported-values').children.length,0);
 assert.match(get('reported-status').textContent,/No board/);
});

test('connect adopts the board limits and warns when an old board reports none',async()=>{
 const defaults=[model.clone(model.LIMITS),model.clone(model.JOINT_LIMITS)];
 try{
  const narrow={limits:[[-50,50],[100,250],[20,150]],joint_limits:[[-80,80],[-90,90],[-90,90]]};
  const {get}=setup({boardExtra:narrow});await get('connect').onclick();
  assert.deepEqual(model.LIMITS,narrow.limits);assert.deepEqual(model.JOINT_LIMITS,narrow.joint_limits);
  assert.equal(get('xyz-0-0').min,-50);assert.equal(get('xyz-0-2').max,150);
  assert.doesNotMatch(get('notice').textContent,/out of date/);
  model.setBoardLimits(...defaults);
  const old=setup({boardExtra:{limits:undefined,joint_limits:undefined}});await old.get('connect').onclick();
  assert.match(old.get('notice').textContent,/out of date/);assert.deepEqual(model.LIMITS,defaults[0]);
 }finally{model.setBoardLimits(...defaults);}
});
test('join templates and saved movements to the end of the sequence',()=>{
 const {get,all,commands}=setup();
 get('join-template').checked=true;all.find(el=>el.dataset.motion==='stand').onclick();
 assert.equal(get('timeline').children.length,21);assert.equal(get('pose-title').textContent,'Stand');
 get('program-name').value='Walk then stand';get('program-name').onchange();get('save').onclick();
 get('new-sequence').onclick();get('library').value='Walk then stand';get('join-saved').onclick();
 assert.equal(get('timeline').children.length,22);assert.equal(commands.length,0);
 get('undo').onclick();assert.equal(get('timeline').children.length,1);
});
test('playback repeat, gait speed and easing reach the board commands',async()=>{
 const {get,commands}=setup();await get('connect').onclick();await get('arm').onclick();
 get('repeat-count').value='2';get('gait-speed').value='2';get('ease-smooth').checked=true;
 await get('play').onclick();
 const poses=commands.filter(c=>c.op==='pose');assert.equal(poses.length,40);
 assert.ok(poses.every(c=>c.ease==='smooth'&&c.duration_ms===350));
 assert.match(get('notice').textContent,/Playback stopped or completed/);
});
test('jog a foot, save it as Stand trim, then Stand uses the trim',async()=>{
 const {get,commands}=setup();await get('connect').onclick();await get('arm').onclick();
 get('jog-leg').value='4';get('jog-step').value='5';
 await get('jog-down').onclick();await get('jog-down').onclick();
 const jog=commands.filter(c=>c.op==='pose').at(-1);assert.equal(jog.positions[4][2],80);assert.equal(jog.duration_ms,500);assert.ok(!('ease' in jog));
 get('save-stand-trim').onclick();assert.equal(get('stand-trim-4').value,10);
 await get('stand').onclick();assert.deepEqual(commands.filter(c=>c.op==='pose').at(-1).positions.map(p=>p[2]),[70,70,70,70,80,70]);
});
test('servo test, joint-limit save and leg release send board commands',async()=>{
 const {get,commands}=setup();await get('connect').onclick();await get('arm').onclick();
 get('limit-0-min').value='-100';get('limit-0-max').value='100';await get('save-limits').onclick();
 assert.equal(JSON.stringify(commands.at(-1)),JSON.stringify({op:'set_joint_limits',joint_limits:[[-100,100],[-180,180],[-180,180]]}));
 get('limit-1-min').value='50';get('limit-1-max').value='10';await get('save-limits').onclick();
 assert.equal(commands.at(-1).op,'set_joint_limits');assert.match(get('notice').textContent,/minimum below/);
 get('release-leg').value='3';await get('release-leg-button').onclick();
 assert.deepEqual(commands.at(-1),{op:'release_leg',leg:3});
});
test('simulate robot connects without USB and plays a pose',async()=>{
 const {get}=setup();await get('simulate').onclick();
 assert.equal(get('connection').textContent,'Simulated robot');assert.equal(get('simulate').disabled,true);
 assert.equal(get('connect').textContent,'Stop simulation');
 await get('arm').onclick();assert.match(get('hardware-state').textContent,/^Simulation · Motors enabled/);
 await get('send').onclick();assert.match(get('notice').textContent,/Playback stopped or completed/);
 await get('connect').onclick();assert.equal(get('connection').textContent,'Offline planning');
});
test('leg calibration tab adjusts, holds and saves through the shared connection',async()=>{
 const {get,downloads}=setup();
 get('tab-calibrate').onclick();assert.match(get('tab-calibrate').className,/active/);assert.doesNotMatch(get('tab-motion').className,/active/);
 assert.equal(get('cal-leg-0').disabled,true,'needs a connection');
 await get('simulate').onclick();assert.equal(get('cal-leg-0').disabled,false);
 await get('cal-leg-1').onclick();assert.match(get('cal-mode').textContent,/PREVIEW/);assert.equal(get('cal-selected-name').textContent,'Left leg 2');
 const start=Number(get('cal-angle-2').textContent.replace('°',''));
 await get('cal-plus-2').onclick();assert.equal(Number(get('cal-angle-2').textContent.replace('°','')),start+1);
 assert.match(get('cal-offset-2').textContent,/Offset \+5\.0°/);assert.match(get('cal-save-label').textContent,/unsaved/);
 get('cal-range-2').value=String(start+4);await get('cal-range-2').onchange({target:get('cal-range-2')});
 assert.equal(Number(get('cal-angle-2').textContent.replace('°','')),start+4);
 await get('cal-move').onclick();assert.match(get('cal-mode').textContent,/HOLDING/);assert.equal(get('cal-move').disabled,true);
 assert.equal(get('send').disabled,true,'motion needs Enable at home after calibration');
 await get('cal-save').onclick();assert.match(get('cal-save-label').textContent,/Ready/);
 get('cal-download').onclick();const file=JSON.parse(await downloads.at(-1).text());assert.ok(Math.abs(file.L2o[2]-8)<0.1,'slider sends the change from the exact target');
 await get('cal-off').onclick();assert.match(get('cal-mode').textContent,/PREVIEW/);
 get('tab-motion').onclick();assert.match(get('tab-motion').className,/active/);
});
test('calibration tab explains why legs are locked',async()=>{
 const {get}=setup({boardExtra:{calibration:undefined}});get('tab-calibrate').onclick();
 assert.match(get('cal-channels').textContent,/Click Connect robot/);
 await get('connect').onclick();
 assert.equal(get('cal-leg-0').disabled,true);assert.match(get('cal-channels').textContent,/too old for calibration/);
 assert.match(get('notice').textContent,/out of date/);
});

test('apply one transition time to every pose, undoably',()=>{
 const {get,commands}=setup();
 get('all-duration').value='2500';get('apply-all-duration').onclick();
 assert.equal(get('duration').value,2500);assert.match(get('total').textContent,/50\.0 SECONDS/);
 get('all-duration').value='50';get('apply-all-duration').onclick();assert.match(get('notice').textContent,/100–10000/);
 get('undo').onclick();assert.equal(get('duration').value,700);assert.equal(commands.length,0);
});
test('battery voltage shows in the header and warns once when low',async()=>{
 const {get}=setup({boardExtra:{power:{voltage:7.42,current:1.234}}});
 assert.equal(get('battery').textContent,'Battery · not connected');
 await get('connect').onclick();assert.equal(get('battery').textContent,'Battery 7.42 V · 1.23 A');
 get('battery-warn').value='7.5';get('battery-warn').onchange();
 assert.match(get('battery').textContent,/LOW/);assert.match(get('battery').className,/low/);assert.match(get('notice').textContent,/Low battery: 7\.42 V/);
 get('battery-warn').value='';get('battery-warn').onchange();assert.doesNotMatch(get('battery').className,/low/);
 const old=setup({boardExtra:{power:undefined}});await old.get('connect').onclick();assert.match(old.get('battery').textContent,/not reported/);
});

test('Stand trim is only saved while the robot is standing',async()=>{
 const {get,commands}=setup();await get('connect').onclick();await get('arm').onclick();
 get('stand-height').value='90';get('stand-height').onchange();
 get('jog-leg').value='2';await get('jog-down').onclick();get('save-stand-trim').onclick();
 assert.match(get('notice').textContent,/Press Stand first/);assert.equal(get('stand-trim-2').value,0);
 await get('stand').onclick();await get('jog-down').onclick();get('save-stand-trim').onclick();
 assert.equal(get('stand-trim-2').value,2);assert.equal(commands.filter(c=>c.op==='pose').at(-1).positions[2][2],92);
});

test('hip jog rotates the selected foot and the template uses hip swing',async()=>{
 const {get,commands,all}=setup();await get('connect').onclick();await get('arm').onclick();
 get('jog-leg').value='1';get('jog-hip-step').value='10';
 await get('jog-hip-plus').onclick();await get('jog-hip-plus').onclick();
 const foot=commands.filter(c=>c.op==='pose').at(-1).positions[1];
 assert.ok(Math.abs(Math.atan2(foot[0],foot[1])*180/Math.PI-20)<1e-9);assert.ok(Math.abs(Math.hypot(foot[0],foot[1])-175)<1e-9);
 assert.match(get('notice').textContent,/L2 hip at/);
 await get('off').onclick();
 get('hip-swing').value='45';all.find(el=>el.dataset.motion==='turn-left').onclick();
 get('timeline').children[2].onclick();assert.ok(Math.abs(Math.atan2(Number(get('xyz-0-0').value),Number(get('xyz-0-1').value))*180/Math.PI-45)<0.01);
});

test('gait selector builds a tripod template',()=>{
 const {get,all}=setup();get('gait').value='tripod';all.find(el=>el.dataset.motion==='forward').onclick();
 assert.equal(get('timeline').children.length,10);assert.equal(get('program-name').value,'Forward (tripod)');
 get('gait').value='crawl';get('generate').onclick();assert.equal(get('timeline').children.length,20);
});

test('insect gait and cycles come from the template settings',()=>{
 const {get,all}=setup();get('gait').value='insect-ripple';get('cycles').value='2';get('duration').value='200';
 all.find(el=>el.dataset.motion==='backward').onclick();
 assert.equal(get('timeline').children.length,1+6+24+6);assert.equal(get('program-name').value,'Backward (insect ripple)');
});

test('building a template warns about poses that may tip',()=>{
 const {get,all}=setup();
 all.find(el=>el.dataset.motion==='forward').onclick();assert.match(get('notice').textContent,/Draft template built/);
 get('stride').value='100';get('lift').value='40';get('height').value='90';all.find(el=>el.dataset.motion==='forward').onclick();
 assert.match(get('notice').textContent,/may tip the robot/);assert.match(get('notice').className,/error/);
});

test('live walk buttons start, steer and stop a board-computed walk',async()=>{
 const {get,commands}=setup();await get('connect').onclick();await get('arm').onclick();
 await get('walk-forward').onclick();assert.match(get('notice').textContent,/too old for live walking/);
 assert.equal(commands.filter(c=>c.op==='walk').length,0);
 const sim=setup();await sim.get('simulate').onclick();await sim.get('arm').onclick();
 sim.get('walk-gait').value='tripod';sim.get('walk-lift').value='20';sim.get('walk-speed').value='1';sim.get('walk-angle').value='12';
 await sim.get('walk-forward').onclick();
 assert.match(sim.get('hardware-state').textContent,/Walking · tripod/);
 assert.equal(sim.get('play').disabled,true);assert.equal(sim.get('arm').disabled,true);assert.equal(sim.get('walk-stop').disabled,false);
 await sim.get('walk-turn-left').onclick();assert.match(sim.get('notice').textContent,/Steering/);
 await sim.get('walk-stop').onclick();assert.match(sim.get('notice').textContent,/Slowing down/);
 assert.equal(sim.get('walk-stop').disabled,true);
});
test('walk preview animates offline without sending commands',()=>{
 const {get,commands,scene}=setup();get('walk-preview').onclick();
 assert.match(get('notice').textContent,/Previewing a forward live walk/);assert.equal(commands.length,0);
});

test('Live walk panel: step height, speed and stride angle, also while walking',async()=>{
 const {get,commands}=setup();
 get('walk-lift').value='25';get('walk-speed').value='1.25';get('walk-angle').value='20';get('walk-speed').oninput();
 assert.equal(get('walk-lift-value').textContent,'25 mm');assert.equal(get('walk-speed-value').textContent,'1.25 steps/s');assert.equal(get('walk-angle-value').textContent,'20°');
 assert.match(get('walk-readout').textContent,/Each step ≈ 61 mm/);
 const sim=setup();await sim.get('simulate').onclick();await sim.get('arm').onclick();
 for(const [id,v] of [['walk-lift','25'],['walk-speed','1.25'],['walk-angle','20']])sim.get(id).value=v;
 await sim.get('walk-forward').onclick();assert.match(sim.get('hardware-state').textContent,/Walking/);
 sim.get('walk-speed').value='2';await sim.get('walk-speed').onchange();
 assert.match(sim.get('notice').textContent,/New walk settings sent/);
 // Turning on the spot collides past about 46°, so a 60° stride angle is refused.
 await sim.get('walk-turn-left').onclick();sim.get('walk-angle').value='60';await sim.get('walk-angle').onchange();
 assert.match(sim.get('notice').textContent,/keeps its current settings/);
 assert.equal(commands.length,0);
});

test('poses and jogs cannot interrupt a live walk; a mid-step pause blocks restarting',async()=>{
 const sim=setup();await sim.get('simulate').onclick();await sim.get('arm').onclick();
 await sim.get('walk-forward').onclick();assert.match(sim.get('hardware-state').textContent,/Walking/);
 assert.equal(sim.get('walk-preview').disabled,true);
 await sim.get('jog-down').onclick();assert.match(sim.get('hardware-state').textContent,/Walking/,'a jog does not pause the walk');
 await new Promise(resolve=>setTimeout(resolve,300));
 await sim.get('pause').onclick();assert.match(sim.get('notice').textContent,/Walk paused mid-step/);
 await sim.get('walk-forward').onclick();assert.match(sim.get('notice').textContent,/stopped mid-step/);
 await sim.get('stand').onclick();await sim.get('walk-forward').onclick();assert.match(sim.get('hardware-state').textContent,/Walking/);
});

test('hiding the tab stops a live walk gently; settings work after a reconnect',async()=>{
 const sim=setup();await sim.get('simulate').onclick();await sim.get('arm').onclick();
 await sim.get('walk-forward').onclick();
 sim.document.hidden=true;sim.docListeners.visibilitychange();await new Promise(resolve=>setTimeout(resolve,0));
 assert.match(sim.get('notice').textContent,/tab was hidden/);
 sim.document.hidden=false;
 // A page that reconnected mid-walk has no remembered stance; settings still go to the board.
 const fresh=setup();await fresh.get('simulate').onclick();await fresh.get('arm').onclick();
 await fresh.get('walk-forward').onclick();vm.runInContext('walkBase=null',fresh.context);
 fresh.get('walk-speed').value='1.5';await fresh.get('walk-speed').onchange();
 assert.match(fresh.get('notice').textContent,/New walk settings sent/);
});
