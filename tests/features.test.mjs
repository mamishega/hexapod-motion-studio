import {test} from 'node:test';
import assert from 'node:assert/strict';
import {balance as supportBalance,bodyStep} from '../webapp/src/motion-geometry.js';
import {readFileSync} from 'node:fs';
import {walkFigures,walkPose,walkAmplitude,checkWalk,walkPreviewFrames,rotateFoot,anglesForPose,generate,generatePosture,postureTarget,validatePostureSettings,appendProgram,scaleDurations,setAllDurations,validateFrame,interpolationSteps,home,clone,DEFAULT_OFFSETS,MAX_FRAMES,POSTURE_HEIGHT,TRIM_LIMIT} from '../webapp/dist/model.js';
import {Player,preflight} from '../webapp/dist/player.js';
import {SimulatedLink} from '../webapp/dist/simulator.js';
import {balance,sampleSequence} from '../webapp/src/motion-geometry.js';

const settings={standHeight:70,sitHeight:30,duration:2000};
test('Stand/Sit heights use the wider range, and stand trims apply per leg',()=>{
 for(const [standHeight,sitHeight] of [[150,100],[0,-100],[130,10]]){
  const valid=validatePostureSettings({...settings,standHeight,sitHeight});
  for(const id of ['stand','sit'])preflight(generatePosture(id,DEFAULT_OFFSETS,valid).frames,generatePosture(id,DEFAULT_OFFSETS,valid).frames[0].positions,DEFAULT_OFFSETS);
 }
 assert.throws(()=>validatePostureSettings({...settings,standHeight:POSTURE_HEIGHT[1]+1}),/-100–150/);
 const trimmed=validatePostureSettings({...settings,standTrim:[4,0,0,0,0,-3]});
 assert.deepEqual(postureTarget('stand',trimmed).positions.map(p=>p[2]),[74,70,70,70,70,67]);
 assert.throws(()=>validatePostureSettings({...settings,standTrim:[TRIM_LIMIT+1,0,0,0,0,0]}),/Stand trim/);
 assert.throws(()=>validatePostureSettings({...settings,sitHeight:60,standTrim:[-15,0,0,0,0,0]}),/L1 sit height must stay below its Stand/);
 assert.deepEqual(validatePostureSettings(settings).standTrim,[0,0,0,0,0,0]);
});
test('joining movements drops only a template start and respects the pose limit',()=>{
 const walk=generate('forward'),stand=generatePosture('stand');
 const joined=appendProgram(walk,stand,{dropStart:true});
 assert.equal(joined.frames.length,walk.frames.length+1);assert.equal(joined.frames.at(-1).name,'Stand');assert.equal(joined.motion,'custom');
 assert.equal(appendProgram(walk,walk).frames.length,40);
 assert.equal(walk.frames.length,20,'original is not modified');
 let long=walk;for(let i=0;i<5;i++)long=appendProgram(long,walk);
 assert.equal(long.frames.length,120);assert.throws(()=>appendProgram(long,walk),new RegExp(`maximum is ${MAX_FRAMES}`));
});
test('gait speed scales transitions within the board range',()=>{
 const frames=[{duration_ms:700},{duration_ms:3000},{duration_ms:300}];
 assert.deepEqual(scaleDurations(frames,2),{frames:[{duration_ms:350},{duration_ms:1500},{duration_ms:150}],clamped:0});
 assert.deepEqual(scaleDurations(frames,4),{frames:[{duration_ms:175},{duration_ms:750},{duration_ms:100}],clamped:1});
 assert.deepEqual(scaleDurations(frames,0.25),{frames:[{duration_ms:2800},{duration_ms:10000},{duration_ms:1200}],clamped:1});
 assert.throws(()=>scaleDurations(frames,5),/speed/);
});
test('player repeats passes and sends easing only when smooth',async()=>{
 const sent=[];const link={send:async(op,args)=>{sent.push({op,...args});return {active:true,moving:false};}};
 const passes=[];const player=new Player(link,()=>{},()=>{},async()=>{});
 const frames=generate('forward').frames.slice(0,3);
 await player.run(frames,{repeat:3,ease:'smooth',onPass:p=>passes.push(p)});
 assert.equal(sent.length,9);assert.deepEqual(passes,[0,1,2]);assert.ok(sent.every(c=>c.ease==='smooth'));
 sent.length=0;await player.run(frames);assert.equal(sent.length,3);assert.ok(sent.every(c=>!('ease' in c)));
 // Until stopped: cancel during the second pass.
 let count=0;const stopper={send:async()=>{if(++count===5)looping.cancel();return {active:true,moving:false};}};
 const looping=new Player(stopper,()=>{},()=>{},async()=>{});
 await looping.run(frames,{repeat:Infinity});assert.equal(count,5);assert.equal(looping.running,false);
});
test('simulated board mirrors the bridge: arm, timed transitions, ease and pause',async()=>{
 let now=0;const sim=new SimulatedLink(()=>{},()=>{},{now:()=>now});
 const hello=await sim.open();assert.equal(hello.simulated,true);assert.equal(hello.active,false);assert.ok(hello.limits&&hello.joint_limits);
 await assert.rejects(sim.send('pose',{positions:home(),duration_ms:700}),/Enable at home/);
 const armed=await sim.send('arm');assert.equal(armed.active,true);assert.deepEqual(armed.positions,home());
 const target=home();target[0][0]=40;
 await assert.rejects(sim.send('pose',{positions:target,duration_ms:50}),/100..10000/);
 assert.equal((await sim.send('pose',{positions:target,duration_ms:1000,ease:'smooth'})).moving,true);
 await assert.rejects(sim.send('pose',{positions:target,duration_ms:1000}),/Wait for this pose/);
 now=500;assert.ok(Math.abs((await sim.send('status')).positions[0][0]-20)<1e-9);
 now=1000;const done=await sim.send('status');assert.equal(done.moving,false);assert.deepEqual(done.positions,target);
 const far=home();far[0]=[999,175,70];await assert.rejects(sim.send('pose',{positions:far,duration_ms:700}),/workspace/);
 assert.equal((await sim.send('off')).active,false);
 await sim.close();await assert.rejects(sim.send('status'),/disconnected/);
});
test('simulated board: release leg, joint test and joint limits',async()=>{
 const sim=new SimulatedLink(()=>{},()=>{},{now:()=>0});await sim.open();await sim.send('arm');
 const start=(await sim.send('status')).angles[0][1];
 const jogged=await sim.send('jog_joint',{leg:0,joint:1,angle:start+5});
 assert.equal(jogged.angles[0][1],start+5);assert.equal(jogged.positions,null);
 await assert.rejects(sim.send('jog_joint',{leg:0,joint:1,angle:start+30}),/10 deg/);
 await assert.rejects(sim.send('set_joint_limits',{joint_limits:[[-5,5],[-90,90],[-90,90]]}),/outside the new limits/);
 const limited=await sim.send('set_joint_limits',{joint_limits:[[-120,120],[-110,110],[-100,100]]});
 assert.deepEqual(limited.joint_limits,[[-120,120],[-110,110],[-100,100]]);
 const released=await sim.send('release_leg',{leg:2});assert.deepEqual(released.released,[2]);
 await assert.rejects(sim.send('jog_joint',{leg:2,joint:0,angle:0}),/released/);
 assert.deepEqual((await sim.send('arm')).released,[]);
});
test('a full sequence plays through the simulated board with the real player',async()=>{
 let now=0;const sim=new SimulatedLink(()=>{},()=>{},{now:()=>now});await sim.open();await sim.send('arm');
 const frames=generate('forward').frames.slice(0,4),seen=[];
 const player=new Player(sim,state=>seen.push(state),()=>{},async ms=>{now+=ms;});
 preflight(frames,home(),DEFAULT_OFFSETS);await player.run(frames,{repeat:2});
 assert.deepEqual((await sim.send('status')).positions,frames.at(-1).positions);assert.ok(seen.some(s=>s.moving));
});
test('balance: stable with all feet down, unstable with one side lifted',()=>{
 const all=balance(home());assert.equal(all.stable,true);assert.equal(all.down,6);assert.ok(all.margin>100);
 const tripod=home();for(const i of [0,4,2])tripod[i][2]=58;assert.equal(balance(tripod).stable,true);
 const side=home();for(const i of [0,1,2])side[i][2]=58;const tipped=balance(side);
 assert.equal(tipped.stable,false);assert.ok(tipped.margin<0);
 const two=home();for(const i of [0,1,2,3])two[i][2]=58;assert.equal(balance(two).down,2);assert.equal(balance(two).stable,false);
});
test('preview easing follows the same path with slower ends',()=>{
 const a=home(),b=clone(a);b[0][0]=40;
 const linear=sampleSequence([{positions:a,duration_ms:500},{positions:b,duration_ms:1000}],750).pose[0][0];
 const eased=sampleSequence([{positions:a,duration_ms:500},{positions:b,duration_ms:1000,ease:'smooth'}],750).pose[0][0];
 assert.ok(Math.abs(linear-10)<1e-9);assert.ok(Math.abs(eased-40*(0.5-0.5*Math.cos(Math.PI/4)))<1e-9);
});
test('simulated board calibrates one leg like the board bridge',async()=>{
 const sim=new SimulatedLink(()=>{},()=>{},{now:()=>0});await sim.open();
 await assert.rejects(sim.send('cal_move'),/Select a leg/);
 let state=await sim.send('cal_select',{leg:3});assert.equal(state.calibration.leg,3);assert.equal(state.active,false);
 const before=state.calibration.targets[1];
 state=await sim.send('cal_adjust',{joint:1,delta:-2});assert.equal(state.offsets[3][1],DEFAULT_OFFSETS[3][1]-2);assert.ok(Math.abs(state.calibration.targets[1]-(before-2))<1e-9);
 await assert.rejects(sim.send('cal_adjust',{joint:1,delta:20}),/-15..15/);
 state=await sim.send('cal_move');assert.equal(state.calibration.holding,true);assert.deepEqual(state.released,[0,1,2,4,5]);
 await assert.rejects(sim.send('pose',{positions:home(),duration_ms:700}),/Enable at home/);
 await sim.send('cal_save');assert.deepEqual(sim.savedOffsets,state.offsets);
 state=await sim.send('arm');assert.equal(state.calibration.holding,false);assert.equal(state.angles[3][1],sim.poseAngles(home())[3][1]);
});

test('transition times span 100–10000 ms with at most 200 checked steps',()=>{
 const pose=home();
 for(const ok of [100,250,10000])validateFrame({name:'x',duration_ms:ok,positions:pose});
 for(const bad of [99,10001,150.5])assert.throws(()=>validateFrame({name:'x',duration_ms:bad,positions:pose}),/100–10000/);
 assert.deepEqual([100,700,4000,10000].map(interpolationSteps),[5,35,200,200]);
 const walk=generate('forward'),slow=setAllDurations(walk,10000);
 assert.ok(slow.frames.every(f=>f.duration_ms===10000));assert.equal(walk.frames[0].duration_ms,700,'original is not modified');
 preflight(slow.frames,home(),DEFAULT_OFFSETS);
 assert.throws(()=>setAllDurations(walk,20000),/100–10000/);
});
test('simulated board plays a long transition in 200 steps',async()=>{
 let now=0;const sim=new SimulatedLink(()=>{},()=>{},{now:()=>now});await sim.open();await sim.send('arm');
 const target=home();target[0][0]=40;await sim.send('pose',{positions:target,duration_ms:10000});
 now=50;assert.ok(Math.abs((await sim.send('status')).positions[0][0]-0.2)<1e-9,'one 50 ms step');
 now=10000;assert.deepEqual((await sim.send('status')).positions,target);
});

test('simulated board refuses calibration on a leg powered for movement',async()=>{
 const sim=new SimulatedLink(()=>{},()=>{},{now:()=>0});await sim.open();
 await sim.send('cal_select',{leg:0});await sim.send('arm');
 await assert.rejects(sim.send('cal_adjust',{joint:0,delta:5}),/Motors are enabled/);
 assert.deepEqual((await sim.send('status')).offsets[0],DEFAULT_OFFSETS[0]);
});
test('walking templates allow big steps and explain unreachable ones',()=>{
 for(const [id,stride] of [['forward',100],['backward',100],['left',80],['right',80],['turn-left',60]]){
  const program=generate(id,{stride,lift:30,height:70});
  preflight(program.frames,home(70),DEFAULT_OFFSETS);
  const moved=program.frames.find(f=>f.name==='L2 / place').positions[1];
  assert.ok(Math.hypot(moved[0],moved[1]-175)>=stride-1e-9,`${id} L2 moves a full stride`);
 }
 assert.throws(()=>generate('forward',{stride:150,lift:12,height:70}),/cannot reach/);
 assert.throws(()=>generate('forward',{stride:151}),/stride 5–150 mm/);
 assert.throws(()=>generate('forward',{lift:61}),/lift 5–60 mm/);
});

test('rotateFoot turns a foot around its hip, keeping reach and height',()=>{
 const turned=rotateFoot([0,175,70],30);
 assert.ok(Math.abs(Math.hypot(turned[0],turned[1])-175)<1e-9);assert.equal(turned[2],70);
 assert.ok(Math.abs(Math.atan2(turned[0],turned[1])*180/Math.PI-30)<1e-9);
 const back=rotateFoot(turned,-30);assert.ok(Math.abs(back[0])<1e-9&&Math.abs(back[1]-175)<1e-9);
});
test('hip swing sizes template steps as exact hip rotations',()=>{
 const homeHip=anglesForPose(home())[1][0];
 for(const [id,swing,expected] of [['turn-left',40,40],['turn-right',40,-40],['forward',60,60],['backward',60,-60]]){
  const program=generate(id,{hipSwing:swing});preflight(program.frames,home(),DEFAULT_OFFSETS);
  const placed=program.frames.find(f=>f.name==='L2 / place').positions;
  assert.ok(Math.abs(anglesForPose(placed)[1][0]-homeHip-expected)<1e-9,`${id} L2 hip turns ${expected}°`);
  assert.ok(Math.abs(Math.hypot(placed[1][0],placed[1][1])-175)<1e-9,'reach unchanged');
 }
 assert.deepEqual(generate('forward',{hipSwing:0,stride:20}),generate('forward',{stride:20}),'0 keeps stride mode');
 // Too large a turn collides on the way between poses; the playback check catches it.
 assert.throws(()=>{const turn=generate('turn-left',{hipSwing:60});preflight(turn.frames,home(),DEFAULT_OFFSETS);},/feet are|cannot reach|joint angle/);
 assert.throws(()=>generate('forward',{hipSwing:91}),/0–90°/);
});

test('tripod gait moves three legs at once and keeps three feet down',()=>{
 for(const id of ['forward','backward','left','right','turn-left','turn-right']){
  const program=generate(id,{gait:'tripod',stride:60});preflight(program.frames,home(),DEFAULT_OFFSETS);
  assert.equal(program.frames.length,10);assert.deepEqual(program.frames.at(-1).positions,home());
  for(let k=1;k<program.frames.length;k++){
   const moved=program.frames[k].positions.map((p,i)=>p.some((v,j)=>v!==program.frames[k-1].positions[i][j])).filter(Boolean).length;
   assert.ok(moved===3||moved===6,`${id} pose ${k} changes ${moved} legs`);
   assert.ok(supportBalance(program.frames[k].positions).down>=3);
  }
 }
 const swing=generate('forward',{gait:'tripod',stride:60}).frames[2].positions;
 for(const i of [0,2,4])assert.ok(swing[i][2]<70,'tripod A (L1, L3, R2) is lifted');
 for(const i of [1,3,5])assert.equal(swing[i][2],70,'tripod B (L2, R1, R3) pushes on the ground');
 assert.equal(generate('forward',{gait:'tripod',hipSwing:30}).name,'Forward (tripod)');
 assert.throws(()=>generate('forward',{gait:'wave'}),/Unknown gait/);
});

test('insect gaits walk continuously with staggered legs and stay balanced',()=>{
 for(const [gait,down,keys,perCycle] of [['insect-tripod',3,12,2],['insect-ripple',4,12,1.5],['insect-wave',5,18,1.2]]){
  const program=generate('forward',{gait,stride:60,lift:20,cycles:2});
  preflight(program.frames,home(),DEFAULT_OFFSETS);
  assert.equal(program.frames.length,1+6+2*keys+6);assert.deepEqual(program.frames.at(-1).positions,home());
  const cycle=program.frames.filter(f=>f.name.startsWith('Cycle'));
  assert.ok(cycle.every(f=>supportBalance(f.positions).down>=down&&supportBalance(f.positions).stable),`${gait}: at least ${down} feet down, stable`);
  // Stepping in and out must be stable too, not just the walking cycle.
  assert.ok(program.frames.every(f=>supportBalance(f.positions).stable),`${gait}: every pose stable`);
  // Continuous: every leg moves in every cycle pose (no stop-and-wait phases).
  for(let k=1;k<cycle.length;k++)assert.ok(cycle[k].positions.every((p,i)=>p.some((v,j)=>Math.abs(v-cycle[k-1].positions[i][j])>1e-9)),`${gait} pose ${k}: all legs move`);
  // Body travel per cycle = stride ÷ fraction of time each foot is down.
  let travel=0;for(let k=0;k<keys;k++){const step=bodyStep((k?cycle[k-1]:program.frames[6]).positions,cycle[k].positions);travel+=Math.hypot(step.dx,step.dy);}
  assert.ok(Math.abs(travel-60*perCycle)<1e-6,`${gait} moves ${60*perCycle} mm per cycle, got ${travel}`);
 }
 // Back legs lead the wave: L3 swings before L2, L2 before L1.
 const wave=generate('forward',{gait:'insect-wave',stride:60,lift:20,cycles:1}).frames.filter(f=>f.name.startsWith('Cycle'));
 const firstLift=leg=>wave.findIndex(f=>f.positions[leg][2]<70);
 assert.ok(firstLift(2)<firstLift(1)&&firstLift(1)<firstLift(0),'L3, then L2, then L1');
 assert.throws(()=>generate('forward',{gait:'insect-wave',cycles:7}),/maximum is 128/);
 assert.throws(()=>generate('forward',{gait:'insect-tripod',cycles:0}),/Cycles must be 1–8/);
});
test('player polls quickly near the end of a transition',async()=>{
 const pauses=[];let polls=0;
 const link={send:async op=>op==='pose'?{active:true,moving:true}:{active:true,moving:++polls<4}};
 const player=new Player(link,()=>{},()=>{},async ms=>{pauses.push(ms);});
 await player.run([{positions:home(),duration_ms:200}]);
 assert.deepEqual(pauses,[150,50,20,20]);
});

test('with a queuing board the sequence plays without pauses between poses',async()=>{
 // Simulated board plus 8 ms of USB delay per message; count the board's idle
 // time between the first and the last pose, in 1 ms steps.
 const run=async queueing=>{
  let now=0,frame=-1,idle=0,started=false;const sim=new SimulatedLink(()=>{},()=>{},{now:()=>now});await sim.open();await sim.send('arm');
  const frames=generate('forward',{gait:'insect-tripod',stride:40,lift:20,duration:150,cycles:1}).frames;
  const tick=ms=>{for(let k=0;k<ms;k++){now++;sim.update();if(sim.transition)started=true;else if(started&&frame<frames.length-1)idle++;}};
  const link={send:async(op,args,guard)=>{tick(8);const state=await sim.send(op,args,guard);if(!queueing)delete state.queued;return state;}};
  const player=new Player(link,()=>{},i=>{frame=i;},async ms=>tick(ms));
  await player.run(frames);
  assert.deepEqual((await sim.send('status')).positions,frames.at(-1).positions);
  return idle;
 };
 assert.equal(await run(true),0,'no idle time between poses');
 assert.ok(await run(false)>200,'older boards pause between poses');
});
test('smoothness adds poses per cycle and keeps the cycle time',()=>{
 for(const smoothness of [1,2,3]){
  const program=generate('forward',{gait:'insect-ripple',stride:50,lift:20,duration:300,cycles:2,smoothness});
  preflight(program.frames,home(),DEFAULT_OFFSETS);
  const cycle=program.frames.filter(f=>f.name.startsWith('Cycle'));
  assert.equal(cycle.length,2*12*smoothness);assert.equal(cycle.reduce((s,f)=>s+f.duration_ms,0),2*3600);
 }
 assert.throws(()=>generate('forward',{gait:'insect-wave',cycles:3,smoothness:3}),/fewer cycles or less smoothness/);
 assert.throws(()=>generate('forward',{gait:'insect-tripod',smoothness:4}),/Smoothness must be/);
});

test('live walk foot paths match the board (shared fixture)',()=>{
 const fixture=JSON.parse(readFileSync(new URL('./walk_fixture.json',import.meta.url),'utf8'));
 for(const c of fixture.cases){const pose=walkPose(c.base,c.params,c.amplitude,c.t);pose.flat().forEach((v,k)=>assert.ok(Math.abs(v-c.pose.flat()[k])<1e-9));}
 for(const [cycle,free,stopping] of fixture.amplitudes){assert.ok(Math.abs(walkAmplitude(cycle,null)-free)<1e-12);assert.ok(Math.abs(walkAmplitude(cycle,1.5)-stopping)<1e-12);}
});
test('live walk: continuous paths, checks and preview',()=>{
 const params={gait:'ripple',direction:[1,0,0.5],stride:40,lift:20,period:1200,cycles:2};
 checkWalk(home(),params);
 assert.throws(()=>checkWalk(home(),{...params,direction:[1,1,1],stride:150,lift:60}),/cannot reach|joint angle|feet are/);
 assert.throws(()=>checkWalk(home(),{...params,direction:[0,0,0]}),/direction/);
 assert.throws(()=>checkWalk(home(),{...params,period:100}),/Cycle time/);
 // Starts and ends in the stance, and feet move only a little between samples.
 assert.deepEqual(walkPose(home(),params,0,0.37),home());
 const frames=walkPreviewFrames(home(),params,2);assert.deepEqual(frames.at(-1).positions,home());
 let jump=0;for(let n=1;n<=480;n++){const a=walkPose(home(),params,1,(n-1)/480),b=walkPose(home(),params,1,n/480);jump=Math.max(jump,...a.map((p,i)=>Math.hypot(...p.map((v,j)=>v-b[i][j]))));}
 assert.ok(jump<2,`max foot movement per 1/480 cycle ${jump}`);
 assert.ok(frames.every(f=>supportBalance(f.positions).stable),'ripple walk stays balanced');
});
test('simulated board walks live: start, steer, stop',async()=>{
 let now=0;const sim=new SimulatedLink(()=>{},()=>{},{now:()=>now});await sim.open();await sim.send('arm');
 await assert.rejects(sim.send('walk',{gait:'tripod',direction:[1,0,0],stride:40,lift:20,period_ms:100}),/Cycle time/);
 let state=await sim.send('walk',{gait:'tripod',direction:[1,0,0],stride:40,lift:20,period_ms:1000});
 assert.equal(state.walking.gait,'tripod');assert.equal(state.moving,true);
 await assert.rejects(sim.send('pose',{positions:home(),duration_ms:700}),/Stop walking first/);
 now=1500;await sim.send('walk',{direction:[0,0,1]});now=1600;state=await sim.send('status');
 assert.ok(state.walking.direction[2]>0&&state.walking.direction[2]<1,'turns gradually');
 await sim.send('walk_stop');now=2700;state=await sim.send('status');
 assert.equal(state.walking,null);assert.equal(state.moving,false);assert.deepEqual(state.positions,home());
});

test('simulated board changes walk settings mid-walk and reports them',async()=>{
 let now=0;const sim=new SimulatedLink(()=>{},()=>{},{now:()=>now});await sim.open();await sim.send('arm');
 await sim.send('walk',{gait:'ripple',direction:[1,0,0],stride:40,lift:20,hip_swing:15,period_ms:1500});
 now=2000;await sim.send('status');
 let state=await sim.send('walk',{lift:30,hip_swing:20,period_ms:800});
 assert.equal(state.walking.period,800);assert.deepEqual(state.walking.target,[1,0,0]);
 now=2100;state=await sim.send('status');assert.ok(state.walking.lift>20&&state.walking.lift<30,'step height blends in');
 now=6000;state=await sim.send('status');assert.equal(state.walking.lift,30);assert.equal(state.walking.hip_swing,20);
 await assert.rejects(sim.send('walk',{lift:70}),/Step height/);
 assert.deepEqual(walkFigures({gait:'tripod',hipSwing:15,speed:1}),{step:175*15*Math.PI/180,speed:2*175*15*Math.PI/180});
});

test('simulated board blocks walking from a mid-step pause until a pose',async()=>{
 let now=0;const sim=new SimulatedLink(()=>{},()=>{},{now:()=>now});await sim.open();await sim.send('arm');
 const start={gait:'tripod',direction:[1,0,0],stride:40,lift:25,hip_swing:15,period_ms:1000};
 await sim.send('walk',start);now=1620;await sim.send('status');assert.equal((await sim.send('pause')).mid_step,true);
 await assert.rejects(sim.send('walk',start),/mid-step/);
 await sim.send('pose',{positions:home(),duration_ms:500});now=2200;assert.equal((await sim.send('status')).mid_step,false);
 assert.ok((await sim.send('walk',start)).walking);
});
