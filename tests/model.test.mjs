import {test} from 'node:test';
import assert from 'node:assert/strict';
import {generate,generatePosture,postureTarget,validatePostureSettings,MOTIONS,anglesForPose,checkCollisions,setBoardLimits,validateFrame,parseProgram,home,clone,LIMITS,JOINT_LIMITS,DEFAULT_OFFSETS} from '../webapp/dist/model.js';
import {legJoints} from '../webapp/src/motion-geometry.js';
import {preflight,Player} from '../webapp/dist/player.js';
for(const motion of MOTIONS) {
 test(`${motion.label}: reachable 20-pose crawl and interpolation`,()=>{
  const program=generate(motion.id);
  assert.equal(program.frames.length,20);
  assert.deepEqual(program.frames[0].positions,home());
  assert.deepEqual(program.frames.at(-1).positions,home());
  for(const frame of program.frames) {
   assert.equal(anglesForPose(frame.positions).length,6);
   assert.ok(frame.positions.filter(xyz=>xyz[2]<70).length<=1);
  }
  preflight(program.frames,home(),DEFAULT_OFFSETS);
 });
}
test('opposite direction pairs reverse the planned stride',()=>{
 for(const [a,b] of [['forward','backward'],['left','right'],['turn-left','turn-right']]) {
  const one=generate(a),two=generate(b);
  for(let i=0;i<20;i++) for(let leg=0;leg<6;leg++) {
   assert.ok(Math.abs(one.frames[i].positions[leg][0]+two.frames[i].positions[leg][0])<1e-8);
   assert.ok(Math.abs(one.frames[i].positions[leg][1]+two.frames[i].positions[leg][1]-350)<1e-8);
  }
 }
});
test('program import round-trips and rejects unsafe targets',()=>{
 const original=generate('forward');assert.deepEqual(parseProgram(JSON.stringify(original)),original);
 const bad=structuredClone(original);bad.frames[4].positions[5][0]=999;
 assert.throws(()=>parseProgram(JSON.stringify(bad)),/workspace/);
 bad.frames=Array(129).fill(original.frames[0]);assert.throws(()=>parseProgram(JSON.stringify(bad)),/1–128/);
 assert.throws(()=>anglesForPose(home(),Array(6).fill([0,200,0])),/joint angle/);
});
test('player finishes each acknowledged pose before starting the next',async()=>{
 const operations=[],states=[],indices=[];
 const link={send:async(op,args,guard)=>{assert.ok(guard());operations.push(op);return {moving:op==='pose',active:true};}};
 const player=new Player(link,state=>states.push(state),i=>indices.push(i),async()=>{});
 await player.run(generate('forward').frames.slice(0,2));
 assert.deepEqual(operations,['pose','status','pose','status']);assert.deepEqual(indices,[0,1]);assert.equal(player.running,false);
});
test('cancel during a transition prevents the next pose',async()=>{
 const operations=[];
 const link={send:async(op)=>{operations.push(op);return {moving:true,active:true};}};
 const player=new Player(link,()=>{},()=>{},async()=>player.cancel());
 await player.run(generate('forward').frames.slice(0,3));
 assert.deepEqual(operations,['pose']);assert.equal(player.running,false);
});
test('player stops if board releases outputs',async()=>{
 const link={send:async op=>({moving:true,active:op==='pose'})};
 const player=new Player(link,()=>{},()=>{},async()=>{});
 await assert.rejects(player.run(generate('forward').frames.slice(0,2)),/disabled/);
 assert.equal(player.running,false);
});

test('disabled outputs in the initial pose reply prevent subsequent poses',async()=>{
 const operations=[];
 const player=new Player({send:async op=>{operations.push(op);return {moving:false,active:false};}},()=>{},()=>{},async()=>{});
 await assert.rejects(player.run(generate('forward').frames.slice(0,2)),/disabled/);
 assert.deepEqual(operations,['pose']);
 assert.equal(player.running,false);
});

test('Stand and Sit paths stay within the workspace and respect calibration',()=>{
 for(const id of ['stand','sit']){
  const program=generatePosture(id);
  preflight(program.frames,program.frames[0].positions,DEFAULT_OFFSETS);
  assert.deepEqual(parseProgram(JSON.stringify(program)),program);
  assert.throws(()=>generatePosture(id,Array(6).fill([0,200,0])),/joint angle/);
 }
 assert.throws(()=>generatePosture('invalid'),/Unknown posture/);
});

test('custom posture transitions validate every interpolation sample',()=>{
 const settings={standHeight:95,sitHeight:40,duration:3100};
 for(const id of ['stand','sit']){
  const program=generatePosture(id,DEFAULT_OFFSETS,settings);
  preflight(program.frames,program.frames[0].positions,DEFAULT_OFFSETS);
  assert.deepEqual(postureTarget(id,settings),program.frames.at(-1));
 }
 assert.throws(()=>validatePostureSettings({...settings,sitHeight:100}),/lower/);
 assert.throws(()=>validatePostureSettings({...settings,duration:500.5}),/100–10000/);
 assert.throws(()=>postureTarget('stand',settings,Array(6).fill([0,200,0])),/joint angle/);
});

test('wide workspace accepts negative coordinates and rejects the hip axis',()=>{
 const pose=home();pose[1]=[-60,120,-40];
 assert.equal(anglesForPose(pose).length,6);
 pose[1]=[0,0,100];assert.throws(()=>anglesForPose(pose),/L2 foot is too close to the hip axis/);
 for(const point of [[0,0,100],[0.2,-0.3,90]])assert.ok(legJoints(point,1).flat().every(Number.isFinite));
});
test('joint errors name the leg, joint and angle; per-joint board limits apply',()=>{
 const pose=home();pose[0]=[-100,-5,60];
 assert.throws(()=>anglesForPose(pose),/L1 hip -?\d+\.\d° is outside its joint angle limit -180\.\.180°/);
 const defaults=[clone(LIMITS),clone(JOINT_LIMITS)];
 try{
  setBoardLimits(defaults[0],[[-30,30],[-180,180],[-180,180]]);
  const turned=home();turned[2]=[120,120,70];
  assert.throws(()=>anglesForPose(turned,[[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]),/L3 hip 45\.0° is outside its joint angle limit -30\.\.30°/);
  assert.throws(()=>setBoardLimits(undefined,defaults[1]),/out of date|did not report/);
 }finally{setBoardLimits(...defaults);}
});
test('collision check rejects touching neighbours and feet inside the body',()=>{
 checkCollisions(home());
 const touching=home();touching[0]=[-110,65,70];touching[1]=[0,100,70];
 assert.throws(()=>checkCollisions(touching),/L1 and L2 feet are \d+ mm apart/);
 const inside=home();inside[1]=[0,-100,0];
 assert.throws(()=>checkCollisions(inside),/L2 foot would hit the body/);
 inside[1]=[0,-100,60];checkCollisions(inside);
 assert.throws(()=>validateFrame({name:'x',duration_ms:700,positions:touching}),/feet are/);
});
