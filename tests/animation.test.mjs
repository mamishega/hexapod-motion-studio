import {test} from 'node:test';
import assert from 'node:assert/strict';
import {legJoints,sampleSequence,mixPose} from '../webapp/src/motion-geometry.js';
import {SceneController} from '../webapp/src/scene-controller.js';
import {generate,MOTIONS,home} from '../webapp/dist/model.js';
const distance=(a,b)=>Math.hypot(...a.map((v,i)=>v-b[i]));
function clock(options={}){let id=0;const scheduled=new Map();const controller=new SceneController({...options,request:fn=>{scheduled.set(++id,fn);return id;},cancel:key=>scheduled.delete(key)});return {controller,scheduled,step:time=>{const callbacks=[...scheduled.values()];scheduled.clear();callbacks.forEach(fn=>fn(time));}};}

test('articulated legs preserve physical segment lengths through every template',()=>{
 for(const motion of MOTIONS)for(const frame of generate(motion.id).frames)for(let i=0;i<6;i++){
  const joints=legJoints(frame.positions[i],i);
  for(const [j,length] of [[0,47],[1,88.46],[2,148.8]])assert.ok(Math.abs(distance(joints[j],joints[j+1])-length)<1e-7);
 }
});
test('preview samples continuous midpoint positions and reaches exact endpoints',()=>{
 const frames=generate('forward').frames;
 assert.equal(sampleSequence(frames,1050).pose[0][2],64);
 assert.equal(sampleSequence(frames,1400).pose[0][2],58);
 assert.deepEqual(sampleSequence(frames,999999).pose,home());
 assert.deepEqual(sampleSequence(frames,-1).pose,home());
});
test('pause cancels scheduled frames and resume excludes time spent paused',()=>{
 const {controller,step,scheduled}=clock();controller.playSequence(generate('forward').frames);
 step(0);step(1000);controller.pause();assert.equal(scheduled.size,0);
 step(100000);assert.equal(controller.getSnapshot().elapsed,1000);
 controller.resume();step(200000);step(200500);assert.equal(controller.getSnapshot().elapsed,1500);
 controller.destroy();assert.equal(scheduled.size,0);
});
test('scrubbing pauses, changes pose and supports slower replay',()=>{
 const {controller,step}=clock();controller.playSequence(generate('forward').frames);controller.seek(1050);
 assert.equal(controller.getSnapshot().playback,'paused');assert.equal(controller.getSnapshot().pose[0][2],64);
 controller.setSpeed(.25);controller.resume();step(0);step(1000);assert.equal(controller.getSnapshot().elapsed,1300);
 assert.throws(()=>controller.setSpeed(30),/Unsupported/);
});
test('selection tweens cancel previous animation and stop at the chosen pose',()=>{
 const {controller,step,scheduled}=clock();const target=home();target[0][0]=20;
 controller.setPose(target,{duration:200});step(0);step(100);assert.equal(controller.getSnapshot().pose[0][0],10);
 const next=home();next[0][0]=-20;controller.setPose(next,{duration:200});step(100);step(300);
 assert.deepEqual(controller.getSnapshot().pose,next);assert.equal(scheduled.size,0);
});
test('playback ends once and can restart without altering the program',()=>{
 const frames=generate('turn-left').frames,original=JSON.stringify(frames),events=[];
 const {controller,step,scheduled}=clock({onPlayback:mode=>events.push(mode)});
 controller.playSequence(frames);step(0);step(14000);
 assert.equal(controller.getSnapshot().playback,'ended');assert.equal(scheduled.size,0);
 assert.deepEqual(events,['playing','ended']);assert.equal(JSON.stringify(frames),original);
 controller.resume();assert.equal(controller.getSnapshot().elapsed,0);controller.stop();assert.equal(scheduled.size,0);
});
test('body kinematics translate and rotate the hexapod in the direction of each gait',()=>{
 const runMotion=id=>{
  const {controller,step}=clock();
  const frames=generate(id).frames;
  controller.playSequence(frames);
  step(0);step(20000);
  return controller.getSnapshot().body;
 };
 const forward=runMotion('forward'),backward=runMotion('backward');
 const left=runMotion('left'),right=runMotion('right');
 const turnLeft=runMotion('turn-left'),turnRight=runMotion('turn-right');
 assert.ok(forward.y<-10&&Math.abs(forward.x)<1&&Math.abs(forward.theta)<0.02);
 assert.ok(backward.y>10&&Math.abs(backward.x)<1&&Math.abs(backward.theta)<0.02);
 assert.ok(left.x<-10&&Math.abs(left.y)<1&&Math.abs(left.theta)<0.02);
 assert.ok(right.x>10&&Math.abs(right.y)<1&&Math.abs(right.theta)<0.02);
 assert.ok(turnLeft.theta>0.04&&Math.hypot(turnLeft.x,turnLeft.y)<1);
 assert.ok(turnRight.theta<-0.04&&Math.hypot(turnRight.x,turnRight.y)<1);
});

