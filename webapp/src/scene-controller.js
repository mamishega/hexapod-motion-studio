import {DEFAULT_POSE,DEFAULT_BODY,copyPose,mixPose,sampleSequence,totalDuration,bodyStep,composeBody,sampleBodyAt,legJoints,transformWorld} from './motion-geometry.js';

// Preview-only clock: this module has no USB transport and cannot move a servo.
export class SceneController {
  constructor({request=callback=>requestAnimationFrame(callback),cancel=id=>cancelAnimationFrame(id),onFrame=()=>{},onPlayback=()=>{}}={}) {
    this.request=request;this.cancelRequest=cancel;this.onFrame=onFrame;this.onPlayback=onPlayback;
    this.listeners=new Set();this.raf=null;this.lastStamp=null;this.tween=null;
    this.state={pose:copyPose(DEFAULT_POSE),body:{...DEFAULT_BODY},view:'angled',speed:1,trails:false,loop:false,history:[],frames:[],elapsed:0,total:0,index:0,playback:'idle',ground:70};
  }
  subscribe=listener=>{this.listeners.add(listener);return()=>this.listeners.delete(listener);};
  getSnapshot=()=>this.state;
  publish(patch){this.state={...this.state,...patch};this.listeners.forEach(listener=>listener());}
  cancel(){if(this.raf!==null)this.cancelRequest(this.raf);this.raf=null;this.lastStamp=null;this.tween=null;}
  schedule(){if(this.raf===null)this.raf=this.request(this.tick);}
  tick=stamp=>{
    this.raf=null;
    if(this.lastStamp===null)this.lastStamp=stamp;
    const delta=Math.max(0,stamp-this.lastStamp);this.lastStamp=stamp;
    if(this.tween){
      const tween=this.tween;tween.elapsed+=delta;const t=Math.min(1,tween.elapsed/tween.duration);
      const pose=mixPose(tween.start,tween.target,t);
      const body=composeBody(this.state.body,bodyStep(this.state.pose,pose));
      this.publish({pose,body,history:this.remember(pose,body)});
      if(t>=1){this.tween=null;this.lastStamp=null;}else this.schedule();return;
    }
    if(this.state.playback!=='playing')return;
    const sample=sampleSequence(this.state.frames,this.state.elapsed+delta*this.state.speed);
    const body=sampleBodyAt(this.state.frames,sample.elapsed,this.loopBase||DEFAULT_BODY);
    const previous=this.state.index,ended=sample.elapsed>=sample.total;
    if(ended&&this.state.loop){
      // Loop: restart from the first pose; keep the motion trail and body heading so the robot keeps walking/turning.
      this.loopBase=body;
      const first=sampleSequence(this.state.frames,0);
      this.publish({...first,body,playback:'playing',history:this.remember(sample.pose,body)});
      this.onFrame(0);this.schedule();return;
    }
    this.publish({...sample,body,playback:ended?'ended':'playing',history:this.remember(sample.pose,body)});
    if(sample.index!==previous)this.onFrame(sample.index);
    if(ended){this.lastStamp=null;this.onPlayback('ended');}else this.schedule();
  };
  // Short rolling history of recent poses and world foot positions, drawn as fading foot trails.
  remember(pose,body=this.state.body){
    const entry=Object.assign(copyPose(pose),{worldFeet:pose.map((p,i)=>transformWorld(legJoints(p,i)[3],body))});
    return [...this.state.history,entry].slice(-16);
  }
  setPose(pose,{duration=260,ground=70}={}) {
    const start=copyPose(this.state.pose);this.cancel();this.loopBase={...DEFAULT_BODY};
    this.publish({frames:[],elapsed:0,total:0,index:0,playback:'idle',ground,body:{...DEFAULT_BODY},history:[]});
    if(duration<=0){this.publish({pose:copyPose(pose)});return;}
    this.tween={start,target:copyPose(pose),duration,elapsed:0};this.schedule();
  }
  playSequence(frames,{ground=70}={}) {
    this.cancel();this.loopBase={...DEFAULT_BODY};
    const copied=frames.map(f=>({positions:copyPose(f.positions),duration_ms:f.duration_ms,ease:f.ease}));
    const sample=sampleSequence(copied,0);
    this.publish({...sample,frames:copied,playback:'playing',ground,body:{...DEFAULT_BODY},history:[]});
    this.onFrame(0);this.onPlayback('playing');this.schedule();
  }
  pause(){if(this.state.playback!=='playing')return;this.cancel();this.publish({playback:'paused'});this.onPlayback('paused');}
  resume(){if(!this.state.frames.length)return;this.cancel();if(this.state.elapsed>=this.state.total){this.loopBase={...DEFAULT_BODY};const sample=sampleSequence(this.state.frames,0);this.publish({...sample,body:{...DEFAULT_BODY},history:[]});this.onFrame(0);}this.publish({playback:'playing'});this.onPlayback('playing');this.schedule();}
  stop(){this.cancel();this.loopBase={...DEFAULT_BODY};this.publish({playback:'idle',frames:[],elapsed:0,total:0,body:{...DEFAULT_BODY},history:[]});this.onPlayback('idle');}
  seek(time){if(!this.state.frames.length)return;this.cancel();this.loopBase={...DEFAULT_BODY};const sample=sampleSequence(this.state.frames,time);const body=sampleBodyAt(this.state.frames,sample.elapsed);this.publish({...sample,body,playback:'paused'});this.onFrame(sample.index);this.onPlayback('paused');}
  setSpeed(speed){if(![0.25,0.5,1,2].includes(speed))throw new Error('Unsupported preview speed');this.publish({speed});}
  setView(view){if(!['angled','top','front','side'].includes(view))throw new Error('Unsupported view');this.publish({view});}
  toggleLoop(){this.publish({loop:!this.state.loop});}
  toggleTrails(){this.publish({trails:!this.state.trails});}
  destroy(){this.cancel();this.listeners.clear();}
}

