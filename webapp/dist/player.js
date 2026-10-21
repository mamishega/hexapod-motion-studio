import {anglesForPose,checkCollisions,validateFrame,interpolationSteps} from './model.js';
export function preflight(frames,start,offsets) {
  let previous=start;
  for(const frame of frames) {
    validateFrame(frame,offsets);
    const steps=interpolationSteps(frame.duration_ms);
    for(let step=1;step<=steps;step++) {
      const fraction=step/steps;
      const pose=previous.map((xyz,i)=>xyz.map((v,j)=>v+(frame.positions[i][j]-v)*fraction));
      anglesForPose(pose,offsets);checkCollisions(pose);
    }
    previous=frame.positions;
  }
}
export class Player {
  constructor(link,onState,onFrame,wait=ms=>new Promise(resolve=>setTimeout(resolve,ms))) {
    this.link=link;this.onState=onState;this.onFrame=onFrame;this.wait=wait;this.generation=0;this.running=false;
  }
  cancel(){this.generation++;this.running=false;}
  // repeat: number of passes, or Infinity until cancelled. ease 'smooth' asks the
  // board to slow the start and end of each transition (older boards ignore it).
  async run(frames,{repeat=1,ease='linear',onPass=()=>{}}={}) {
    if(this.running)throw new Error('A sequence is already running.');
    this.running=true;const generation=++this.generation;
    const valid=()=>generation===this.generation;
    const extra=ease==='smooth'?{ease}:{};
    // Poll while busy(state) holds: every 150 ms (keeping the board's 3 s
    // watchdog fed), but every 20 ms near the expected end of `expected` ms.
    // steady: poll at a third of a pose instead, so a late reply never adds up
    // across poses and the next pose is always queued in time.
    const waitWhile=async(state,busy,expected,steady=false)=>{
      let polls=0,waited=0;
      while(busy(state)&&valid()) {
        const pause=steady?Math.max(20,Math.min(150,Math.round(expected/3))):Math.max(20,Math.min(150,expected-waited));waited+=pause;
        await this.wait(pause);if(!valid())break;
        state=await this.link.send('status',{},valid);if(!valid())break;this.onState(state);
        if(!state.active)throw new Error('Outputs were disabled; playback stopped.');
        // Allow the full transition plus a margin for slow replies.
        if(++polls>Math.ceil(expected/20)+40)throw new Error('Pose did not finish; playback stopped.');
      }
      return state;
    };
    try{
      let state=null,previous=null;
      for(let pass=0;pass<repeat&&valid();pass++) {onPass(pass);
      for(let i=0;i<frames.length&&valid();i++) {
        const frame=frames[i];
        // Boards that report 'queued' take the next pose while the current one
        // plays and start it the moment it ends: no pause between poses.
        const queue=!!state?.moving&&'queued' in state;
        if(!queue)this.onFrame(i);
        state=await this.link.send('pose',{positions:frame.positions,duration_ms:frame.duration_ms,...extra,...(queue?{queue:true}:{})},valid);
        if(!valid())break;this.onState(state);
        // Bug: a completed pose reply with disabled outputs skipped the polling check.
        if(!state.active)throw new Error('Outputs were disabled; playback stopped.');
        if(queue){state=await waitWhile(state,s=>s.queued,previous.duration_ms,true);if(!valid())break;this.onFrame(i);}
        else if(!('queued' in state))state=await waitWhile(state,s=>s.moving,frame.duration_ms);
        previous=frame;
      }
      }
      // Let the last pose finish before reporting playback complete.
      if(valid()&&state?.moving)await waitWhile(state,s=>s.moving,previous.duration_ms);
    }catch(error){if(valid())throw error;}
    finally{if(valid())this.running=false;}
  }
}
