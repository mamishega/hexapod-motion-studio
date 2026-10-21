export const NAMES = ['L1','L2','L3','R1','R2','R3'];
// Offline defaults: full leg reach (47 + 88.46 + 148.8 mm) on every axis, and
// (hip, upper, lower) servo limits. On connect, setBoardLimits() replaces both
// with the board's LIMITS and JOINT_LIMITS, so the board files are the source.
export const LIMITS = [[-284,284],[-284,284],[-284,284]];
export const JOINT_LIMITS = [[-180,180],[-180,180],[-180,180]];
export const JOINT_NAMES = ['hip','upper leg','lower leg'];
export function setBoardLimits(limits,jointLimits) {
  const valid=pairs=>Array.isArray(pairs)&&pairs.length===3&&pairs.every(p=>Array.isArray(p)&&p.length===2&&p.every(Number.isFinite)&&p[0]<p[1]);
  if(!valid(limits)||!valid(jointLimits))throw new Error('The board did not report its workspace and joint limits. Copy the current FUNCTIONS.py and motion_bridge.py to the board.');
  // Mutate in place so every importer sees the board's values.
  limits.forEach((p,j)=>{LIMITS[j][0]=p[0];LIMITS[j][1]=p[1];});
  jointLimits.forEach((p,j)=>{JOINT_LIMITS[j][0]=p[0];JOINT_LIMITS[j][1]=p[1];});
}
// Approximate collision geometry, matching the preview's hip mounts in
// src/motion-geometry.js. These are not measured from the frame.
const MOUNTS = [[-58,-70],[-72,0],[-58,70],[58,-70],[72,0],[58,70]];
const ROTATIONS = [30,0,-30,150,180,-150].map(v=>v*Math.PI/180);
const NEIGHBOURS = [[0,1],[1,2],[3,4],[4,5],[0,3],[2,5]];
const BODY_OUTLINE = [0,1,2,5,4,3].map(i=>MOUNTS[i]);
export const MIN_FOOT_GAP = 30;   // mm between neighbouring feet
export const BODY_CLEARANCE = 20; // mm below the hip plane for feet under the body
function footInBody([x,y,z],i) {
  const t=ROTATIONS[i],[mx,my]=MOUNTS[i];
  return [mx+x*Math.sin(t)-y*Math.cos(t),my-x*Math.cos(t)-y*Math.sin(t),z];
}
function insideOutline([px,py]) {
  let inside=false;
  for(let a=0,b=BODY_OUTLINE.length-1;a<BODY_OUTLINE.length;b=a++){
    const [ax,ay]=BODY_OUTLINE[a],[bx,by]=BODY_OUTLINE[b];
    if((ay>py)!==(by>py)&&px<(bx-ax)*(py-ay)/(by-ay)+ax)inside=!inside;
  }
  return inside;
}
export function checkCollisions(pose) {
  const feet=pose.map(footInBody);
  feet.forEach((foot,i)=>{if(foot[2]<BODY_CLEARANCE&&insideOutline(foot))throw new Error(`${NAMES[i]} foot would hit the body. Move it outward or lower it below Z ${BODY_CLEARANCE} mm.`);});
  for(const [a,b] of NEIGHBOURS){const gap=Math.hypot(...feet[a].map((v,k)=>v-feet[b][k]));if(gap<MIN_FOOT_GAP)throw new Error(`${NAMES[a]} and ${NAMES[b]} feet are ${gap.toFixed(0)} mm apart (minimum ${MIN_FOOT_GAP} mm).`);}
}
export const DEFAULT_OFFSETS = [[-25,5,0],[-15,3,4],[-20,0,0],[-20,-12,-12],[-20,15,-10],[-37,0,-20]];
export const MOTIONS = [
  {id:'forward', label:'Forward', icon:'↑', forward:1, lateral:0, turn:0},
  {id:'backward', label:'Backward', icon:'↓', forward:-1, lateral:0, turn:0},
  {id:'left', label:'Sideways left', icon:'←', forward:0, lateral:1, turn:0},
  {id:'right', label:'Sideways right', icon:'→', forward:0, lateral:-1, turn:0},
  {id:'turn-left', label:'Turn left', icon:'↶', forward:0, lateral:0, turn:1},
  {id:'turn-right', label:'Turn right', icon:'↷', forward:0, lateral:0, turn:-1}
];
export const clone = value => JSON.parse(JSON.stringify(value));
// Transition time per pose, shared with the board (motion_bridge.py).
export const MIN_DURATION = 100, MAX_DURATION = 10000;
// Interpolation steps for one transition: every 20 ms, capped at 200 so long
// transitions stay quick to check. Must match steps() in motion_bridge.py.
export const interpolationSteps = duration => Math.max(1,Math.min(200,Math.floor(duration/20)));
const validDuration = value => Number.isInteger(value)&&value>=MIN_DURATION&&value<=MAX_DURATION;
export function home(height=70) { return NAMES.map(()=>[0,175,height]); }
export function anglesForPose(pose, offsets=DEFAULT_OFFSETS) {
  if(!Array.isArray(pose)||pose.length!==6) throw new Error('A pose needs six legs.');
  return pose.map((xyz,i)=>{
    if(!Array.isArray(xyz)||xyz.length!==3||xyz.some((v,j)=>typeof v!=='number'||!Number.isFinite(v)||v<LIMITS[j][0]||v>LIMITS[j][1])) throw new Error(`${NAMES[i]} is outside the training workspace.`);
    const [x,y,z]=xyz, l1=47,l2=88.46,l3=148.8;
    const a=Math.hypot(x,y), b=Math.hypot(a-l1,z);
    // The hip angle is undefined for a foot on the hip's vertical axis.
    if(a<1) throw new Error(`${NAMES[i]} foot is too close to the hip axis (X and Y near 0).`);
    if(b<Math.abs(l2-l3)-1e-6||b>l2+l3+1e-6) throw new Error(`${NAMES[i]} cannot reach this position.`);
    const degrees=r=>r*180/Math.PI, acos=v=>degrees(Math.acos(Math.max(-1,Math.min(1,v))));
    const hip=90-degrees(Math.atan2(y,x));
    const upper=acos((l2*l2+b*b-l3*l3)/(2*l2*b))-(90-degrees(Math.atan2(a-l1,z)));
    const lower=90-acos((l2*l2+l3*l3-b*b)/(2*l2*l3));
    const sign=i<3?1:-1;
    const values=[hip,sign*upper,sign*lower].map((v,j)=>v+offsets[i][j]);
    values.forEach((v,j)=>{const [low,high]=JOINT_LIMITS[j];if(!Number.isFinite(v)||v<low||v>high)throw new Error(`${NAMES[i]} ${JOINT_NAMES[j]} ${v.toFixed(1)}° is outside its joint angle limit ${low}..${high}°. Reduce the stride/lift or edit the pose.`);});
    return values;
  });
}
export function validateFrame(frame,offsets=DEFAULT_OFFSETS) {
  if(!frame||typeof frame.name!=='string'||frame.name.length>80) throw new Error('Pose names must be text up to 80 characters.');
  if(!validDuration(frame.duration_ms)) throw new Error(`Pose duration must be ${MIN_DURATION}–${MAX_DURATION} ms.`);
  anglesForPose(frame.positions,offsets);
  checkCollisions(frame.positions);
  return frame;
}
// Template input ranges. These are wide on purpose: every generated pose and
// path is still checked for reach, joint limits and collisions, and the error
// names the leg that cannot make the step.
export const STRIDE_RANGE = [5,150], LIFT_RANGE = [5,60], BODY_HEIGHT_RANGE = [30,150], HIP_SWING_MAX = 90;
// Turn a foot around its hip's vertical axis, keeping its reach and height.
// Positive degrees increase the hip angle (the foot moves toward +X).
export function rotateFoot([x,y,z],degrees) {
  const angle=Math.atan2(x,y)+degrees*Math.PI/180,reach=Math.hypot(x,y);
  return [reach*Math.sin(angle),reach*Math.cos(angle),z];
}
// Gaits: 'crawl' moves one leg at a time (5 feet down); 'tripod' moves three
// at once (L1, L3, R2 alternate with R1, R3, L2), so 3 feet are always down.
// The insect gaits are continuous cycles like a real insect: each leg spends
// part of the cycle pushing back on the ground (stance) and the rest swinging
// forward along an arc, with the legs' phases staggered back to front.
export const GAITS = ['crawl','tripod','insect-tripod','insect-ripple','insect-wave'];
const TRIPODS = [[0,2,4],[3,5,1]];
// stance: fraction of the cycle each foot is on the ground; phase: when each
// leg (L1, L2, L3, R1, R2, R3) starts its swing; keys: poses per cycle.
const INSECT = {
  'insect-tripod':{stance:1/2,phase:[0,1/2,0,1/2,0,1/2],keys:12,label:'insect tripod'},
  'insect-ripple':{stance:2/3,phase:[2/3,1/3,0,1/6,5/6,1/2],keys:12,label:'insect ripple'},
  'insect-wave':{stance:5/6,phase:[2/6,1/6,0,5/6,4/6,3/6],keys:18,label:'insect wave'}
};
export const CYCLES_RANGE = [1,8];
// Smoothness multiplies the poses per insect cycle (rounder foot arcs) and
// divides each pose's time to match, so the walking speed stays the same.
export const SMOOTHNESS = [1,2,3];
// hipSwing > 0 sizes the step as a hip rotation in degrees instead of a stride in mm.
export function generate(id,{stride=15,lift=12,height=70,duration=700,hipSwing=0,gait='crawl',cycles=3,smoothness=1}={},offsets=DEFAULT_OFFSETS) {
  const motion=MOTIONS.find(m=>m.id===id); if(!motion) throw new Error('Unknown movement.');
  const inside=(v,[low,high])=>Number.isFinite(v)&&v>=low&&v<=high;
  if(!inside(stride,STRIDE_RANGE)||!inside(lift,LIFT_RANGE)||!inside(height,BODY_HEIGHT_RANGE)) throw new Error(`Use stride ${STRIDE_RANGE.join('–')} mm, lift ${LIFT_RANGE.join('–')} mm and height ${BODY_HEIGHT_RANGE.join('–')} mm.`);
  if(!inside(hipSwing,[0,HIP_SWING_MAX])) throw new Error(`Hip swing must be 0–${HIP_SWING_MAX}°.`);
  if(!GAITS.includes(gait)) throw new Error('Unknown gait.');
  if(!Number.isInteger(cycles)||!inside(cycles,CYCLES_RANGE)) throw new Error(`Cycles must be ${CYCLES_RANGE.join('–')}.`);
  if(!SMOOTHNESS.includes(smoothness)) throw new Error(`Smoothness must be ${SMOOTHNESS.join(', ')}.`);
  const frames=[], pose=home(height), theta=[30,0,-30,150,180,-150].map(v=>v*Math.PI/180);
  const add=name=>{const frame={name,positions:clone(pose),duration_ms:duration};validateFrame(frame,offsets);frames.push(frame);};
  // Place foot i at a fraction k of a full step from home (k = 1 is one stride ahead).
  const place=(i,k)=>{
    const t=theta[i];
    // Step direction in this leg's frame: X is across the leg (hip rotation), Y along it (reach).
    const across=motion.forward*Math.cos(t)-motion.lateral*Math.sin(t)+motion.turn,along=motion.forward*Math.sin(t)+motion.lateral*Math.cos(t);
    if(hipSwing>0){
      // Rotate the foot around the hip by the across part of the swing; change
      // the reach by the matching arc length for the along part.
      const swing=hipSwing*Math.PI/180;
      [pose[i][0],pose[i][1]]=rotateFoot([0,175+k*along*175*swing,0],k*across*hipSwing);
    } else {
      pose[i][0]=k*stride*across;
      pose[i][1]=175+k*stride*along;
    }
  };
  // Raise (up) or lower feet by the template's foot lift.
  const raise=(legs,up)=>legs.forEach(i=>{pose[i][2]=up?height-lift:height;});
  add('Home / start');
  const insect=INSECT[gait];
  if(insect){
    const keys=insect.keys*smoothness,step=Math.max(MIN_DURATION,Math.round(duration/smoothness));
    const total=1+6+keys*cycles+6;
    if(total>MAX_FRAMES) throw new Error(`${cycles} cycles need ${total} poses; the maximum is ${MAX_FRAMES}. Use fewer cycles or less smoothness.`);
    // Foot i at cycle time t (0..1): swing forward along a raised arc, then
    // push back in a straight line on the ground. k runs -0.5 .. +0.5 strides.
    const cyclePose=t=>NAMES.forEach((_,i)=>{
      const p=((t-insect.phase[i])%1+1)%1,swing=1-insect.stance;
      if(p<swing){const s=p/swing;place(i,-0.5+(1-Math.cos(Math.PI*s))/2);pose[i][2]=height-lift*Math.sin(Math.PI*s);}
      else{place(i,0.5-(p-swing)/insect.stance);pose[i][2]=height;}
    });
    const start=(()=>{cyclePose(0);const p=clone(pose);pose.splice(0,6,...home(height));return p;})();
    // Step in from home, three legs at a time, so the cycle starts in rhythm.
    // Every foot is put down first, even one whose rhythm starts mid-swing, so
    // the other three never lift while it is still in the air.
    for(const group of TRIPODS){
      raise(group,true); add('Step in / lift');
      group.forEach(i=>{pose[i][0]=start[i][0];pose[i][1]=start[i][1];}); add('Step in / place');
      raise(group,false); add('Step in / lower');
    }
    for(let c=0;c<cycles;c++)for(let j=1;j<=keys;j++){cyclePose(j/keys);add(`Cycle ${c+1} · ${j}/${keys}`);frames.at(-1).duration_ms=step;}
    // Step out back to home, three legs at a time.
    for(const group of TRIPODS){
      raise(group,true); add('Step out / lift');
      group.forEach(i=>{pose[i][0]=0;pose[i][1]=175;}); add('Step out / place');
      raise(group,false); add(group===TRIPODS[1]?'Home / end':'Step out / lower');
    }
    return {schema:'hexapod-motion-program-v1',name:`${motion.label} (${insect.label})`,motion:id,frames};
  }
  if(gait==='tripod'){
    // Tripod: each group swings half a stride ahead while the other, still on
    // the ground, pushes half a stride back, which moves the body. The last
    // half-step re-centres every foot so the cycle ends at home and repeats.
    const [a,b]=TRIPODS;
    raise(a,true); add('Tripod A / lift');
    a.forEach(i=>place(i,0.5));b.forEach(i=>place(i,-0.5)); add('Tripod A swing · B push');
    raise(a,false); add('Tripod A / lower');
    raise(b,true); add('Tripod B / lift');
    b.forEach(i=>place(i,0.5));a.forEach(i=>place(i,-0.5)); add('Tripod B swing · A push');
    raise(b,false); add('Tripod B / lower');
    raise(a,true); add('Tripod A / lift');
    a.forEach(i=>place(i,0));b.forEach(i=>place(i,0)); add('Re-centre · B push');
    raise(a,false); add('Home / end');
  } else {
    // Draft crawl: reposition one foot at a time, then drag the supported stance
    // back to home. Direction transforms extend the existing leg-angle model;
    // verify mounting and actual travel before trusting a direction label.
    for(const i of [0,5,1,3,2,4]) {
      raise([i],true); add(`${NAMES[i]} / lift`);
      place(i,1); add(`${NAMES[i]} / place`);
      raise([i],false); add(`${NAMES[i]} / lower`);
    }
    pose.splice(0,6,...home(height)); add('Supported stance / return');
  }
  return {schema:'hexapod-motion-program-v1',name:gait==='tripod'?`${motion.label} (tripod)`:motion.label,motion:id,frames};
}
export function parseProgram(text,offsets=DEFAULT_OFFSETS) {
  if(text.length>300000) throw new Error('Program file is too large.');
  const program=JSON.parse(text);
  if(program?.schema!=='hexapod-motion-program-v1'||typeof program.name!=='string'||program.name.length>80||!Array.isArray(program.frames)||program.frames.length<1||program.frames.length>128) throw new Error('Expected a motion program with 1–128 poses.');
  program.frames.forEach(frame=>validateFrame(frame,offsets));
  return {schema:program.schema,name:program.name,motion:typeof program.motion==='string'?program.motion:'custom',frames:program.frames.map(frame=>({name:frame.name,duration_ms:frame.duration_ms,positions:clone(frame.positions)}))};
}

export const POSTURES = [
  {id:'stand',label:'Stand',icon:'⇧'},
  {id:'sit',label:'Sit / crouch',icon:'⇩'}
];
export const DEFAULT_POSTURE_SETTINGS = {standHeight:70,sitHeight:30,duration:2000,sitTrim:[0,0,0,0,0,0],standTrim:[0,0,0,0,0,0]};
// Stand/Sit foot heights (Z, mm), including each leg's trim. Z is measured down
// from the hip, so a higher Stand pushes the feet further toward the ground.
export const POSTURE_HEIGHT = [-100,150];
// Per-leg Z trim (mm) for each posture. Positive lowers that foot, raising its
// corner of the body. Stand trims can be saved from the jog tool's touch-down.
export const TRIM_LIMIT = 40;
export function validatePostureSettings(settings) {
  const {standHeight,sitHeight,duration}=settings;
  const zeros=[0,0,0,0,0,0];
  const sitTrim=settings.sitTrim===undefined?zeros:settings.sitTrim,standTrim=settings.standTrim===undefined?zeros:settings.standTrim;
  const [low,high]=POSTURE_HEIGHT;
  if(![standHeight,sitHeight].every(v=>Number.isFinite(v)&&v>=low&&v<=high))throw new Error(`Stand and Sit heights must be ${low}–${high} mm.`);
  if(sitHeight>=standHeight)throw new Error('Sit height must be lower than Stand height.');
  if(!validDuration(duration))throw new Error(`Posture transition must be ${MIN_DURATION}–${MAX_DURATION} ms.`);
  for(const [label,trims] of [['Sit',sitTrim],['Stand',standTrim]])
    if(!Array.isArray(trims)||trims.length!==6||!trims.every(v=>Number.isFinite(v)&&Math.abs(v)<=TRIM_LIMIT))throw new Error(`${label} trim needs six values within ±${TRIM_LIMIT} mm.`);
  NAMES.forEach((name,i)=>{
    const sit=sitHeight+sitTrim[i],stand=standHeight+standTrim[i];
    if([sit,stand].some(z=>z<low||z>high))throw new Error(`${name} trimmed heights must stay within ${low}–${high} mm.`);
    if(sit>=stand)throw new Error(`${name} sit height must stay below its Stand height.`);
  });
  return {standHeight,sitHeight,duration,sitTrim:[...sitTrim],standTrim:[...standTrim]};
}
function posturePose(id,{standHeight,sitHeight,sitTrim,standTrim}) {
  return id==='stand'?standTrim.map(trim=>[0,175,standHeight+trim]):sitTrim.map(trim=>[0,175,sitHeight+trim]);
}
export function postureTarget(id,settings=DEFAULT_POSTURE_SETTINGS,offsets=DEFAULT_OFFSETS) {
  if(!POSTURES.some(p=>p.id===id))throw new Error('Unknown posture.');
  const valid=validatePostureSettings(settings);
  return validateFrame({name:id==='stand'?'Stand':'Sit / lowered crouch',positions:posturePose(id,valid),duration_ms:valid.duration},offsets);
}
export function generatePosture(id,offsets=DEFAULT_OFFSETS,settings=DEFAULT_POSTURE_SETTINGS) {
  if(!POSTURES.some(p=>p.id===id))throw new Error('Unknown posture.');
  const valid=validatePostureSettings(settings);
  const poses=id==='stand'?['sit','stand']:['stand','sit'];
  const frames=poses.map((pose,i)=>validateFrame({name:i===0?'Starting posture':id==='stand'?'Stand':'Sit / lowered crouch',positions:posturePose(pose,valid),duration_ms:i===0?500:valid.duration},offsets));
  return {schema:'hexapod-motion-program-v1',name:id==='stand'?'Stand':'Sit / crouch',motion:id,frames};
}

export const MAX_FRAMES = 128;
// Join another movement onto the end of a sequence. A template's first pose is
// only an assumed starting posture, so it is dropped (dropStart) when joining.
export function appendProgram(program,addition,{dropStart=false}={},offsets=DEFAULT_OFFSETS) {
  const frames=clone(addition.frames.slice(dropStart&&addition.frames.length>1?1:0));
  if(program.frames.length+frames.length>MAX_FRAMES)throw new Error(`Joining would make ${program.frames.length+frames.length} poses; the maximum is ${MAX_FRAMES}.`);
  frames.forEach(frame=>validateFrame(frame,offsets));
  return {...clone(program),motion:'custom',frames:[...clone(program.frames),...frames]};
}
// Playback speed: divide every transition time, keeping the range the board
// accepts. Reports how many poses hit that range so the UI can say so.
export const GAIT_SPEEDS = [0.25,0.5,0.75,1,1.5,2,3,4];
export function scaleDurations(frames,speed) {
  if(!GAIT_SPEEDS.includes(speed))throw new Error('Unsupported playback speed.');
  let clamped=0;
  const scaled=frames.map(frame=>{const ideal=Math.round(frame.duration_ms/speed),duration_ms=Math.max(MIN_DURATION,Math.min(MAX_DURATION,ideal));if(duration_ms!==ideal)clamped++;return {...frame,duration_ms};});
  return {frames:scaled,clamped};
}
// Set every pose's transition time at once (for speeding up or slowing down a whole sequence).
export function setAllDurations(program,duration) {
  if(!validDuration(duration))throw new Error(`Transition time must be a whole number of ${MIN_DURATION}–${MAX_DURATION} ms.`);
  return {...clone(program),frames:program.frames.map(frame=>({...clone(frame),duration_ms:duration}))};
}
// Live walking computed on the board (motion_bridge.py walk_pose): the board
// runs inverse kinematics every 20 ms along these continuous foot paths.
// Must match walk_pose() exactly (checked by tests/walk_fixture.json).
export const WALK_GAITS = {tripod:[1/2,[0,1/2,0,1/2,0,1/2]],ripple:[2/3,[2/3,1/3,0,1/6,5/6,1/2]],wave:[5/6,[2/6,1/6,0,5/6,4/6,3/6]]};
const LEG_ANGLES=[30,0,-30,150,180,-150].map(v=>v*Math.PI/180);
export const WALK_RAMP = 1;
export function walkPose(base,{gait,direction,stride,lift,hipSwing=0},amplitude,t) {
  const [stance,phases]=WALK_GAITS[gait],[forward,lateral,turn]=direction,swing=1-stance;
  return base.map(([bx,by,bz],i)=>{
    const p=((t-phases[i])%1+1)%1;let k,up=0;
    if(p<swing){const s=p/swing;k=-0.5+(1-Math.cos(Math.PI*s))/2;up=Math.sin(Math.PI*s);}
    else k=0.5-(p-swing)/stance;
    k*=amplitude;
    const th=LEG_ANGLES[i],across=forward*Math.cos(th)-lateral*Math.sin(th)+turn,along=forward*Math.sin(th)+lateral*Math.cos(th);
    let x,y;
    if(hipSwing>0){const sw=hipSwing*Math.PI/180,reach=Math.hypot(bx,by)+k*along*175*sw,angle=Math.atan2(bx,by)+k*across*sw;x=reach*Math.sin(angle);y=reach*Math.cos(angle);}
    else{x=bx+k*stride*across;y=by+k*stride*along;}
    return [x,y,bz-amplitude*lift*up];
  });
}
const smoothstep=x=>{x=Math.max(0,Math.min(1,x));return x*x*(3-2*x);};
export function walkAmplitude(cycle,stopAt) {
  let amplitude=smoothstep(cycle/WALK_RAMP);
  if(stopAt!==null)amplitude=Math.min(amplitude,smoothstep(1-(cycle-stopAt)/WALK_RAMP));
  return amplitude;
}
export function validateWalk({gait,direction,stride,lift,hipSwing=0,period,cycles=0}) {
  if(!Object.hasOwn(WALK_GAITS,gait))throw new Error('Gait must be tripod, ripple or wave.');
  if(!Array.isArray(direction)||direction.length!==3||!direction.every(v=>Number.isFinite(v)&&Math.abs(v)<=1)||!direction.some(Boolean))throw new Error('Choose a walking direction.');
  if(!(stride>=5&&stride<=150))throw new Error('Stride must be 5–150 mm.');
  if(!(lift>=0&&lift<=60))throw new Error('Step height must be 0–60 mm.');
  if(!(hipSwing>=0&&hipSwing<=90))throw new Error('Stride angle must be 0–90°.');
  if(!Number.isInteger(period)||period<400||period>20000)throw new Error('Cycle time must be 400–20000 ms.');
  if(!Number.isInteger(cycles)||cycles<0||cycles>100)throw new Error('Cycles must be 0 (until stopped) to 100.');
}
// Check the whole walk from a stance: reach and joint limits like the board,
// plus the app's collision check, at several speeds through the ramp.
export function checkWalk(base,params,offsets=DEFAULT_OFFSETS) {
  validateWalk(params);
  for(const amplitude of [0.25,0.5,0.75,1])for(let n=0;n<96;n++){const pose=walkPose(base,params,amplitude,n/96);anglesForPose(pose,offsets);checkCollisions(pose);}
}
// Poses for previewing a walk in the app (the robot itself moves continuously).
export function walkPreviewFrames(base,params,cycles=3) {
  const keys=24,stopAt=WALK_RAMP+cycles,total=Math.round((stopAt+WALK_RAMP)*keys),frames=[];
  const duration=Math.max(MIN_DURATION,Math.round(params.period/keys));
  for(let n=1;n<=total;n++){const cycle=n/keys;frames.push({name:`Walk ${cycle.toFixed(2)}`,duration_ms:duration,positions:walkPose(base,params,n===total?0:walkAmplitude(cycle,stopAt),cycle%1)});}
  return frames;
}
// Live walk figures for the panel: each foot's step along its arc (mm) at the
// home reach, and the body's ground speed (mm/s) for a gait and speed.
export function walkFigures({gait,hipSwing,speed}) {
  const step=175*hipSwing*Math.PI/180,stance=WALK_GAITS[gait][0];
  return {step,speed:step/stance*speed};
}
