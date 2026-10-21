import {mountMotionScene,balance} from './scene.bundle.js';
import {NAMES,LIMITS,JOINT_LIMITS,DEFAULT_OFFSETS,MOTIONS,POSTURES,generatePosture,DEFAULT_POSTURE_SETTINGS,validatePostureSettings,postureTarget,clone,home,rotateFoot,anglesForPose,checkCollisions,setBoardLimits,generate,validateFrame,parseProgram,appendProgram,scaleDurations,setAllDurations,MAX_FRAMES,checkWalk,walkPreviewFrames,walkFigures} from './model.js';
import {SerialLink} from './transport.js';
import {Player,preflight} from './player.js';
import {SimulatedLink} from './simulator.js';
const $=id=>document.getElementById(id);
const TRIM_IDS=[0,1,2,3,4,5].map(i=>'sit-trim-'+i),STAND_TRIM_IDS=[0,1,2,3,4,5].map(i=>'stand-trim-'+i);
const LIMIT_IDS=[0,1,2].map(j=>[`limit-${j}-min`,`limit-${j}-max`]);
const JOG_IDS=['jog-up','jog-down','jog-x-minus','jog-x-plus','jog-y-minus','jog-y-plus','jog-hip-minus','jog-hip-plus','save-stand-trim','jog-capture'],SERVO_IDS=['servo-minus5','servo-minus1','servo-plus1','servo-plus5'];
const PLAYBACK_IDS=['repeat-count','repeat-forever','gait-speed','ease-smooth'];
const BATTERY_WARN='hexapod-battery-warn-v1';
// Live walk directions: [forward, sideways left, turn left], each -1..1.
const WALK_DIRECTIONS={'walk-forward':[1,0,0],'walk-back':[-1,0,0],'walk-left':[0,1,0],'walk-right':[0,-1,0],'walk-turn-left':[0,0,1],'walk-turn-right':[0,0,-1],'walk-curve-left':[1,0,0.5],'walk-curve-right':[1,0,-0.5]};
let walkBase=null,tab='motion',calDirty=false,batteryWarned=false,passLabel='',offsets=clone(DEFAULT_OFFSETS),program=generate('forward'),index=0,link=null,live=null,player=null,busy=false,polling=false,pendingStops=0,previewSession=false,dirty=false,library={};
const history=[],STORAGE='hexapod-motion-library-v1';
function message(text,error=false){$('notice').textContent=text;$('notice').classList.toggle('error',error);}
function log(text){history.push(new Date().toLocaleTimeString()+'  '+text);$('activity').textContent=history.slice(-30).join('\n');}
function frame(){return program.frames[index];}
const DRAFT='hexapod-motion-draft-v1',SETTINGS='hexapod-posture-settings-v1';
let undoStack=[],redoStack=[],snapshot={program:clone(program),index},postureSettings=clone(DEFAULT_POSTURE_SETTINGS);
function persistDraft(){
  try{localStorage.setItem(DRAFT,JSON.stringify({program,index,offsets}));$('draft-status').textContent='Draft saved automatically';}
  catch{$('draft-status').textContent='Draft storage unavailable — export JSON to keep edits';}
}
function changed(){
  // Keep the previous complete sequence so replacement actions are undoable too.
  undoStack.push(snapshot);if(undoStack.length>30)undoStack.shift();redoStack=[];
  snapshot={program:clone(program),index};dirty=true;
  $('saved-status').textContent='Unsaved sequence edits';persistDraft();
}
function restoreEdit(from,to){
  if(busy||pendingStops||player?.running||!from.length)return;
  stopPreview();to.push({program:clone(program),index});
  const prior=from.pop();program=clone(prior.program);index=prior.index;
  snapshot={program:clone(program),index};dirty=true;persistDraft();render();
  $('saved-status').textContent='Unsaved sequence edits';
  message('Edit restored. No robot commands were sent.');
}
function showReported(pose,duration=150){
  $('scene-source').textContent='BOARD-REPORTED COMMAND';$('pose-title').textContent='Board-reported pose';
  motionScene.setPose(pose,{duration,ground:Number($('height').value)||70});
}
function renderReported(){
  $('reported-values').replaceChildren();
  if(!live&&$('scene-source').textContent==='BOARD-REPORTED COMMAND')$('scene-source').textContent='LAST COMMAND · DISCONNECTED';
  $('reported-status').textContent=live?.positions?'Last command reported by the board · not sensor feedback':'No board-reported pose available';
  if(!live?.positions)return;
  NAMES.forEach((name,i)=>{const row=document.createElement('tr');
    [name,...live.positions[i].map(v=>v.toFixed(1))].forEach(value=>{const cell=document.createElement('td');cell.textContent=value;row.append(cell);});
    $('reported-values').append(row);
  });
}
const motionScene=mountMotionScene($('robot'), {
  onFrame:i=>{index=i;render();},
  onPlayback:mode=>{$('timeline').classList.toggle('playing',mode==='playing');$('preview').textContent=mode==='playing'?'Ⅱ Pause preview':mode==='paused'?'▷ Resume preview':mode==='ended'?'↻ Replay sequence':'▷ Preview sequence';}
});
function stopPreview(){previewSession=false;motionScene.stop();}

function hardware(){
  const online=!!link&&!!live,running=!!player?.running,locked=busy||pendingStops>0;
  const simulated=!!link?.simulated,walking=!!live?.walking,placed=online&&live.active&&!!live.positions&&!walking,released=live?.released||[];
  $('connect').textContent=online?(simulated?'Stop simulation':'Disconnect ↗'):'Connect robot ↗';$('connect').disabled=locked||running;
  $('simulate').disabled=online||locked;
  $('connection').textContent=online?(simulated?'Simulated robot':'Servo 2040 connected'):'Offline planning';
  $('status-detail').textContent=online?(simulated?'No USB · simulated training bridge':'USB · training bridge active'):'Preview and edit without a robot';$('dot').classList.toggle('live',online);
  // Enable at home is also how to recover after a joint test or a released leg (position unknown).
  $('arm').disabled=!online||locked||running||placed||walking;
  // Directions start a walk from a held stance, or steer one that is running.
  Object.keys(WALK_DIRECTIONS).forEach(id=>$(id).disabled=!(placed||walking&&!live.walking.stopping)||locked||running);
  $('walk-stop').disabled=!walking||!!live.walking.stopping;
  for(const id of ['walk-gait','walk-cycles','walk-preview'])$(id).disabled=walking||locked||running;
  for(const id of ['walk-lift','walk-speed','walk-angle'])$(id).disabled=locked||running||!!live?.walking?.stopping;
  $('send').disabled=!placed||locked||running;
  for(const id of ['stand','sit'])$(id).disabled=!placed||locked||running;
  $('play').disabled=!placed||locked||running;
  JOG_IDS.forEach(id=>$(id).disabled=!placed||locked||running);
  SERVO_IDS.forEach(id=>$(id).disabled=!online||!live.active||!live.angles||locked||running);
  for(const id of ['servo-set-min','servo-set-max'])$(id).disabled=!live?.angles;
  $('save-limits').disabled=!online||locked||running;
  $('release-leg-button').disabled=!online||!live.active||pendingStops>0;
  const servo=live?.angles?.[Number($('servo-leg').value)||0]?.[Number($('servo-joint').value)||0];
  $('servo-angle').textContent=Number.isFinite(servo)?servo.toFixed(1)+'°':'—';
  $('pause').disabled=!online;$('off').disabled=!online;
  $('capture').disabled=!online||!live.positions||running||locked;
  $('hardware-state').textContent=(simulated?'Simulation · ':'')+(!online?'Plan first. Move when ready.':walking?`Walking · ${live.walking.gait} · cycle ${Math.floor(live.walking.cycle)+1}${live.walking.stopping?' · stopping':''}`:live.moving?'Moving to the next pose…'+passLabel:!live.active?'Motors disabled':released.length?`${released.map(i=>NAMES[i]).join(', ')} released · Enable at home to move again`:!live.positions?'Joint test · Enable at home before moving':'Motors enabled · holding pose');
  $('live-info').textContent=live?.positions?`Last commanded L1: X ${live.positions[0][0].toFixed(1)}, Y ${live.positions[0][1].toFixed(1)}, Z ${live.positions[0][2].toFixed(1)} mm. Capture to inspect all six legs.`:'No live command data. Angles and positions are model targets, not feedback sensors.';
  document.querySelectorAll('.pose-input,#frame-name,#duration,#program-name,#stride,#lift,#height,#hip-swing,#gait,#cycles,#smoothness,#generate,#duplicate,#remove,#new-sequence,#import,#load,#delete-saved,[data-motion],.frame').forEach(el=>el.disabled=locked||running);
  $('preview').disabled=locked||running;
  $('undo').disabled=locked||running||!undoStack.length;$('redo').disabled=locked||running||!redoStack.length;
  for(const id of ['stand-height','sit-height','posture-duration',...TRIM_IDS,...STAND_TRIM_IDS,...PLAYBACK_IDS,'join-template','join-saved','all-duration','apply-all-duration'])$(id).disabled=locked||running;
  renderReported();renderCalibration();renderBattery();
}
function applyState(state){
  live={...live,...state};
  if(state.joint_limits)try{setBoardLimits(clone(LIMITS),state.joint_limits);}catch{}
  // Calibration changes the board's offsets live; check later poses against them.
  if(state.offsets)offsets=clone(state.offsets);
  hardware();
  if((player?.running||state.walking)&&state.positions)showReported(state.positions,state.walking?200:150);
}
function renderScene(){
  if(player?.running)return;
  $('scene-source').textContent=previewSession?'SEQUENCE PREVIEW':'SELECTED TARGET';
  if(!previewSession)motionScene.setPose(frame().positions,{ground:Number($('height').value)||70});
}

function renderAngles(){
  let angles;try{angles=anglesForPose(frame().positions,offsets);checkCollisions(frame().positions);$('validity').textContent='VALID TARGET';$('validity').classList.remove('invalid');}
  catch(error){$('validity').textContent='CHECK POSE';$('validity').classList.add('invalid');message(error.message,true);}
  $('angle-values').replaceChildren();NAMES.forEach((name,i)=>{const cell=document.createElement('div');cell.className='angle-cell';const title=document.createElement('strong');title.textContent=name;const values=document.createElement('span');values.textContent=angles?angles[i].map(v=>v.toFixed(1)+'°').join(' / '):'Outside limits';cell.append(title,values);$('angle-values').append(cell);});
}
function renderTimeline(){
  $('timeline').replaceChildren();program.frames.forEach((pose,i)=>{const button=document.createElement('button');button.className='frame'+(i===index?' current':'');if(i===index)currentFrameButton=button;button.setAttribute('aria-pressed',String(i===index));const number=document.createElement('b');number.textContent=String(i+1).padStart(2,'0');const name=document.createElement('strong');name.textContent=pose.name;const duration=document.createElement('small');duration.textContent=(pose.duration_ms/1000).toFixed(1)+' sec';button.append(number,name,duration);button.onclick=()=>{if(player?.running)return;stopPreview();index=i;render();};$('timeline').append(button);});
  followCurrentPose();
  $('total').textContent=`${program.frames.length} POSES · ${(program.frames.reduce((sum,f)=>sum+f.duration_ms,0)/1000).toFixed(1)} SECONDS`;
}
// Keep the current pose centred in the timeline strip during preview or playback.
// Scrolls only the strip itself, never the page.
let currentFrameButton=null;
function followCurrentPose(){
  const box=$('timeline'),current=currentFrameButton;
  if(!current?.getBoundingClientRect||!box.scrollTo)return;
  const b=box.getBoundingClientRect(),c=current.getBoundingClientRect();
  box.scrollTo({left:box.scrollLeft+(c.left-b.left)-(b.width-c.width)/2,behavior:'smooth'});
}
function render(){
  $('pose-title').textContent=frame().name;$('frame-badge').textContent=String(index+1).padStart(2,'0')+' / '+program.frames.length;
  $('frame-name').value=frame().name;$('duration').value=frame().duration_ms;$('program-name').value=program.name;
  NAMES.forEach((name,i)=>[0,1,2].forEach(j=>$(`xyz-${i}-${j}`).value=Number(frame().positions[i][j].toFixed(2))));
  document.querySelectorAll('[data-motion]').forEach(button=>button.classList.toggle('chosen',button.dataset.motion===program.motion));
  renderAngles();renderScene();renderTimeline();hardware();
}
function parameters(){return {stride:Number($('stride').value),lift:Number($('lift').value),height:Number($('height').value),duration:Number($('duration').value),hipSwing:Number($('hip-swing').value)||0,gait:$('gait').value||'crawl',cycles:Number($('cycles').value)||3,smoothness:Number($('smoothness').value)||1};}
function build(id){if(player?.running||busy||pendingStops)return;stopPreview();try{const next=POSTURES.some(p=>p.id===id)?generatePosture(id,offsets,postureSettings):generate(id,parameters(),offsets);
  // Joining drops the template's assumed starting pose; the joined poses start from the current last pose.
  if($('join-template').checked){const before=program.frames.length;program=appendProgram(program,next,{dropStart:true},offsets);index=before;changed();render();message(`${next.name} joined: ${program.frames.length-before} poses added. Check the transition into pose ${before+1}.`);return;}
  program=next;index=0;changed();render();
  // Warn about poses where the body centre falls outside the feet on the ground.
  const tipping=program.frames.map((f,i)=>balance(f.positions).stable?null:i+1).filter(Boolean);
  if(tipping.length)message(`Template built, but ${tipping.length} pose(s) may tip the robot (pose ${tipping.slice(0,5).join(', ')}${tipping.length>5?'…':''}). Use a shorter stride or a gait with more feet down.`,true);
  else message('Draft template built. Verify each pose and actual movement direction before playback.');}catch(error){message(error.message,true);}}
// Playback settings shared by the robot buttons and the preview.
function playbackOptions(){
  const count=Math.round(Number($('repeat-count').value)||1);
  return {repeat:$('repeat-forever').checked?Infinity:Math.max(1,Math.min(99,count)),speed:Number($('gait-speed').value)||1,ease:$('ease-smooth').checked?'smooth':'linear'};
}
function playbackFrames(frames,options=playbackOptions()){
  const scaled=scaleDurations(frames,options.speed);
  return {frames:scaled.frames.map(f=>options.ease==='smooth'?{...f,ease:'smooth'}:f),clamped:scaled.clamped};
}
[...MOTIONS,...POSTURES].forEach(motion=>{const button=document.createElement('button');button.dataset.motion=motion.id;button.innerHTML=`<span class="icon">${motion.icon}</span><span>${motion.label}</span>`;button.onclick=()=>build(motion.id);$('motions').append(button);});
NAMES.forEach((name,i)=>{const row=document.createElement('tr');const label=document.createElement('td');label.textContent=name;row.append(label);[0,1,2].forEach(j=>{const cell=document.createElement('td'),input=document.createElement('input');input.id=`xyz-${i}-${j}`;input.className='pose-input';input.type='number';input.min=LIMITS[j][0];input.max=LIMITS[j][1];input.step='1';input.setAttribute('aria-label',`${name} ${['X','Y','Z'][j]} millimetres`);input.onchange=()=>{stopPreview();const updated=clone(frame());updated.positions[i][j]=Number(input.value);try{if(input.value==='')throw new Error('Enter a coordinate.');validateFrame(updated,offsets);program.frames[index]=updated;changed();render();message('Pose edited in preview. Use Send selected pose to move the robot.');}catch(error){message(error.message,true);render();}};cell.append(input);row.append(cell);});$('positions').append(row);});
$('generate').onclick=()=>build([...MOTIONS,...POSTURES].some(m=>m.id===program.motion)?program.motion:'forward');
$('frame-name').onchange=()=>{stopPreview();frame().name=$('frame-name').value.trim().slice(0,80)||'Untitled pose';changed();render();};
$('duration').onchange=()=>{stopPreview();const updated=clone(frame());updated.duration_ms=Number($('duration').value);try{validateFrame(updated,offsets);program.frames[index]=updated;changed();}catch(error){message(error.message,true);}render();};
$('apply-all-duration').onclick=()=>{
  if(busy||pendingStops||player?.running)return;stopPreview();
  try{const duration=Number($('all-duration').value);program=setAllDurations(program,duration);changed();render();message(`Every pose now takes ${duration} ms (${(duration*program.frames.length/1000).toFixed(1)} s in total). Undo restores the previous times.`);}
  catch(error){message(error.message,true);}
};
$('program-name').onchange=()=>{program.name=$('program-name').value.trim().slice(0,80)||'My movement';changed();hardware();};
$('duplicate').onclick=()=>{if(program.frames.length>=MAX_FRAMES){message('Maximum 128 poses per movement.',true);return;}stopPreview();const added=clone(frame());added.name=(added.name+' / copy').slice(0,80);program.frames.splice(index+1,0,added);index++;changed();render();};
$('remove').onclick=()=>{stopPreview();if(program.frames.length===1){message('Keep at least one pose.',true);return;}program.frames.splice(index,1);index=Math.min(index,program.frames.length-1);changed();render();};
$('new-sequence').onclick=()=>{
  if(busy||pendingStops||player?.running)return;
  stopPreview();
  try{
    const first={name:'Home / start',positions:home(Number($('height').value)||70),duration_ms:700};
    validateFrame(first,offsets);
    // changed() keeps the previous sequence on the undo stack, so nothing is lost.
    program={schema:'hexapod-motion-program-v1',name:'New movement',motion:'custom',frames:[first]};
    index=0;changed();render();
    $('program-name').focus?.();$('program-name').select?.();
    message('New sequence started with a home pose. Edit it, then use Duplicate pose to add more poses. Undo brings back the previous sequence.');
  }catch(error){message(error.message,true);}
};
$('capture').onclick=()=>{if(!live?.positions)return;if(program.frames.length>=128){message('Maximum 128 poses.',true);return;}stopPreview();const captured={name:'Captured commanded pose',positions:clone(live.positions),duration_ms:700};try{validateFrame(captured,offsets);program.frames.push(captured);index=program.frames.length-1;changed();render();message('Captured the last board-reported command for all six legs. This is not a sensor measurement.');}catch(error){message(error.message,true);}};
$('preview').onclick=()=>{
  if(busy||pendingStops||player?.running)return;
  if(previewSession){
    if(motionScene.getSnapshot().playback==='playing')motionScene.pause();
    else motionScene.resume();
    return;
  }
  let frames;
  try{program.frames.forEach(f=>validateFrame(f,offsets));frames=playbackFrames(program.frames).frames;}catch(error){message(error.message,true);return;}
  previewSession=true;
  motionScene.playSequence(frames,{ground:Number($('height').value)||70});
  message('Animating between commanded pose targets at the playback speed and easing. This preview sends no commands to the robot.');
};

function refreshLibrary(){const selected=$('library').value;$('library').replaceChildren();const empty=document.createElement('option');empty.value='';empty.textContent='Choose a saved movement…';$('library').append(empty);Object.keys(library).forEach(key=>{const option=document.createElement('option');option.value=key;option.textContent=key;$('library').append(option);});if(library[selected])$('library').value=selected;}
try{const saved=JSON.parse(localStorage.getItem(STORAGE)||'{}');if(saved&&typeof saved==='object'&&!Array.isArray(saved))library=saved;}catch{message('Browser storage is unavailable. Use Export JSON to keep your movement.',true);}
$('save').onclick=()=>{try{parseProgram(JSON.stringify(program),offsets);if(!Object.hasOwn(library,program.name)&&Object.keys(library).length>=20)throw new Error('Keep up to 20 saved movements. Export or delete one first.');const next={...library,[program.name]:clone(program)};localStorage.setItem(STORAGE,JSON.stringify(next));library=next;dirty=false;refreshLibrary();$('saved-status').textContent='Saved in this browser';message('Movement saved in this browser. Export JSON for a portable backup.');}catch(error){message(error.message,true);}};
$('load').onclick=()=>{const selected=$('library').value;if(!Object.hasOwn(library,selected))return;try{const next=parseProgram(JSON.stringify(library[selected]),offsets);stopPreview();program=next;index=0;changed();render();$('saved-status').textContent='Loaded from this browser';}catch(error){message(error.message,true);}};
$('join-saved').onclick=()=>{const selected=$('library').value;if(!Object.hasOwn(library,selected)||busy||pendingStops||player?.running)return;try{const addition=parseProgram(JSON.stringify(library[selected]),offsets);stopPreview();const before=program.frames.length;program=appendProgram(program,addition,{},offsets);index=before;changed();render();message(`${selected} joined: ${addition.frames.length} poses added. Check the transition into pose ${before+1}.`);}catch(error){message(error.message,true);}};
$('delete-saved').onclick=()=>{const selected=$('library').value;if(!Object.hasOwn(library,selected))return;try{const next={...library};delete next[selected];localStorage.setItem(STORAGE,JSON.stringify(next));library=next;refreshLibrary();message('Saved copy deleted. The current open sequence is unchanged.');}catch(error){message(error.message,true);}};
function download(text,type,extension){const url=URL.createObjectURL(new Blob([text],{type})),a=document.createElement('a');a.href=url;a.download=(program.name.replace(/[^a-z0-9_-]/gi,'_')||'movement')+'.'+extension;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
$('export').onclick=()=>{try{program.frames.forEach(f=>validateFrame(f,offsets));download(JSON.stringify({...program,coordinate_system:'leg-local XYZ mm; commanded targets, not measurements',calibration_offsets:offsets,frames:program.frames.map(f=>({...f,angles:anglesForPose(f.positions,offsets)}))},null,2),'application/json','json');}catch(error){message(error.message,true);}};
$('csv').onclick=()=>{try{const lines=['frame,pose,transition_ms,leg,x_mm,y_mm,z_mm,hip_deg,upper_deg,lower_deg'];program.frames.forEach((f,i)=>{const a=anglesForPose(f.positions,offsets);const label='"'+f.name.replace(/"/g,'""').replace(/^[=+@-]/,"'$&")+'"';NAMES.forEach((name,j)=>lines.push([i+1,label,f.duration_ms,name,...f.positions[j],...a[j].map(v=>v.toFixed(3))].join(',')));});download(lines.join('\r\n'),'text/csv','csv');}catch(error){message(error.message,true);}};
$('import').onchange=async event=>{try{const file=event.target.files[0];if(!file)return;if(file.size>300000)throw new Error('Program file is too large.');const text=await file.text();
  // Bug: file reads can finish after playback starts. Recheck before replacing its timeline.
  if(busy||pendingStops||player?.running)throw new Error('Stop playback and finish the current command before importing a movement.');
  const next=parseProgram(text,offsets);stopPreview();program=next;index=0;changed();render();message('Movement imported. No commands were sent to the robot.');}catch(error){message(error.message,true);}finally{event.target.value='';}};
async function disconnected(error,source=link){if(link!==source)return;player?.cancel();const old=source;link=null;live=null;busy=false;hardware();message(error.message+' If the training bridge is running, its three-second timeout releases the motors. Output state cannot be confirmed while disconnected.',true);log(error.message);if(old)await old.close();}
async function command(op,args={},success=op==='arm'?'Robot enabled at home. Preview the movement before playing it.':'Command acknowledged.'){if(!link||busy||pendingStops||player?.running)return false;stopPreview();busy=true;hardware();try{applyState(await link.send(op,args));if(live.positions)showReported(live.positions);message(success);return true;}catch(error){message(error.message,true);return false;}finally{busy=false;hardware();}}
function renderLimits(){LIMIT_IDS.forEach(([min,max],j)=>{$(min).value=JOINT_LIMITS[j][0];$(max).value=JOINT_LIMITS[j][1];});}
$('connect').onclick=()=>connect(SerialLink);
$('simulate').onclick=()=>connect(SimulatedLink);
// Leg calibration tab: ported from the calibration web app, using the same
// connection. One leg holds the home reference pose while its offsets change.
const JOINT_LABELS=['Hip rotation','Upper leg','Lower leg'];
function setTab(next){
  tab=next;if(document.body)document.body.dataset.tab=next;
  $('tab-motion').classList.toggle('active',next==='motion');$('tab-calibrate').classList.toggle('active',next==='calibrate');
  try{localStorage.setItem('hexapod-tab-v1',next);}catch{}
  if(next==='calibrate'&&link&&live&&!live.calibration)message('The board’s motion_bridge.py is too old for calibration. Copy the current Board Files to the board.',true);
}
$('tab-motion').onclick=()=>setTab('motion');$('tab-calibrate').onclick=()=>setTab('calibrate');
// Servo power from the Servo 2040's voltage/current sensors (reported with every board reply).
function renderBattery(){
  const power=live?.power,warn=Number($('battery-warn').value)||0,low=!!power&&warn>0&&power.voltage<warn;
  $('battery').textContent=!link||!live?'Battery · not connected':power?`Battery ${power.voltage.toFixed(2)} V · ${power.current.toFixed(2)} A${low?' · LOW':''}`:'Battery · not reported by this board';
  $('battery').classList.toggle('low',low);
  if(low&&!batteryWarned){batteryWarned=true;message(`Low battery: ${power.voltage.toFixed(2)} V is below ${warn} V. Stop and recharge before the servos brown out.`,true);log(`Low battery ${power.voltage.toFixed(2)} V`);}
  if(!low)batteryWarned=false;
}
$('battery-warn').onchange=()=>{const value=Number($('battery-warn').value);if(!Number.isFinite(value)||value<0){$('battery-warn').value='';}try{localStorage.setItem(BATTERY_WARN,$('battery-warn').value);}catch{}renderBattery();};
try{const saved=localStorage.getItem(BATTERY_WARN);if(saved)$('battery-warn').value=saved;}catch{}
function renderCalibration(){
  const cal=live?.calibration,ready=!!link&&!!cal,locked=busy||pendingStops>0||!!player?.running,leg=cal?.leg??null,chosen=leg!==null;
  const targets=cal?.targets,max=cal?.max_adjustment??15;
  NAMES.forEach((name,i)=>{$(`cal-leg-${i}`).disabled=!ready||locked;$(`cal-leg-${i}`).classList.toggle('chosen',leg===i);});
  $('cal-selected-name').textContent=chosen?`${NAMES[leg].startsWith('L')?'Left':'Right'} leg ${NAMES[leg][1]}`:'Choose a leg';
  $('cal-channels').textContent=chosen&&live.channels?`Board connectors ${live.channels[leg].map(pin=>pin+1).join(' / ')}`:ready?'Select a leg to begin.':link&&live?'This board’s motion_bridge.py is too old for calibration. Copy the current Board Files to the board, restart it, and reconnect.':'Click Connect robot (or Simulate robot) first, then select a leg.';
  $('cal-mode').textContent=!chosen?'NO LEG SELECTED':cal.holding?'HOLDING POSITION':'PREVIEW · OUTPUT OFF';
  $('cal-move').disabled=!ready||!chosen||locked||cal.holding||!targets;$('cal-move').textContent=cal?.holding?'Leg enabled':'Enable leg & apply';
  JOINT_LABELS.forEach((label,j)=>{
    const angle=targets?.[j],offset=chosen?offsets[leg][j]:null,range=$(`cal-range-${j}`),[low,high]=JOINT_LIMITS[j];
    $(`cal-angle-${j}`).textContent=Number.isFinite(angle)?`${angle.toFixed(1)}°`:'—°';
    $(`cal-offset-${j}`).textContent=Number.isFinite(offset)?`Offset ${offset>=0?'+':''}${offset.toFixed(1)}°`:'Offset —';
    range.min=Math.max(low,(angle??0)-max).toFixed(2);range.max=Math.min(high,(angle??0)+max).toFixed(2);range.value=angle??0;
    range.disabled=!ready||!chosen||locked||!Number.isFinite(angle);
    range.title=cal?.holding?'Release to move this joint':'Preview only; enable the leg to apply';
    $(`cal-minus-${j}`).disabled=range.disabled||angle-1<low;$(`cal-plus-${j}`).disabled=range.disabled||angle+1>high;
  });
  const reference=cal?.reference;['x','y','z'].forEach((axis,i)=>$(`cal-${axis}`).textContent=reference?.[i]??'—');
  const power=live?.power;$('cal-power').textContent=power?`${power.voltage.toFixed(1)} V · ${power.current.toFixed(2)} A`:'not reported';
  $('cal-save').disabled=$('cal-download').disabled=!ready||locked;$('cal-off').disabled=!link||!live;
  $('cal-save-label').textContent=calDirty?'You have unsaved adjustments.':'Ready when you are.';
}
NAMES.forEach((name,i)=>$(`cal-leg-${i}`).onclick=async()=>{if(await command('cal_select',{leg:i},`${name} selected and all servos off. Adjust in preview, then Enable leg & apply. The first movement can jump.`))render();});
$('cal-move').onclick=()=>command('cal_move',{},'Leg holding its reference pose; the other legs are off. Adjust one joint at a time against your physical reference. Use Enable at home before moving in Motion studio.');
async function calAdjust(joint,delta){
  const holding=live?.calibration?.holding;
  if(await command('cal_adjust',{joint,delta},holding?'Joint angle applied. The selected leg is enabled.':'Preview updated. Motors are off. Click Enable leg & apply to move to these angles.')){calDirty=true;log(`${NAMES[live.calibration.leg]} joint ${joint+1}: ${delta>0?'+':''}${delta.toFixed(2)}°`);render();}
}
JOINT_LABELS.forEach((label,j)=>{
  $(`cal-minus-${j}`).onclick=()=>calAdjust(j,-1);$(`cal-plus-${j}`).onclick=()=>calAdjust(j,1);
  // Show the requested angle during the drag; send one command on release.
  $(`cal-range-${j}`).oninput=event=>{$(`cal-angle-${j}`).textContent=`${Number(event.target.value).toFixed(1)}°`;};
  $(`cal-range-${j}`).onchange=event=>{const target=live?.calibration?.targets?.[j];if(!Number.isFinite(target))return;const max=live.calibration.max_adjustment??15;return calAdjust(j,Math.max(-max,Math.min(max,Number(event.target.value)-target)));};
});
$('cal-save').onclick=async()=>{if(await command('cal_save',{},'Calibration saved on the robot. It will load automatically after a restart.')){calDirty=false;renderCalibration();}};
$('cal-download').onclick=()=>{
  if(!live?.offsets)return;
  const data=Object.fromEntries(NAMES.map((name,i)=>[name+'o',live.offsets[i]]));
  const url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)+'\n'],{type:'application/json'})),a=document.createElement('a');
  a.href=url;a.download='calibration_offsets.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);log('Downloaded current offsets');
};
$('cal-off').onclick=()=>stop('off');
// Live walk: the board computes the foot paths and inverse kinematics itself.
// Panel settings: step height (mm), speed (steps per second, one step per leg
// per cycle) and stride angle (hip rotation per step, degrees).
function walkSettings(){
  const lift=Number($('walk-lift').value),speed=Number($('walk-speed').value),angle=Number($('walk-angle').value);
  return {lift:Number.isFinite(lift)&&$('walk-lift').value!==''?lift:20,speed:speed||0.7,hipSwing:angle||15};
}
function walkParams(direction){
  const {lift,speed,hipSwing}=walkSettings();
  // The stride angle sizes each step, so the stride in mm is not used here.
  return {gait:$('walk-gait').value||'ripple',direction,stride:40,lift,hipSwing,period:Math.max(400,Math.min(20000,Math.round(1000/speed))),cycles:Math.round(Number($('walk-cycles').value)||0)};
}
function renderWalkReadout(){
  const {lift,speed,hipSwing}=walkSettings(),figures=walkFigures({gait:$('walk-gait').value||'ripple',hipSwing,speed});
  $('walk-lift-value').textContent=`${lift} mm`;$('walk-speed-value').textContent=`${speed.toFixed(2)} steps/s`;$('walk-angle-value').textContent=`${hipSwing}°`;
  $('walk-readout').textContent=`Each step ≈ ${figures.step.toFixed(0)} mm · walking speed ≈ ${figures.speed.toFixed(0)} mm/s forward · one step cycle every ${(1/speed).toFixed(2)} s.`;
}
// Changing a setting while walking sends it to the board, which blends into it.
for(const id of ['walk-lift','walk-speed','walk-angle']){
  $(id).oninput=renderWalkReadout;
  $(id).onchange=async()=>{
    renderWalkReadout();
    if(!live?.walking||live.walking.stopping)return;
    // Check the new settings from the stance the walk started in, as the board will.
    const params=walkParams(live.walking.target);
    // walkBase is unknown if this page reconnected mid-walk; the board still checks.
    try{if(walkBase)checkWalk(walkBase,params,offsets);}catch(error){message(`${error.message} The walk keeps its current settings.`,true);return;}
    await command('walk',{lift:params.lift,hip_swing:params.hipSwing,period_ms:params.period},'New walk settings sent: the robot blends into them over about one step.');
  };
}
$('walk-gait').onchange=renderWalkReadout;
async function walk(direction){
  if(!link||!live?.active||busy||pendingStops||player?.running)return;
  if(!('walking' in live)){message('The board’s motion_bridge.py is too old for live walking. Copy the current Board Files to the board.',true);return;}
  const steering=!!live.walking;
  const params=walkParams(direction);
  try{if(!steering){if(!live.positions)throw new Error('Enable at home before walking.');if(live.mid_step)throw new Error('The last walk stopped mid-step; press Stand (or send a pose) before walking again.');checkWalk(live.positions,params,offsets);walkBase=clone(live.positions);}}
  catch(error){message(error.message,true);return;}
  stopPreview();
  const args=steering?{direction}:{gait:params.gait,direction,stride:params.stride,lift:params.lift,hip_swing:params.hipSwing,period_ms:params.period,cycles:params.cycles};
  await command('walk',args,steering?'Steering: the walk turns toward the new direction over about a cycle.':`Walking (${params.gait}). Press another direction to steer, or ■ to stop in the starting stance.`);
}
Object.entries(WALK_DIRECTIONS).forEach(([id,direction])=>$(id).onclick=()=>walk(direction));
$('walk-stop').onclick=()=>command('walk_stop',{},'Slowing down: the robot stops in its starting stance after about one cycle.');
$('walk-preview').onclick=()=>{
  if(busy||pendingStops||player?.running)return;
  try{
    const base=live?.positions&&!live.walking?live.positions:home(Number($('height').value)||70),params=walkParams([1,0,0]);
    checkWalk(base,params,offsets);
    previewSession=true;motionScene.playSequence(walkPreviewFrames(base,params,params.cycles||3),{ground:Number($('height').value)||70});
    message('Previewing a forward live walk, sampled from the same foot paths the board uses. Nothing is sent to the robot.');
  }catch(error){message(error.message,true);}
};
async function connect(LinkType){
  if(busy||pendingStops||player?.running)return;
  stopPreview();
  if(link&&live){
    // Bug: disconnect left movement controls enabled while awaiting off.
    // Capture the connection before awaiting, and lock controls through cleanup.
    const connection=link;busy=true;hardware();
    try{await connection.send('off');}catch{}
    if(link!==connection)return;
    link=null;live=null;player=null;
    try{await connection.close();}finally{busy=false;hardware();}
    return;
  }
  if(LinkType===SerialLink&&!navigator.serial){message('Open this app in desktop Microsoft Edge or Chrome for USB control.',true);return;}
  busy=true;hardware();
  const connection=new LinkType(event=>{
    // Bug: delayed callbacks from a closed port could corrupt a replacement connection.
    if(link!==connection)return;
    applyState(event.state);
    if(event.event==='timeout'){player?.cancel();hardware();message('Connection timeout: motors disabled and playback cancelled.',true);}
    if(event.event==='error'){player?.cancel();hardware();message(`Board stopped the movement: ${event.error}`,true);}
  },error=>{void disconnected(error,connection);});
  link=connection;
  try{
    const state=await connection.open();if(link!==connection)return;
    offsets=clone(state.offsets);
    // The board's limits replace the offline defaults; an old bridge reports none.
    let outdated='';
    try{setBoardLimits(state.limits,state.joint_limits);NAMES.forEach((_,i)=>[0,1,2].forEach(j=>{const input=$(`xyz-${i}-${j}`);input.min=LIMITS[j][0];input.max=LIMITS[j][1];}));}
    catch(error){outdated=error.message;}
    applyState(state);renderLimits();
    player=new Player(connection,applyState,i=>{index=i;render();});render();
    if(outdated||!state.calibration)message(`Connected, but the board files are out of date: ${outdated||'leg calibration is not supported.'} Copy the current Board Files to the board.`,true);
    else if(connection.simulated)message('Simulated robot connected. Commands go to a simulated board in this page, never over USB.');
    else message('Connected. Preview now uses your board’s saved calibration and limits. Enable at home only with the robot supported.');
    log(connection.simulated?'Simulated training bridge connected':'Training bridge connected');
  }catch(error){await disconnected(error,connection);}
  finally{if(link===connection||!link){busy=false;hardware();}}
}
$('arm').onclick=()=>command('arm');
// jog: a fine-tuning move; ignores the playback speed/easing and stays quiet on success.
async function play(selectedOnly=false,requestedFrames=null,{jog=false}={}){if(!link||!live?.active||!live.positions||busy||pendingStops||player?.running)return;
  // A pose during a live walk would be refused and the error handling would pause the walk.
  if(live.walking){message('Stop the live walk first (■ or X).',true);return;}
  stopPreview();
  const options=jog?{repeat:1,speed:1,ease:'linear'}:playbackOptions(),repeat=selectedOnly||requestedFrames?1:options.repeat,originalIndex=index;
  try{
    const {frames,clamped}=playbackFrames(requestedFrames||(selectedOnly?[clone(frame())]:clone(program.frames)),options);
    // A repeat also moves from the last pose back to the first, so check that path too.
    preflight(repeat>1?[...frames,...frames]:frames,live.positions,offsets);
    player.onFrame=i=>{if(requestedFrames)return;index=selectedOnly?originalIndex:i;render();};
    const running=player.run(frames,{repeat,ease:options.ease,onPass:pass=>{passLabel=repeat>1?` · pass ${pass+1}${repeat===Infinity?'':' of '+repeat}`:'';hardware();}});hardware();
    const quick=frames.filter(f=>f.duration_ms<300).length;
    if(!jog)message(`Playing ${repeat===Infinity?'until stopped':repeat>1?repeat+' times':'once'} at ${options.speed}×${options.ease==='smooth'?' with easing':''}.${clamped?` ${clamped} transition(s) kept within 100–10,000 ms.`:''}${quick?` ${quick} transition(s) under 300 ms: the servos may not keep up.`:''} Pause holds position; Disable motors releases torque.`);
    await running;if(link&&live?.active&&!jog)message('Playback stopped or completed. The robot is holding its last commanded pose.');
  }catch(error){player?.cancel();pendingStops++;hardware();try{if(link)applyState(await link.send('pause'));}catch{}finally{pendingStops--;}message(error.message,true);}finally{passLabel='';hardware();}}
// Direct posture buttons send only the destination, interpolating from the board's current pose.
// They never replay the offline template's assumed starting posture or implicitly enable motors.
for(const id of ['stand','sit'])$(id).onclick=async()=>{
  if(!link||!live?.active||busy||pendingStops||player?.running)return;
  try{const target=postureTarget(id,postureSettings,offsets);await play(true,[target]);}
  catch(error){message(error.message,true);}
};
$('send').onclick=()=>play(true);$('play').onclick=()=>play(false);
async function stop(op){
  stopPreview();player?.cancel();
  // Bug: cancel() unlocked Play before the stop reply. Keep controls locked until all stops finish.
  pendingStops++;hardware();
  try{
    if(!link)return;
    const wasWalking=!!live?.walking;
    applyState(await link.send(op));
    // Bug: cancelling playback froze the preview between reports. Show the acknowledged held pose.
    if(live.positions)showReported(live.positions,0);
    message(op==='off'?'All outputs disabled. Keep the robot supported.':wasWalking?'Walk paused mid-step: some feet may be in the air. Press Stand (or send a pose) before walking again; use ■ to stop a walk gently.':'Playback paused. Motors hold the last commanded pose.');
  }catch(error){await disconnected(error);}
  finally{pendingStops--;hardware();}
}
$('pause').onclick=()=>stop('pause');$('off').onclick=()=>stop('off');
// Fine-tuning on the robot. Jogs move one foot from the board's last commanded pose.
async function jogFoot(axis,direction){
  if(!live?.positions)return;
  const leg=Number($('jog-leg').value)||0,step=Number($('jog-step').value)||2,target=clone(live.positions);
  target[leg][axis]+=direction*step;
  await play(true,[{name:`Jog ${NAMES[leg]}`,positions:target,duration_ms:500}],{jog:true});
  if(live?.positions)message(`${NAMES[leg]} foot at X ${live.positions[leg][0].toFixed(1)}, Y ${live.positions[leg][1].toFixed(1)}, Z ${live.positions[leg][2].toFixed(1)} mm.`);
}
// Turn the selected foot around its hip, keeping reach and height.
async function jogHip(direction){
  if(!live?.positions)return;
  const leg=Number($('jog-leg').value)||0,step=Number($('jog-hip-step').value)||5,target=clone(live.positions);
  target[leg]=rotateFoot(target[leg],direction*step);
  await play(true,[{name:`Hip ${NAMES[leg]}`,positions:target,duration_ms:500}],{jog:true});
  if(live?.positions)try{message(`${NAMES[leg]} hip at ${anglesForPose(live.positions,offsets)[leg][0].toFixed(1)}° (foot X ${live.positions[leg][0].toFixed(1)}, Y ${live.positions[leg][1].toFixed(1)} mm).`);}catch{}
}
$('jog-hip-minus').onclick=()=>jogHip(-1);$('jog-hip-plus').onclick=()=>jogHip(1);
for(const [id,axis,direction] of [['jog-up',2,-1],['jog-down',2,1],['jog-x-minus',0,-1],['jog-x-plus',0,1],['jog-y-minus',1,-1],['jog-y-plus',1,1]])$(id).onclick=()=>jogFoot(axis,direction);
$('save-stand-trim').onclick=()=>{
  if(!live?.positions||busy||pendingStops||player?.running)return;
  const leg=Number($('jog-leg').value)||0,[x,y,z]=live.positions[leg];
  try{
    const standing=live.positions.every((p,i)=>i===leg||Math.abs(p[2]-(postureSettings.standHeight+postureSettings.standTrim[i]))<0.5);
    if(!standing)throw new Error('Press Stand first, then lower this foot: the trim is measured with the other legs at their Stand heights.');
    const standTrim=[...postureSettings.standTrim];standTrim[leg]=Math.round((z-postureSettings.standHeight)*10)/10;
    const next=validatePostureSettings({...postureSettings,standTrim});postureTarget('stand',next,offsets);savePostureSettings(next);
    message(`${NAMES[leg]} Stand trim saved as ${standTrim[leg]} mm (foot Z ${z.toFixed(1)} mm).${x!==0||y!==175?' Only its height is stored; Stand keeps X 0, Y 175.':''}`);
  }catch(error){message(error.message,true);}
};
$('jog-capture').onclick=()=>$('capture').onclick();
// Servo test: step one joint to find its real travel, then save limits on the board.
function selectedServo(){return [Number($('servo-leg').value)||0,Number($('servo-joint').value)||0];}
for(const [id,delta] of [['servo-minus5',-5],['servo-minus1',-1],['servo-plus1',1],['servo-plus5',5]])$(id).onclick=async()=>{
  const [leg,joint]=selectedServo(),current=live?.angles?.[leg]?.[joint];if(!Number.isFinite(current))return;
  const angle=Math.max(-180,Math.min(180,Math.round((current+delta)*10)/10));
  await command('jog_joint',{leg,joint,angle},`${NAMES[leg]} ${['hip','upper leg','lower leg'][joint]} at ${angle.toFixed(1)}°. Use Enable at home before moving the legs again.`);
};
for(const [id,end] of [['servo-set-min',0],['servo-set-max',1]])$(id).onclick=()=>{const [leg,joint]=selectedServo(),current=live?.angles?.[leg]?.[joint];if(Number.isFinite(current))$(LIMIT_IDS[joint][end]).value=Math.round(current*10)/10;};
for(const id of ['servo-leg','servo-joint'])$(id).onchange=()=>hardware();
$('save-limits').onclick=async()=>{
  const limits=LIMIT_IDS.map(pair=>pair.map(id=>Number($(id).value)));
  if(!limits.every(([low,high])=>Number.isFinite(low)&&Number.isFinite(high)&&low<high)){message('Each joint needs a minimum below its maximum.',true);return;}
  if(await command('set_joint_limits',{joint_limits:limits},'Joint limits saved on the board. Poses are now checked against them.')){renderLimits();render();}
  else renderLimits();
};
// Release one leg: stop-like, so it works during playback and is not blocked by other commands.
$('release-leg-button').onclick=async()=>{
  if(!link)return;const leg=Number($('release-leg').value)||0;
  stopPreview();player?.cancel();pendingStops++;hardware();
  try{applyState(await link.send('release_leg',{leg}));message(`${NAMES[leg]} released; the other legs are holding. Use Enable at home before moving again.`);}
  catch(error){message(error.message,true);}
  finally{pendingStops--;hardware();}
};
// Emergency stop: Esc anywhere, or Space when no control has focus, disables the motors.
$('off').title='Emergency stop (Esc or Space)';
window.addEventListener('keydown',event=>{
  if(event.repeat||!link)return;
  const idle=document.activeElement===document.body||document.activeElement===null;
  if(event.key==='Escape'||(event.key===' '&&idle)){event.preventDefault();stop('off');return;}
  // Arrow keys jog the selected foot when no field has focus.
  const jog={ArrowUp:[2,-1],ArrowDown:[2,1],ArrowLeft:[0,-1],ArrowRight:[0,1]}[event.key];
  const hip={'[':-1,']':1}[event.key],ready=idle&&tab==='motion'&&live?.active&&live.positions&&!live.walking&&!busy&&!pendingStops&&!player?.running;
  if(jog&&ready){event.preventDefault();jogFoot(...jog);}
  // Live walk keys: W A S D move, Q / E turn, X stops.
  const walkKey={w:'walk-forward',s:'walk-back',a:'walk-left',d:'walk-right',q:'walk-turn-left',e:'walk-turn-right',x:'walk-stop'}[event.key?.toLowerCase()];
  if(walkKey&&idle&&tab==='motion'&&!$(walkKey).disabled){event.preventDefault();$(walkKey).onclick();}
  if(hip&&ready){event.preventDefault();jogHip(hip);}
});
// Status every 650 ms, or every 200 ms while walking so the preview follows the robot.
let lastPoll=0;
setInterval(async()=>{
  if(!link||!live||busy||pendingStops||polling||player?.running)return;
  if(!live.walking&&Date.now()-lastPoll<650)return;
  const connection=link;polling=true;lastPoll=Date.now();
  try{const state=await connection.send('status');if(link===connection)applyState(state);}
  catch(error){await disconnected(error,connection);}
  finally{polling=false;}
},200);
// Hidden tabs get their timers slowed (eventually to once a minute), which
// would let the board's 3 s watchdog cut the servos mid-step. Stop gently instead.
document.addEventListener?.('visibilitychange',()=>{
  if(!document.hidden||!link||!live?.walking||live.walking.stopping)return;
  link.send('walk_stop').then(applyState).catch(()=>{});
  message('This tab was hidden, so the walk is stopping in its starting stance. Keep Motion Studio visible while the robot walks.',true);log('Tab hidden: walk stopped');
});
window.addEventListener('beforeunload',event=>{if(dirty||calDirty||live?.active){event.preventDefault();event.returnValue='';}});
$('board-files').innerHTML=['main.py','FUNCTIONS.py','calibration_store.py','motion_bridge.py'].map(name=>`<a href="board/${name}" download>${name}</a>`).join(', ');
$('undo').onclick=()=>restoreEdit(undoStack,redoStack);
$('redo').onclick=()=>restoreEdit(redoStack,undoStack);
function renderPostureSettings(){
  $('stand-height').value=postureSettings.standHeight;$('sit-height').value=postureSettings.sitHeight;$('posture-duration').value=postureSettings.duration;
  TRIM_IDS.forEach((id,i)=>$(id).value=postureSettings.sitTrim[i]);
  STAND_TRIM_IDS.forEach((id,i)=>$(id).value=postureSettings.standTrim[i]);
}
function savePostureSettings(next){
  postureSettings=next;renderPostureSettings();
  try{localStorage.setItem(SETTINGS,JSON.stringify(next));return true;}
  catch{message('Settings applied for this session. Browser storage is unavailable.',true);return false;}
}
for(const id of ['stand-height','sit-height','posture-duration',...TRIM_IDS,...STAND_TRIM_IDS])$(id).onchange=()=>{
  if(busy||pendingStops||player?.running){renderPostureSettings();return;}
  try{
    const next=validatePostureSettings({standHeight:Number($('stand-height').value),sitHeight:Number($('sit-height').value),duration:Number($('posture-duration').value),sitTrim:TRIM_IDS.map(id=>Number($(id).value)),standTrim:STAND_TRIM_IDS.map(id=>Number($(id).value))});
    postureTarget('stand',next,offsets);postureTarget('sit',next,offsets);
    if(savePostureSettings(next))message('Posture settings saved. Rebuild a template to update its poses; robot buttons use these settings.');
  }catch(error){message(error.message,true);}
  renderPostureSettings();
};
try{const saved=localStorage.getItem(SETTINGS);if(saved)postureSettings=validatePostureSettings(JSON.parse(saved));}
catch{message('Saved posture settings could not be loaded; using defaults.',true);}
renderPostureSettings();
try{
  const saved=localStorage.getItem(DRAFT);
  if(saved){const draft=JSON.parse(saved);program=parseProgram(JSON.stringify(draft.program),draft.offsets||DEFAULT_OFFSETS);
    offsets=clone(draft.offsets||DEFAULT_OFFSETS); // Retain draft calibration until the connected board supplies fresh offsets.
    index=Number.isInteger(draft.index)?Math.max(0,Math.min(program.frames.length-1,draft.index)):0;
    snapshot={program:clone(program),index};dirty=true;
    $('draft-status').textContent='Recovered your last draft';$('saved-status').textContent='Recovered draft — not a named save';
    message('Recovered your last draft. No robot commands were sent. Connect to validate it against the current board calibration.');
  }
}catch{$('draft-status').textContent='Draft could not be recovered. Import a JSON backup if available.';}
refreshLibrary();renderLimits();renderWalkReadout();render();
// Open the remembered tab, or the calibration tab directly with index.html#calibrate.
try{if(globalThis.location?.hash==='#calibrate'||localStorage.getItem('hexapod-tab-v1')==='calibrate')setTab('calibrate');}catch{}
