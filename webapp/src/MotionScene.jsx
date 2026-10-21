import React,{useEffect,useSyncExternalStore} from 'react';
import {createRoot} from 'react-dom/client';
import {SceneController} from './scene-controller.js';
import {LEG_NAMES,DEFAULT_BODY,legJoints,project,transformWorld,balance} from './motion-geometry.js';

const polyline=(points,view)=>points.map(p=>project(p,view).join(',')).join(' ');
function Robot({state}) {
  const {pose,body:worldBody=DEFAULT_BODY,view,ground,trails,frames,history=[]}=state;
  const world=pt=>transformWorld(pt,worldBody);
  const body=[[-58,-70,9],[0,-101,9],[58,-70,9],[72,0,9],[58,70,9],[0,101,9],[-58,70,9],[-72,0,9]].map(world);
  const nose=[[-13,-78,10],[0,-97,10],[13,-78,10]].map(world);
  const legs=pose.map((xyz,i)=>({i,joints:legJoints(xyz,i).map(world),lift:ground-xyz[2]})).sort((a,b)=>project(a.joints[0],view)[1]-project(b.joints[0],view)[1]);
  const grid=[];for(let v=-300;v<=300;v+=40){grid.push([[-300,v,-ground],[300,v,-ground]],[[v,-300,-ground],[v,300,-ground]]);}
  function drawLeg({i,joints,lift}) {
    const points=joints.map(p=>project(p,view)),foot=points[3],shadow=project([joints[3][0],joints[3][1],-ground],view);
    return <g key={i}>
      <ellipse cx={shadow[0]} cy={shadow[1]} rx={(view==='top'?11:13)*(1-Math.min(Math.max(lift,0),30)/90)} ry={(view==='top'?11:5)*(1-Math.min(Math.max(lift,0),30)/90)} fill="#071c22" opacity={.5-Math.min(Math.max(lift,0),30)/90}/>
      {lift>1&&<line x1={foot[0]} y1={foot[1]} x2={shadow[0]} y2={shadow[1]} stroke="#6c9680" strokeDasharray="3 4"/>}
      {[0,1,2].map(j=><g key={j}><line x1={points[j][0]} y1={points[j][1]} x2={points[j+1][0]} y2={points[j+1][1]} stroke="#0b2026" strokeWidth={[16,13,10][j]} strokeLinecap="round"/><line x1={points[j][0]} y1={points[j][1]} x2={points[j+1][0]} y2={points[j+1][1]} stroke={lift>1?['#9fbf8c','#d6eaa7','#a2c58b'][j]:['#567c73','#8eaca0','#63897c'][j]} strokeWidth={[11,8,5][j]} strokeLinecap="round"/></g>)}
      {points.slice(0,3).map(([x,y],j)=><g key={j}><circle cx={x} cy={y} r={j===0?7:6} fill="#183b40" stroke="#8cab92" strokeWidth="1.5"/><circle cx={x} cy={y} r="2" fill="#c8dfa0"/></g>)}
      {lift>1&&<circle cx={foot[0]} cy={foot[1]} r="11" fill="#e7f7bc" opacity=".22"/>}
      <circle cx={foot[0]} cy={foot[1]} r="5" fill={lift>1?'#e7f7bc':'#a6c290'}/>
      <text x={foot[0]+(i<3?-16:16)} y={foot[1]+4} textAnchor={i<3?'end':'start'} fill="#bbd1be" fontSize="11" fontFamily="sans-serif">{LEG_NAMES[i]}</text>
    </g>;
  }
  const label=project(world([0,0,11]),view);
  const turnDeg=worldBody.theta*180/Math.PI;
  const support=balance(pose),centre=project([...world([0,0,0]).slice(0,2),-ground],view);
  const moved=Math.hypot(worldBody.x,worldBody.y)>0.3||Math.abs(turnDeg)>0.2;
  return <svg viewBox="0 0 620 440" role="img" aria-label={`Animated hexapod in ${view} view. Six articulated legs show commanded pose targets.`}>
    <defs><radialGradient id="scene-ground"><stop offset="0" stopColor="#244950" stopOpacity=".5"/><stop offset="1" stopColor="#122c33" stopOpacity="0"/></radialGradient><linearGradient id="shell" x1="0" y1="0" x2="1" y2="1"><stop stopColor="#335954"/><stop offset="1" stopColor="#17363c"/></linearGradient></defs>
    <rect width="620" height="440" fill="url(#scene-ground)"/>
    <g opacity=".6">{grid.map((line,i)=><polyline key={i} points={polyline(line,view)} fill="none" stroke="#2b474b" strokeWidth=".7"/>)}</g>
    <polygon points={polyline(body.map(([x,y])=>[x,y,-ground]),view)} fill="#091e25" opacity=".45"/>
    {support.hull.length>=3&&<polygon className="support-area" points={polyline(support.hull.map(([x,y])=>[...world([x,y,0]).slice(0,2),-ground]),view)} fill={support.stable?'#9fbf8c':'#e0806b'} fillOpacity=".14" stroke={support.stable?'#9fbf8c':'#e0806b'} strokeDasharray="5 4" strokeWidth="1.2"/>}
    <g className="balance-centre"><circle cx={centre[0]} cy={centre[1]} r="6" fill="none" stroke={support.stable?'#d6eaa7':'#f09a85'} strokeWidth="1.6"/><line x1={centre[0]-9} y1={centre[1]} x2={centre[0]+9} y2={centre[1]} stroke={support.stable?'#d6eaa7':'#f09a85'}/><line x1={centre[0]} y1={centre[1]-9} x2={centre[0]} y2={centre[1]+9} stroke={support.stable?'#d6eaa7':'#f09a85'}/></g>
    {trails&&frames.length>0&&LEG_NAMES.map((_,i)=><polyline key={i} points={polyline(frames.map(frame=>world(legJoints(frame.positions[i],i)[3])),view)} fill="none" stroke="#b5ce8c" opacity=".35" strokeDasharray="3 5" strokeWidth="1.2"/>)}
    {history.length>1&&LEG_NAMES.map((_,i)=>{const pts=history.map(h=>project(h.worldFeet?h.worldFeet[i]:world(legJoints(h[i],i)[3]),view));return <g key={'t'+i}>{pts.slice(1).map((p,k)=><line key={k} x1={pts[k][0]} y1={pts[k][1]} x2={p[0]} y2={p[1]} stroke="#d6eaa7" strokeWidth="2.2" strokeLinecap="round" opacity={(k+1)/pts.length*.55}/>)}</g>;})}
    {legs.filter(leg=>project(leg.joints[0],view)[1]<=205).map(drawLeg)}
    <polygon points={polyline(body.map(([x,y,z])=>[x,y,z-18]),view)} fill="#0d242b" stroke="#44665e"/>
    <polygon points={polyline(body,view)} fill="url(#shell)" stroke="#74917c" strokeWidth="1.4"/>
    <polygon points={polyline(body.map(([x,y,z])=>[worldBody.x+(x-worldBody.x)*.77,worldBody.y+(y-worldBody.y)*.77,z+.1]),view)} fill="none" stroke="#476c60"/>
    <polygon points={polyline(nose,view)} fill="#cfe5a3" opacity=".88"/>
    <circle cx={label[0]} cy={label[1]} r="22" fill="#cfe5a3"/><text x={label[0]} y={label[1]+7} textAnchor="middle" fill="#203d36" fontSize="23" fontFamily="sans-serif">H</text>
    {legs.filter(leg=>project(leg.joints[0],view)[1]>205).map(drawLeg)}
    {moved&&<g transform="translate(14,418)"><rect x="0" y="-18" width="230" height="22" rx="6" fill="#0b2026" opacity=".78" stroke="#365258" strokeWidth=".8"/><text x="9" y="-3" fill="#cfe5a3" fontSize="10.5" fontFamily="monospace">{`Body ΔX ${worldBody.x.toFixed(1)}  ΔY ${(-worldBody.y).toFixed(1)} mm  Yaw ${turnDeg>=0?'+':''}${turnDeg.toFixed(1)}°`}</text></g>}
  </svg>;
}
function BalanceLabel({pose}) {
  const {stable,margin,down}=balance(pose);
  const text=down<3?`Balance: only ${down} feet down`:stable?`Balance OK · ${margin.toFixed(0)} mm margin`:'Balance: centre outside the feet';
  return <span className={stable?'balance-ok':'balance-warn'} title="Estimate: support area of the lowest feet vs. the body centre. Mass distribution is not modelled.">{text}</span>;
}
function MotionScene({controller}) {
  const state=useSyncExternalStore(controller.subscribe,controller.getSnapshot,controller.getSnapshot);
  useEffect(()=>()=>controller.cancel(),[controller]);
  return <div className="react-motion-scene">
    <div className="scene-view-controls"><div className="view-tabs"><button aria-pressed={state.view==='angled'} onClick={()=>controller.setView('angled')}>Angled view</button><button aria-pressed={state.view==='top'} onClick={()=>controller.setView('top')}>Top view</button><button aria-pressed={state.view==='front'} onClick={()=>controller.setView('front')}>Front</button><button aria-pressed={state.view==='side'} onClick={()=>controller.setView('side')}>Side</button></div><div className="scene-toggles"><button className="trail-toggle" aria-pressed={state.loop} onClick={()=>controller.toggleLoop()} title="Repeat the preview until paused">↻ Loop</button><button className="trail-toggle" aria-pressed={state.trails} onClick={()=>controller.toggleTrails()}>Foot paths</button></div></div>
    <Robot state={state}/>
    <div className="scene-key"><span><i/> {state.playback==='playing'?'Animating between poses':state.playback==='paused'?'Preview paused':'Articulated pose model'}</span><BalanceLabel pose={state.pose}/><span>Not sensor feedback</span></div>
    <div className="animation-scrubber"><label htmlFor="animation-time">Preview position</label><input id="animation-time" type="range" min="0" max={state.total||1} step="10" value={state.elapsed} disabled={!state.frames.length} onChange={event=>controller.seek(Number(event.target.value))}/><output>{(state.elapsed/1000).toFixed(1)} / {(state.total/1000).toFixed(1)}s</output></div>
    <div className="animation-settings"><span>Preview speed</span><select aria-label="Preview animation speed" value={state.speed} onChange={event=>controller.setSpeed(Number(event.target.value))}><option value="0.25">0.25× · slow</option><option value="0.5">0.5×</option><option value="1">1× · normal</option><option value="2">2×</option></select><small>Visual preview only</small></div>
  </div>;
}
// The balance estimate is shared with the app so it can warn when building templates.
export {balance} from './motion-geometry.js';
export function mountMotionScene(container,options={}) {
  const controller=new SceneController(options),root=createRoot(container);
  root.render(<MotionScene controller={controller}/>);
  const destroy=controller.destroy.bind(controller);
  controller.destroy=()=>{root.unmount();destroy();};
  return controller;
}
