export const LEG_NAMES = ['L1','L2','L3','R1','R2','R3'];
const MOUNTS = [[-58,-70,0],[-72,0,0],[-58,70,0],[58,-70,0],[72,0,0],[58,70,0]];
const ROTATIONS = [30,0,-30,150,180,-150].map(v=>v*Math.PI/180);
export const copyPose = pose => pose.map(xyz=>xyz.slice());
export const mixPose = (start,end,t) => start.map((xyz,i)=>xyz.map((v,j)=>v+(end[i][j]-v)*Math.max(0,Math.min(1,t))));
export const DEFAULT_POSE = LEG_NAMES.map(()=>[0,175,70]);

export function legJoints(xyz,index) {
  const [x,y,z]=xyz,radial=Math.hypot(x,y),reach=Math.hypot(radial-47,z);
  const beta=Math.acos(Math.max(-1,Math.min(1,(88.46**2+reach**2-148.8**2)/(2*88.46*reach))));
  const upper=beta-Math.atan2(z,radial-47);
  const kneeRadial=47+88.46*Math.cos(upper);
  // A foot on the hip axis has no direction; point the leg straight out to avoid NaN.
  const [ux,uy]=radial<1e-6?[0,1]:[x/radial,y/radial];
  const local=[[0,0,0],[47*ux,47*uy,0],[kneeRadial*ux,kneeRadial*uy,-88.46*Math.sin(upper)],[x,y,z]];
  const theta=ROTATIONS[index],[mx,my,mz]=MOUNTS[index];
  // Invert the source's leg-local forward/lateral transform into body space.
  // Positive local Z points down; the renderer's Z points up.
  return local.map(([lx,ly,lz])=>[mx+lx*Math.sin(theta)-ly*Math.cos(theta),my-lx*Math.cos(theta)-ly*Math.sin(theta),mz-lz]);
}
export function project(point,view='angled') {
  const [x,y,z]=point;
  if(view==='top')return [310+x*0.94,205+y*0.94];
  // Straight-on elevations: horizontal axis + height only, so a tilted body is obvious.
  if(view==='front')return [310+x*0.94,235-z*1.1];
  if(view==='side')return [310+y*0.94,235-z*1.1];
  const yaw=0.12,rx=x*Math.cos(yaw)-y*Math.sin(yaw),ry=x*Math.sin(yaw)+y*Math.cos(yaw);
  return [310+rx*0.98,205+ry*0.62-z*0.9];
}
export function totalDuration(frames) {return frames.reduce((sum,f)=>sum+f.duration_ms,0);}
export function sampleSequence(frames,time) {
  if(!frames.length)throw new Error('Animation needs at least one pose.');
  const total=totalDuration(frames),elapsed=Math.max(0,Math.min(total,time));
  if(elapsed>=total)return {pose:copyPose(frames.at(-1).positions),index:frames.length-1,elapsed,total};
  let startTime=0;
  for(let i=0;i<frames.length;i++) {
    const end=startTime+frames[i].duration_ms;
    if(elapsed<end){
      const previous=i===0?frames[0].positions:frames[i-1].positions,t=(elapsed-startTime)/frames[i].duration_ms;
      // Match the board's 'smooth' ease: same path, slower start and end.
      return {pose:mixPose(previous,frames[i].positions,frames[i].ease==='smooth'?0.5-0.5*Math.cos(Math.PI*t):t),index:i,elapsed,total};
    }
    startTime=end;
  }
}

export const DEFAULT_BODY = {x:0,y:0,theta:0};
export function bodyStep(prev,next,eps=0.75) {
  const gp=Math.max(...prev.map(p=>p[2])),gn=Math.max(...next.map(p=>p[2]));
  const P=[],Q=[];
  for(let i=0;i<6;i++) {
    // A foot pushes the body only while it stays down: one that has just begun
    // to lift (insect gaits lift and swing at once) is already moving forward.
    if(prev[i][2]>=gp-eps&&next[i][2]>=gn-eps&&Math.abs(next[i][2]-prev[i][2])<1e-6) {
      P.push(legJoints(prev[i],i)[3]);
      Q.push(legJoints(next[i],i)[3]);
    }
  }
  if(!P.length)return {dx:0,dy:0,dtheta:0};
  const center=pts=>[pts.reduce((s,p)=>s+p[0],0)/pts.length,pts.reduce((s,p)=>s+p[1],0)/pts.length];
  const pc=center(P),qc=center(Q);
  let phi=0;
  if(P.length>1) {
    let dot=0,cross=0;
    P.forEach((p,k)=>{
      const px=p[0]-pc[0],py=p[1]-pc[1],qx=Q[k][0]-qc[0],qy=Q[k][1]-qc[1];
      dot+=qx*px+qy*py;cross+=qx*py-qy*px;
    });
    phi=Math.atan2(cross,dot);
  }
  const co=Math.cos(phi),si=Math.sin(phi);
  return {dx:pc[0]-(co*qc[0]-si*qc[1]),dy:pc[1]-(si*qc[0]+co*qc[1]),dtheta:phi};
}
export function composeBody(w,s) {
  const co=Math.cos(w.theta),si=Math.sin(w.theta);
  return {
    x:Math.max(-180,Math.min(180,w.x+co*s.dx-si*s.dy)),
    y:Math.max(-180,Math.min(180,w.y+si*s.dx+co*s.dy)),
    theta:w.theta+s.dtheta
  };
}
export function transformWorld([x,y,z],body=DEFAULT_BODY) {
  const co=Math.cos(body.theta),si=Math.sin(body.theta);
  return [body.x+co*x-si*y,body.y+si*x+co*y,z];
}
export function sampleBodyAt(frames,elapsed,base=DEFAULT_BODY) {
  if(!frames.length||elapsed<=0)return {...base};
  let w={...base},prev=sampleSequence(frames,0).pose;
  const step=25;
  for(let t=step;t<elapsed;t+=step) {
    const next=sampleSequence(frames,t).pose;
    w=composeBody(w,bodyStep(prev,next));
    prev=next;
  }
  const finalPose=sampleSequence(frames,elapsed).pose;
  return composeBody(w,bodyStep(prev,finalPose));
}


// Static balance estimate: the support polygon is the convex hull of the feet
// that are down (lowest, within eps), in body XY. The body centre is assumed at
// the middle of the hip mounts; real mass distribution is not modelled.
// margin > 0: distance (mm) from the centre to the nearest support edge.
export function balance(pose,eps=0.75) {
  const lowest=Math.max(...pose.map(p=>p[2]));
  const feet=pose.map((p,i)=>p[2]>=lowest-eps?legJoints(p,i)[3]:null).filter(Boolean).map(([x,y])=>[x,y]);
  const hull=convexHull(feet);
  if(hull.length<3)return {hull,margin:-Infinity,stable:false,down:feet.length};
  let margin=Infinity;
  for(let k=0;k<hull.length;k++){
    const [ax,ay]=hull[k],[bx,by]=hull[(k+1)%hull.length];
    // Hull is counter-clockwise, so the inside is to the left of each edge.
    margin=Math.min(margin,((bx-ax)*(0-ay)-(by-ay)*(0-ax))/Math.hypot(bx-ax,by-ay));
  }
  return {hull,margin,stable:margin>0,down:feet.length};
}
function convexHull(points) {
  const sorted=[...points].sort((a,b)=>a[0]-b[0]||a[1]-b[1]);
  if(sorted.length<3)return sorted;
  const cross=(o,a,b)=>(a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0]);
  const half=list=>{const out=[];for(const p of list){while(out.length>=2&&cross(out.at(-2),out.at(-1),p)<=0)out.pop();out.push(p);}out.pop();return out;};
  return [...half(sorted),...half([...sorted].reverse())];
}
