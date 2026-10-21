import {test} from 'node:test';
import assert from 'node:assert/strict';
import {SerialLink,PROTOCOL} from '../webapp/dist/transport.js';
const flush=()=>new Promise(resolve=>setImmediate(resolve));
function setup(){const written=[],events=[];const link=new SerialLink(event=>events.push(event),error=>events.push(error));link.writer={write:async bytes=>written.push(JSON.parse(new TextDecoder().decode(bytes)))};return {link,written,events};}
function reply(link,id,state={}){link.accept(JSON.stringify({id,ok:true,state:{protocol:PROTOCOL,...state}}));}
test('USB messages are acknowledged by ID and queued serially',async()=>{
 const {link,written}=setup();const first=link.send('hello'),second=link.send('status');await flush();
 assert.equal(written.length,1);reply(link,999);assert.ok(link.pending);reply(link,1);await first;await flush();assert.equal(written.length,2);reply(link,2);await second;
});
test('cancellation discards queued movement but allows off command',async()=>{
 const {link,written}=setup();let allowed=true;
 const first=link.send('status'),move=link.send('pose',{},()=>allowed),off=link.send('off');
 const rejected=assert.rejects(move,/cancelled/);allowed=false;await flush();reply(link,1);await first;await rejected;await flush();
 assert.deepEqual(written.map(r=>r.op),['status','off']);reply(link,2);await off;
});
test('board errors and timeout events remain visible',async()=>{
 const {link,events}=setup();const command=link.send('pose');await flush();link.accept('{"id":1,"ok":false,"error":"Joint limit"}');await assert.rejects(command,/Joint limit/);
 link.accept(JSON.stringify({event:'timeout',state:{protocol:PROTOCOL,active:false}}));assert.equal(events[0].state.active,false);
 link.accept('ImportError: no module named motion_bridge');assert.match(link.diagnostic,/motion_bridge/);
});

test('late write failure cannot reject the next command',async()=>{
 const {link}=setup();let rejectWrite;
 link.writer.write=()=>new Promise((resolve,reject)=>{rejectWrite=reject;});
 const first=link.send('status');await flush();const rejectFirst=rejectWrite;
 reply(link,1);await first;
 const second=link.send('status');const result=second.then(()=>null,error=>error);
 await flush();rejectFirst(new Error('Late write failure'));await flush();
 assert.equal(link.pending?.id,2);
 reply(link,2);assert.equal(await result,null);
});
test('synchronous write failures clear the pending request',async()=>{
 const {link}=setup();link.writer.write=()=>{throw new Error('Writer unavailable');};
 await assert.rejects(link.send('status'),/Writer unavailable/);
 const pending=link.pending;
 // Clean up even on a failing regression, so the timeout does not delay the suite.
 if(pending)clearTimeout(pending.timer);
 assert.equal(pending,null);
});
