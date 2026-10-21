export const PROTOCOL='hexapod-motion-v1';
export class SerialLink {
  constructor(event,lost) {this.event=event;this.lost=lost;this.pending=null;this.id=0;this.queue=Promise.resolve();this.closing=false;this.diagnostic='';}
  async open() {
    this.port=await navigator.serial.requestPort();await this.port.open({baudRate:115200});
    await this.port.setSignals({dataTerminalReady:true});
    this.reader=this.port.readable.getReader();this.writer=this.port.writable.getWriter();this.reading=this.read();
    return this.send('hello');
  }
  send(op,args={},guard=()=>true) {
    const task=this.queue.then(()=>{
      if(!guard()) throw new Error('Movement cancelled.');
      if(!this.writer||this.closing) throw new Error('Robot is disconnected.');
      return new Promise((resolve,reject)=>{
        const id=++this.id;
        const timer=setTimeout(()=>{this.pending=null;reject(new Error('No training bridge reply. Upload the training Board Files, close Thonny, and restart the board. '+this.diagnostic));},op==='hello'?5000:2000);
        this.pending={id,timer,resolve,reject};
        // Bug: a delayed write rejection could fail the following request after this one was acknowledged.
        // Tie failures to their request ID, and clear pending state on synchronous write errors too.
        const writeFailed=error=>{if(this.pending?.id===id)this.fail(error);};
        try{Promise.resolve(this.writer.write(new TextEncoder().encode(JSON.stringify({...args,id,op})+'\n'))).catch(writeFailed);}
        catch(error){writeFailed(error);}
      });
    });
    this.queue=task.catch(()=>{});return task;
  }
  fail(error) {if(this.pending){clearTimeout(this.pending.timer);this.pending.reject(error);this.pending=null;}}
  accept(line) {
    let message;try{message=JSON.parse(line);}catch{this.diagnostic=(this.diagnostic+' '+line).slice(-500);return;}
    if(!message||typeof message!=='object')return;
    if(this.pending&&message.id===this.pending.id){const pending=this.pending;this.pending=null;clearTimeout(pending.timer);
      if(message.ok&&message.state?.protocol===PROTOCOL)pending.resolve(message.state);
      else pending.reject(new Error(message.error||'Wrong board program. Use the Motion Trainer board files.'));
    } else if(message.event&&message.state?.protocol===PROTOCOL)this.event(message);
  }
  async read() {
    const decoder=new TextDecoder();let buffer='';
    try{while(!this.closing){const {value,done}=await this.reader.read();if(done)break;buffer+=decoder.decode(value,{stream:true});let at;
      while((at=buffer.indexOf('\n'))>=0){const line=buffer.slice(0,at).trim();buffer=buffer.slice(at+1);if(line)this.accept(line);}
      if(buffer.length>20000)throw new Error('Unexpected serial output. Restart the training bridge.');}
      if(!this.closing)throw new Error('USB disconnected.');
    }catch(error){this.fail(error);if(!this.closing)this.lost(error);}
  }
  async close(){this.closing=true;this.fail(new Error('Disconnected.'));try{await this.reader?.cancel();await this.reading;}catch{}
    try{this.reader?.releaseLock();}catch{}
    try{this.writer?.releaseLock();}catch{}
    try{await this.port?.close();}catch{}
    this.reader=this.writer=this.port=null;}
}
