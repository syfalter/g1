const $ = id => document.getElementById(id);
const phases = {STARTUP_REQUIRED:'待初始化',READY:'就绪',RUNNING:'执行中',STOP_UNCONFIRMED:'停止待确认',RECOVERY_REQUIRED:'等待恢复',HANDOFF_PENDING:'交接中',AWAITING_WSL:'等待 WSL 接管',WSL_OWNED:'WSL 调试中',AWAITING_JETSON:'等待 Jetson 接管'};
let token='', state=null, curves=null, lease=sessionStorage.getItem('previewLease'), selectedCurve=0, pending=false, noticeTimer;
const colors=['#187b73','#c56a27','#4969ba','#aa4585','#5a8739','#ba4141','#756947','#3091a3'];
function notice(text){$('message').textContent=text;$('message').classList.add('visible');clearTimeout(noticeTimer);noticeTimer=setTimeout(()=>$('message').classList.remove('visible'),7000);}
async function get(path){const response=await fetch(path);if(!response.ok)throw Error('状态连接失败');return response.json();}
async function post(path,body){if(pending)return;pending=true;render();try{const response=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json','X-Preview-Token':token},body:JSON.stringify(body)});const data=await response.json();if(!response.ok)throw Error(`${data.code}: ${data.message}`);if(data.token){lease=data.token;sessionStorage.setItem('previewLease',lease);}notice('离线操作已受理');await refresh();return data;}catch(e){notice(e.message);}finally{pending=false;render();}}
function task(signal){return post('/api/task',{taskId:`WEB-${Date.now()}`,taskType:signal?'NLG_WOW_OPERATION':'ROBOT_INITIALIZE',parameters:signal?{wowSignal:signal}:{}});}
function handoff(action){return post('/api/authority',{action,epoch:state.epoch,token:lease,supportReady:$('handoff-support').checked});}
function recovery(step){return post('/api/recovery',{step,contactClear:$('contact-clear').checked,supportReady:$('support-ready').checked});}
function text(parent,tag,value){const n=document.createElement(tag);n.textContent=value;parent.appendChild(n);return n;}
function enable(id,condition){$(id).disabled=pending||!condition;}
function render(){if(!state)return;const s=state,p=s.phase,ready=p==='READY',own=s.owner==='jetson';$('phase').textContent=phases[p]||p;$('owner').textContent=({jetson:'Jetson',wsl:'WSL',none:'两端未持有',unknown:'待核对'})[s.owner];$('stage').textContent=s.stage||'—';$('bundle').textContent=s.bundle.slice(0,16);$('air').textContent=`AIR · ${s.mapping.AIR==='push'?'推进':'拉出'}`;$('ground').textContent=`GROUND · ${s.mapping.GROUND==='push'?'推进':'拉出'}`;$('mapping').value=s.mapping.AIR;
enable('initialize',own&&(ready||p==='STARTUP_REQUIRED'));enable('air',own&&ready);enable('ground',own&&ready);enable('stop',own&&p==='RUNNING');enable('fault',own&&p==='RUNNING');enable('mapping',own&&(ready||p==='STARTUP_REQUIRED'));enable('release',own&&(ready||p==='STARTUP_REQUIRED')&&$('handoff-support').checked);enable('wsl-claim',p==='AWAITING_WSL'&&!!lease);enable('wsl-release',p==='WSL_OWNED'&&!!lease&&$('handoff-support').checked);enable('acquire',p==='AWAITING_JETSON');enable('confirm-stop',['STOP_UNCONFIRMED','RECOVERY_REQUIRED'].includes(p));enable('inspect',p==='RECOVERY_REQUIRED'&&s.recovery_step===1&&$('contact-clear').checked&&$('support-ready').checked);enable('reset',p==='RECOVERY_REQUIRED'&&s.recovery_step===2&&own);enable('reconcile',p==='RECOVERY_REQUIRED'&&s.recovery_step===2&&s.owner==='unknown'&&$('support-ready').checked);
$('stop-evidence').textContent=s.stop_confirmed?'模拟停止已确认':'停止未确认，任务受理保持关闭';$('fault-detail').textContent=s.fault||'无待处理故障';$('recovery-next').textContent=p==='STARTUP_REQUIRED'?'下一步：双臂同时准备':p==='READY'?'可接收新任务':'恢复不重播中断动作';$('authority-note').textContent=p==='WSL_OWNED'?'Jetson 任务受理关闭；不会超时抢回控制权':p==='AWAITING_WSL'?'模拟输出已释放，等待 WSL 确认':'控制权代次 '+s.epoch;
$('tasks').replaceChildren();s.tasks.forEach(t=>{const row=document.createElement('tr');[t.taskId,t.operation,t.status,`${t.progress}/1`].forEach(v=>text(row,'td',v));$('tasks').appendChild(row);});$('events').replaceChildren();[...s.events].reverse().forEach(e=>{const row=document.createElement('div');text(row,'time',new Date(e.time*1000).toLocaleTimeString());text(row,'span',e.message);$('events').appendChild(row);});}
async function refresh(){try{state=await get('/api/state');$('connection').textContent='离线服务已连接';render();}catch(e){$('connection').textContent='连接中断';document.querySelectorAll('button').forEach(b=>b.disabled=true);}}
function plot(){
  if(!curves)return;
  const list=curves[$('curve-operation').value];
  if(!list?.length)return;
  selectedCurve=Math.max(0,Math.min(selectedCurve,list.length-1));
  $('curve-tabs').replaceChildren();
  list.forEach((s,i)=>{const b=text($('curve-tabs'),'button',`${s.label.replace('Rail recorded ','')} · ${s.duration.toFixed(2)}s`);b.classList.toggle('selected',i===selectedCurve);b.onclick=()=>{selectedCurve=i;plot();};});
  const s=list[selectedCurve],canvas=$('plot'),r=canvas.getBoundingClientRect(),d=devicePixelRatio||1;
  canvas.width=Math.round(r.width*d);canvas.height=Math.round(r.height*d);
  const ctx=canvas.getContext('2d');ctx.scale(d,d);const w=r.width,h=r.height,pad=40;
  let lo=Infinity,hi=-Infinity;
  for(const p of s.points)for(const value of p.positions){lo=Math.min(lo,value);hi=Math.max(hi,value);}
  lo-=.1;hi+=.1;
  const time=p=>p.time_from_start.sec+p.time_from_start.nanosec*1e-9;
  const tmax=time(s.points.at(-1));ctx.font='11px system-ui';ctx.fillStyle='#707d85';ctx.strokeStyle='#dce3e6';
  for(let i=0;i<5;i++){const y=pad+(h-pad*2)*i/4;ctx.beginPath();ctx.moveTo(pad,y);ctx.lineTo(w-10,y);ctx.stroke();ctx.fillText((hi-(hi-lo)*i/4).toFixed(1),0,y+4);}
  ctx.fillText('rad',0,15);ctx.fillText(`${tmax.toFixed(2)} s`,w-64,h-7);$('legend').replaceChildren();
  s.jointNames.forEach((name,j)=>{ctx.strokeStyle=colors[j%colors.length];ctx.lineWidth=1.5;ctx.beginPath();s.points.forEach((p,i)=>{const x=pad+time(p)/tmax*(w-pad-10),y=pad+(hi-p.positions[j])/(hi-lo)*(h-2*pad);if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);});ctx.stroke();const label=text($('legend'),'span',name.replace('right_','').replace('left_','').replace('_joint',''));label.style.borderColor=colors[j%colors.length];});
}
$('initialize').onclick=()=>task();$('air').onclick=()=>task('AIR');$('ground').onclick=()=>task('GROUND');$('stop').onclick=()=>post('/api/stop',{});$('fault').onclick=()=>post('/api/simulate-fault',{});$('release').onclick=()=>handoff('release_jetson');$('wsl-claim').onclick=()=>handoff('claim_wsl');$('wsl-release').onclick=()=>handoff('release_wsl');$('acquire').onclick=()=>handoff('acquire_jetson');$('confirm-stop').onclick=()=>recovery('confirm_stop');$('inspect').onclick=()=>recovery('inspect');$('reset').onclick=()=>recovery('reset');$('reconcile').onclick=()=>recovery('reconcile_owner');$('mapping').onchange=()=>post('/api/mapping',{AIR:$('mapping').value,GROUND:$('mapping').value==='push'?'pull':'push'});$('curve-operation').onchange=()=>{selectedCurve=0;plot();};document.querySelectorAll('input[type=checkbox]').forEach(x=>x.onchange=render);new ResizeObserver(plot).observe($('plot'));
(async()=>{try{token=(await get('/api/session')).token;curves=await get('/api/curves');plot();await refresh();setInterval(refresh,600);}catch(e){notice(e.message);}})();
