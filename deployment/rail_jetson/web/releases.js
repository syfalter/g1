const actionLabels = {push:'滑轨推进',pull:'滑轨拉出',dring_left:'拉环左拨',dring_right:'拉环右拨',
  dring_left_segment:'拉环左拨单段',dring_right_segment:'拉环右拨单段',
  gear_above_to_lower:'准备位到下杆位',gear_above_to_upper:'准备位到上杆位',
  gear_lower_to_upper:'下杆位到上杆位',gear_upper_to_lower:'上杆位到下杆位'};
let releaseRows=[], loadedReleaseHash=null, loadingCurves=false;
function releaseDetail(){
  const row=releaseRows.find(r=>r.version===$('release-select').value);
  $('release-actions').replaceChildren();
  if(row)Object.entries(row.actions).forEach(([name,a])=>{
    const tr=document.createElement('tr');
    [actionLabels[name]||name,`${a.slot} / ${a.scope==='segment'?'单段':'完整循环'}`,
      a.verification==='executed'?'控制序列执行成功':'仅规划，未实机验证',a.source].forEach(v=>text(tr,'td',v));
    $('release-actions').appendChild(tr);
  });
  $('release-status').textContent=row?`${row.release_kind==='patch'?'模块更新，其他动作保留':'完整版本'} · SHA256 ${row.sha256.slice(0,16)}`:'尚无待启用版本';
}
async function releasesRefresh(){
  try{
    const selected=$('release-select').value,data=await get('/api/releases');
    releaseRows=data.releases;$('release-select').replaceChildren();
    releaseRows.forEach(r=>{const option=text($('release-select'),'option',r.version);option.value=r.version;});
    if(releaseRows.some(r=>r.version===selected))$('release-select').value=selected;
    releaseDetail();
    if(data.errors.length)notice(`有 ${data.errors.length} 个损坏版本被排除，未启用`);
  }catch(e){notice(e.message);}
}
async function updateReleaseView(){
  if(!state)return;
  $('active-version').textContent=state.release_version||'历史滑轨基线';
  const idle=state.owner==='jetson'&&!state.active&&['READY','STARTUP_REQUIRED'].includes(state.phase);
  enable('air',state.owner==='jetson'&&state.phase==='READY'&&state.availableActions.includes(state.mapping.AIR));
  enable('ground',state.owner==='jetson'&&state.phase==='READY'&&state.availableActions.includes(state.mapping.GROUND));
  enable('activate-release',idle&&!!$('release-select').value);
  enable('rollback-release',idle&&!!state.previous_release?.version);
  enable('run-action',state.owner==='jetson'&&state.phase==='READY'&&!!$('action-select').value);
  enable('run-nws',state.owner==='jetson'&&state.phase==='READY'&&
    state.availableActions.includes('dring_left')&&state.availableActions.includes('dring_right'));
  if(loadedReleaseHash===state.bundle||loadingCurves)return;
  loadingCurves=true;
  try{
    const requested=state.bundle,next=await get('/api/curves');
    curves=next;selectedCurve=0;
    for(const id of ['curve-operation','action-select']){
      const selected=$(id).value;$(id).replaceChildren();
      Object.keys(curves).forEach(name=>{const option=text($(id),'option',actionLabels[name]||name);option.value=name;});
      if(selected in curves)$(id).value=selected;
    }
    loadedReleaseHash=requested;plot();
  }catch(e){notice(e.message);}finally{loadingCurves=false;}
}
$('release-select').onchange=releaseDetail;
$('refresh-releases').onclick=releasesRefresh;
$('activate-release').onclick=async()=>{
  await post('/api/releases/activate',{version:$('release-select').value,expectedSha256:state.bundle});
  await releasesRefresh();await updateReleaseView();
};
$('rollback-release').onclick=async()=>{
  if(!state.previous_release?.version)return;
  await post('/api/releases/activate',{version:state.previous_release.version,expectedSha256:state.bundle});
  await releasesRefresh();await updateReleaseView();
};
$('run-action').onclick=()=>post('/api/task',{taskId:`WEB-ACTION-${Date.now()}`,taskType:'RECORDED_ACTION',parameters:{action:$('action-select').value}});
$('run-nws').onclick=()=>post('/api/task',{taskId:`WEB-NWS-${Date.now()}`,taskType:'NWS_CYCLE',parameters:{count:1}});
releasesRefresh();setInterval(updateReleaseView,700);
