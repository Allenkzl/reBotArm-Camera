"use strict";
(() => {
  const root = document.querySelector('#shooting-studio');
  const $ = selector => root.querySelector(selector);
  const esc = value => String(value ?? '').replace(/[&<>"']/g, char => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[char]));
  let status = {}, online = false, lastReceived = 0, pending = false, activeGroup = null, rowsKey = '', navKey = '';
  let urgentCount = 0, awaitingFresh = false, commandEpoch = 0;
  const urgentPaths = new Set();
  const blocked = () => pending || urgentCount > 0 || awaitingFresh;
  const groups = () => status.shooting_groups || [];
  const group = () => groups().find(item => String(item.id) === String(activeGroup));
  const recording = () => status.recording_slot != null && Number(status.recording_slot) > 0;
  const active = () => recording() || !!status.group_playback || ['starting','replaying','replay_starting','group_waiting','homing'].includes(status.mode);
  const editable = () => online && !blocked() && !active();
  const ready = () => online && !blocked() && status.enabled === true && (status.connected === true || status.dry_run === true) && ['running','holding'].includes(status.mode) && !active();
  const isRecording = block => recording() && block.slot != null && Number(block.slot) === Number(status.recording_slot);
  const note = (message, error = false) => { $('#studio-message').textContent = message; $('#studio-message').dataset.error = String(error); };
  function eventLog(event) {
    const row = document.createElement('div'); row.className = `event ${event.level === 'danger' || event.level === 'error' ? 'danger' : ''}`;
    row.textContent = `[${event.ts || new Date().toLocaleTimeString('zh-CN', {hour12:false})}] ${event.message || ''}`;
    $('#runtime-events').prepend(row);
    while ($('#runtime-events').children.length > 100) $('#runtime-events').lastChild.remove();
    if (['danger','error'].includes(event.level)) note(event.message || '运行异常', true);
  }
  function update(next) {
    const firstSnapshot = !Object.hasOwn(status, 'updated_ms');
    const nextTime = Number(next.updated_ms) || 0, currentTime = Number(status.updated_ms) || 0;
    const nextRevision = Number(next.revision) || 0, currentRevision = Number(status.revision) || 0;
    if (nextTime < currentTime || (nextTime === currentTime && nextRevision < currentRevision)) return false;
    status = next; online = true; lastReceived = Date.now(); render();
    if (firstSnapshot) note(status.enabled ? '已连接机械臂服务，请确认当前运行状态' : '已连接，点击「启动机械臂」开始操作');
    return true;
  }
  async function mutate(path, data = {}, success = '操作已提交') {
    if (blocked() || !online) return;
    const epoch = commandEpoch;
    pending = true; render();
    try {
      const response = await fetch(path, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(data), signal:AbortSignal.timeout(15000)});
      const payload = await response.json();
      if (!response.ok) throw new Error(payload.error || response.statusText);
      if (epoch !== commandEpoch) return;
      if (!update(payload)) return;
      note(success); eventLog({message:success}); return payload;
    } catch (error) {
      if (epoch !== commandEpoch) return;
      note(`操作失败：${error.message}。请核实设备状态后再操作。`, true);
      eventLog({level:'danger',message:`${path}：${error.message}`});
      if (error.name === 'TimeoutError' || error.name === 'TypeError') online = false;
    } finally { pending = false; render(); }
  }
  function render() {
    if (!groups().some(item => String(item.id) === String(activeGroup))) activeGroup = groups()[0]?.id ?? null;
    const g = group(), enabled = status.enabled === true, busy = active();
    root.dataset.enabled = String(online && enabled);
    const modes = {idle:'未启动',starting:'正在启动',running:'拖动模式',holding:'保持位置',replaying:'播放中',replay_starting:'准备播放',group_waiting:'动作间隔中',homing:'正在返回 Home',error:'异常，请查看详情',disabled:'已失能'};
    $('#studio-status').textContent = !online ? '连接中断 / 状态未知' : `${modes[status.mode] || status.mode || '状态未知'}${enabled ? ' · 已使能' : ''}${recording() ? ' · 录制中' : ''}`;
    $('#connection-label').textContent = !online ? '未连接网页服务' : status.dry_run ? '模拟服务 · 不控制电机' : '实机服务 · J1–J6';
    $('#studio-start').disabled = !online || blocked() || enabled || busy;
    $('#studio-return').disabled = !online || blocked() || !enabled || ['starting','homing'].includes(status.mode);
    // Stopping remains reachable even while another request is in flight.
    $('#studio-emergency').disabled = !online || (!enabled && status.mode !== 'starting') || urgentPaths.has('/api/stop');
    $('#direct-disable').disabled = !online || (!enabled && status.mode !== 'starting') || urgentPaths.has('/api/emergency-disable');
    $('#studio-add-group').disabled = !editable();
    const nextNav = JSON.stringify([groups().map(item => [item.id,item.name,item.blocks.length]),activeGroup,busy,blocked(),online]);
    if (nextNav !== navKey) {
      $('#studio-groups').innerHTML = groups().map(item => `<button type="button" class="group-tab" data-group="${esc(item.id)}" aria-pressed="${String(item.id)===String(activeGroup)}"><span>${esc(item.name)}</span><span class="group-count">${item.blocks.length} 动作</span></button>`).join('');
      navKey = nextNav;
    }
    const name = $('#studio-group-name'), gap = $('#studio-gap');
    if (document.activeElement !== name) name.value = g?.name || '';
    if (document.activeElement !== gap) gap.value = g?.interval_s ?? 5;
    name.disabled = gap.disabled = !g || !editable();
    $('#studio-delete-group').disabled = !g || groups().length <= 1 || !editable();
    $('#studio-add-action').disabled = !g || !editable();
    const recorded = g?.blocks.filter(block => block.trajectory).length || 0;
    $('#studio-summary').textContent = g ? `${g.blocks.length} 个动作 · ${recorded} 个已录制` : '等待拍摄组数据';
    const playing = !!status.group_playback || ['replaying','replay_starting','group_waiting'].includes(status.mode);
    $('#studio-play-group').textContent = playing ? '停止播放' : '播放整组';
    $('#studio-play-group').disabled = playing ? !online || blocked() : !g || !ready() || !g.blocks.length || recorded !== g.blocks.length;
    const nextRows = JSON.stringify([g,online,blocked(),busy,enabled,status.connected,status.dry_run,status.recording_slot,status.group_playback?.block_id]);
    if (nextRows !== rowsKey) {
      $('#studio-blocks').innerHTML = g ? g.blocks.map((block,index) => {
        const rec = isRecording(block), selected = String(status.group_playback?.block_id) === String(block.id), trajectory = block.trajectory;
        return `<div class="action-row ${rec || selected ? 'is-active':''}"><span class="action-index">${String(index+1).padStart(2,'0')}</span><div class="action-content"><input class="action-name" data-name="${esc(block.id)}" aria-label="动作 ${index+1} 名称" value="${esc(block.name)}" maxlength="60" ${!editable()?'disabled':''}><div class="action-meta"><span class="record-dot"></span><span class="${rec?'recording-label':''}" ${rec?'id="recording-meta"':''}>${rec ? '正在录制' : trajectory ? `已录制 · ${Number(trajectory.duration_s).toFixed(1)} 秒 · ${Number(trajectory.sample_count)||0} 点` : '尚未录制'}</span><span>J1–J6 轨迹</span></div></div><div class="action-tools"><button type="button" class="small-button" data-record="${esc(block.id)}" ${rec ? (!online||blocked()?'disabled':'') : (!ready()?'disabled':'')}>${rec?'结束录制':trajectory?'重新录制':'录制动作'}</button><button type="button" class="small-button" data-play="${esc(block.id)}" ${!ready()||!trajectory?'disabled':''}>单独播放</button><button type="button" class="delete-action" data-remove="${esc(block.id)}" aria-label="删除动作 ${index+1}" ${!editable()||g.blocks.length<=1?'disabled':''}>×</button></div></div>`;
      }).join('') : '<p class="empty-state">正在读取拍摄组…</p>';
      rowsKey = nextRows;
    }
    if ($('#recording-meta')) $('#recording-meta').textContent = `正在录制 · ${Number(status.recording_samples)||0} 点`;
    $('#studio-order').textContent = g?.blocks.map((_,index)=>String(index+1).padStart(2,'0')).join(' → ') || '';
    const playback = status.group_playback;
    $('#studio-progress').hidden = !playing;
    if (playback) {
      const phase = {transition:'移动到动作起点',playing:'播放动作',waiting:'等待下一个动作'}[playback.phase] || playback.phase;
      $('#studio-progress-label').textContent = `整组播放 · ${playback.index} / ${playback.total} · ${phase}${playback.phase==='waiting'?` · 剩余 ${Math.ceil(Number(playback.remaining_s)||0)} 秒`:''}`;
      $('#studio-progress-fill').style.width = `${Math.min(100,Math.max(0,(Number(playback.index)-1)/Math.max(1,Number(playback.total))*100))}%`;
    } else if (playing) { $('#studio-progress-label').textContent = '正在播放单个动作…'; $('#studio-progress-fill').style.width = '0%'; }
    $('#runtime-summary').textContent = `控制状态：${status.mode || '未知'} · 电机连接：${!online?'未知':status.connected?'已连接':'未连接'} · 日志：${status.log_path || '暂无'}`;
    $('#runtime-joints').innerHTML = ['J1','J2','J3','J4','J5','J6'].map((joint,index) => `<span>${joint} ${online && status.connected && Number.isFinite(status.q_deg?.[index]) ? status.q_deg[index].toFixed(1)+'°':'—'}</span>`).join('');
    $('#runtime-profile').textContent = `固定相机总重：950 克 · 补偿比例：${(status.joint_percent || [80,80,80,80,80,80]).join(' / ')}% · 速度：25%`;
  }
  function ids(blockId) { return {group_id:group().id, ...(blockId == null ? {} : {block_id:group().blocks.find(block=>String(block.id)===String(blockId)).id})}; }
  $('#studio-start').onclick = () => { if (confirm('启动机械臂并进入可拖动录制模式？请确认人在旁边且周围无障碍。')) mutate('/api/start',{camera_profile:'heavy',joint_percent:status.joint_percent || [80,80,80,80,80,80]},'启动请求已提交，请等待设备确认'); };
  $('#studio-return').onclick = () => { if (confirm('关闭机械臂：中止当前动作，返回 Home 零位后失能。请在旁保护，回到安全姿态前保持支撑。')) mutate('/api/home-disable',{},'已请求返回 Home，完成后将失能；请观察设备状态'); };
  async function urgent(path, message) {
    // Emergency controls bypass ordinary pending requests, but block new motion.
    if (!online || urgentPaths.has(path)) return;
    commandEpoch += 1; urgentCount += 1; awaitingFresh = true; urgentPaths.add(path); render();
    try {
      const response = await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:'{}',signal:AbortSignal.timeout(10000)});
      const payload = await response.json(); if (!response.ok) throw new Error(payload.error || response.statusText);
      update(payload); note(message);
    } catch (error) { note(`停止请求未确认：${error.message}，请现场处理`,true); }
    finally { urgentCount -= 1; commandEpoch += 1; urgentPaths.delete(path); render(); poll(); }
  }
  $('#studio-emergency').onclick = () => urgent('/api/stop','已请求停止当前动作并保持使能；请核实设备状态');
  $('#direct-disable').onclick = () => { if (confirm('立即失能会失去支撑力，机械臂可能下落。确认已人工支撑机械臂，并执行紧急失能？')) urgent('/api/emergency-disable','已请求紧急失能，请现场确认机械臂状态'); };
  $('#studio-payload').onclick = () => { const detail=$('#payload-detail'); detail.hidden=!detail.hidden; $('#studio-payload').setAttribute('aria-expanded',String(!detail.hidden)); };
  $('#studio-groups').onclick = event => { const button=event.target.closest('[data-group]'); if(button){activeGroup=button.dataset.group;rowsKey='';render();} };
  $('#studio-add-group').onclick = async () => { const previous = new Set(groups().map(item=>String(item.id))); const result = await mutate('/api/groups/add',{},'已新增拍摄组（默认四个动作，间隔五秒）'); const added = result?.shooting_groups?.find(item=>!previous.has(String(item.id))); if(added){activeGroup=added.id;render();} };
  $('#studio-delete-group').onclick = () => { if(group() && confirm(`删除「${group().name}」及组内动作关联？此操作不可撤销。`)) mutate('/api/groups/delete',ids(),'已删除拍摄组'); };
  $('#studio-group-name').onchange = event => { const name=event.target.value.trim(); if(!name){render();return;} mutate('/api/groups/update',{...ids(),name},'拍摄组名称已保存'); };
  $('#studio-gap').onchange = event => { const value=Number(event.target.value); if(event.target.value==='' || !Number.isFinite(value) || value<0 || value>60){note('动作间隔请填写 0–60 秒',true);event.target.value=group()?.interval_s??5;return;} mutate('/api/groups/update',{...ids(),interval_s:value},'动作间隔已保存'); };
  $('#studio-add-action').onclick = () => mutate('/api/blocks/add',ids(),'已新增动作块');
  $('#studio-play-group').onclick = () => { if(status.group_playback || ['replaying','replay_starting','group_waiting'].includes(status.mode)){mutate('/api/stop',{},'已请求停止播放并保持使能');return;} if(group() && confirm(`按顺序播放「${group().name}」中的 ${group().blocks.length} 个动作？动作间隔 ${group().interval_s} 秒，完成后保持使能。`)) mutate('/api/groups/play',{...ids(),speed:.25},'整组播放已提交'); };
  $('#studio-blocks').onchange = event => { const input=event.target.closest('[data-name]'); if(input && editable()){const name=input.value.trim();if(!name){rowsKey='';render();return;}mutate('/api/blocks/update',{...ids(input.dataset.name),name},'动作名称已保存');} };
  $('#studio-blocks').onclick = event => {
    const remove=event.target.closest('[data-remove]'), record=event.target.closest('[data-record]'), play=event.target.closest('[data-play]');
    if(remove && editable()){if(confirm('删除这个动作块？此操作不可撤销。'))mutate('/api/blocks/delete',ids(remove.dataset.remove),'动作块已删除');return;}
    if(record){const block=group()?.blocks.find(item=>String(item.id)===record.dataset.record);if(!block)return;if(isRecording(block)){mutate('/api/blocks/stop-recording',ids(block.id),'动作录制已保存');return;}if(!ready())return;if(block.trajectory && !confirm('重新录制将替换此动作的已有轨迹，继续？'))return;mutate('/api/blocks/record',ids(block.id),'正在录制，请手动拖动机械臂，完成后点击结束录制');return;}
    if(play && ready() && confirm('播放这个动作？机械臂将先移动到记录起点，再执行轨迹。'))mutate('/api/blocks/play',{...ids(play.dataset.play),speed:.25},'动作播放已提交');
  };
  let polling=false;
  async function poll() {
    if (polling) return;
    polling = true; const epoch = commandEpoch;
    try {
      const response = await fetch('/api/status',{cache:'no-store',signal:AbortSignal.timeout(5000)});
      if (!response.ok) throw new Error(response.statusText);
      const next = await response.json();
      if (epoch === commandEpoch && update(next) && urgentCount === 0) { awaitingFresh = false; render(); }
    } catch (error) {
      if (Date.now()-lastReceived > 5000) { online=false; render(); note('网页服务连接中断，设备状态未知。请检查网络和现场机械臂状态。',true); }
    } finally { polling=false; }
  }
  const source=new EventSource('/api/events');
  source.onmessage=message=>{try{const payload=JSON.parse(message.data);if(payload.type==='status')update(payload.status);if(payload.type==='event')eventLog(payload.event);}catch(error){eventLog({level:'danger',message:`状态消息无法读取：${error.message}`});}};
  render();poll();setInterval(poll,2000);
})();
