"use strict";
(() => {
  const playbackSpeed = .5;
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
  const playbackHint = () => !online ? '网页服务未连接' : !status.enabled ? '请先启动机械臂' : blocked() ? '请等待当前请求完成' : active() ? '请先结束录制或停止当前动作' : !['running','holding'].includes(status.mode) ? '请先处理机械臂异常状态' : '';
  const pointMode = () => status.camera_profile === 'heavy';
  const cameraReady = () => status.camera?.ready === true && !status.camera?.requires_review && !status.camera?.busy;
  const photoPoints = trajectory => Array.isArray(trajectory?.photo_points) ? trajectory.photo_points : [];
  const precisionReplay = trajectory => status.camera_profile === 'unloaded_50g' &&
    trajectory?.camera_profile === 'unloaded_50g' && photoPoints(trajectory).length > 0;
  const photoCount = blocks => blocks.reduce((count, block) => count + photoPoints(block.trajectory).length, 0);
  const cameraHint = () => !online ? '网页服务未连接' : status.camera?.requires_review ? '拍照结果待核实，请检查相机后重新点击连接确认' : !status.camera?.bridge_online ? '相机桥未连接' : !status.camera?.device_online ? 'ESP32 未连接' : status.camera?.busy ? '相机正在处理操作' : !status.camera?.connected ? '请先连接蓝牙相机' : '相机尚未就绪';
  const replayHint = blocks => playbackHint() || (photoCount(blocks) && !cameraReady() ? cameraHint() : '');
  const recordingError = slot => String(status.recording_errors?.[String(slot)] || '').replace(/outside soft limits/gi, '超出软件范围').replace(/deg/g, '°');
  const isRecording = block => recording() && block.slot != null && Number(block.slot) === Number(status.recording_slot);
  const note = (message, error = false) => { $('#studio-message').textContent = message; $('#studio-message').dataset.error = String(error); };
  function eventLog(event) {
    const row = document.createElement('div'); row.className = `event ${event.level === 'danger' || event.level === 'error' ? 'danger' : ''}`;
    row.textContent = `[${event.ts || new Date().toLocaleTimeString('zh-CN', {hour12:false})}] ${event.message || ''}`;
    $('#runtime-events').prepend(row);
    while ($('#runtime-events').children.length > 100) $('#runtime-events').lastChild.remove();
    if (['danger','error'].includes(event.level)) note(event.message || '运行异常', true);
  }
  let angleTargets = Array(6).fill(null);
  const angleGrid = $('#live-angles-grid');
  let referenceCaptured = false;
  function renderAngleTargets() {
    angleGrid.innerHTML = angleTargets.map((target, i) => `
    <div class="live-angle">
      <span>J${i + 1}</span><strong class="live-angle-value">—</strong>
      <small>目标 ${Number.isFinite(target) && Number.isFinite(status.home_q_deg?.[i]) ? (target-status.home_q_deg[i]).toFixed(2) + '°' : '—'} · <span class="live-angle-error">偏差 —</span></small>
    </div>`).join('');
  }
  renderAngleTargets();
  function renderLiveAngles() {
    const live = online && Date.now() - lastReceived < 1000 &&
      status.connected === true && status.enabled === true &&
      Array.isArray(status.q_deg) && status.q_deg.length === 6 &&
      status.q_deg.every(Number.isFinite);
    $('#live-angles-status').textContent = live ? '实时反馈 · 目标容差 ±0.5°' :
      online && status.enabled === false ? '已失能 · 暂无实时反馈' : '等候实时反馈';
    const homeReading = referenceCaptured && online && status.enabled === false &&
      status.message?.includes('已返回 Home') && Array.isArray(status.q_deg) &&
      status.q_deg.length === 6 && status.q_deg.every(Number.isFinite);
    if (homeReading) $('#live-angles-status').textContent = '回 Home 时读数 · 已失能';
    Array.from(angleGrid.children).forEach((card, i) => {
      const angle = live || homeReading ? status.q_deg[i] : null;
      const error = angle !== null && Number.isFinite(angleTargets[i]) ? angle - angleTargets[i] : null;
      card.dataset.state = live && error !== null ? (Math.abs(error) <= .5 ? 'ready' : 'adjust') : 'idle';
      card.querySelector('.live-angle-value').textContent = angle !== null && Number.isFinite(status.home_q_deg?.[i]) ? (angle-status.home_q_deg[i]).toFixed(2) + '°' : '—';
      card.querySelector('.live-angle-error').textContent = error !== null
        ? '偏差 ' + (error > 0 ? '+' : '') + error.toFixed(2) + '°' : '偏差 —';
    });
  }

  let holdPreviewStarted = null;
  const framingCue = $('#framing-cue');
  function renderFramingCue() {
    const message = status.message || '';
    const mode = status.mode;
    const capture = status.photo_capture;
    let title = '正在获取机械臂状态', detail = '等待工作台连接', tone = 'waiting';
    if (!online || Date.now() - lastReceived >= 2000) {
      title = '连接中断 · 机械臂状态未知';
      detail = '请检查网络及现场机械臂状态';
      tone = 'error';
    } else if (mode === 'error' || (status.enabled && capture?.phase === 'error')) {
      title = '操作已停止 · 请检查';
      detail = mode === 'error' ? message : capture.message;
      tone = 'error';
    } else if (mode === 'starting') {
      title = '机械臂正在启动'; detail = '正在检查反馈并准备控制'; tone = 'moving';
    } else if (mode === 'homing') {
      title = '机械臂正在返回 Home'; detail = message; tone = 'moving';
    } else if (!status.enabled) {
      title = '机械臂已失能'; detail = message.includes('已返回 Home') ? '已返回 Home，等待下一次操作' : '等待启动机械臂'; tone = 'idle';
    } else if (/正在返回 Home/.test(message)) {
      title = '拍照完成 · 正在返回 Home'; detail = message; tone = 'moving';
    } else if (mode === 'replaying' && /正在核对六轴/.test(message)) {
      title = '正在校正拍摄点 · 等待到位'; detail = message; tone = 'waiting';
    } else if (mode === 'replaying' && ['triggering','waiting_result','post_delay'].includes(capture?.phase)) {
      title = {triggering:'已到达拍摄点 · 正在触发快门', waiting_result:'已到达拍摄点 · 等待相机结果', post_delay:'拍照完成 · 正在保持位置'}[capture.phase];
      detail = capture.message || message; tone = capture.phase === 'post_delay' ? 'ready' : 'waiting';
    } else if (mode === 'replaying' && /正在前往|平稳回放/.test(message)) {
      title = '机械臂正在运动 · 前往拍摄点'; detail = message; tone = 'moving';
    } else if (mode === 'replay_starting') {
      title = '正在准备回放'; detail = '正在检查动作与相机状态'; tone = 'waiting';
    } else if (mode === 'group_waiting') {
      title = '等待下一个动作'; detail = message; tone = 'waiting';
    } else if (mode === 'holding') {
      title = /已返回 Home/.test(message) ? '已返回 Home · 保持使能' :
        /精度测试通过|构图对照完成/.test(message) ? '已到达拍摄点 · 保持位置' : '机械臂已停止 · 保持使能';
      detail = message; tone = 'ready';
    } else if (mode === 'running' && /Home/.test(message)) {
      title = /保持到位/.test(message) ? '新 Home 保持到位' : '正在固定保持新 Home';
      detail = message; tone = /保持到位/.test(message) ? 'ready' : 'waiting';
    } else if (mode === 'running' && status.recording_slot != null && pointMode() && !cameraReady()) {
      title = '正在录制 · 相机尚未就绪';
      detail = '空格和鼠标打点需要相机就绪。请先停止录制，再连接蓝牙相机。';
      tone = 'error';
    } else if (mode === 'running' && status.recording_slot != null) {
      title = /等待单次快门/.test(message) ? '点位已记录 · 等待拍照结果' :
        /快门指令已完成/.test(message) ? '点位已记录 · 拍照指令完成' : '正在录制 · 请手动摆位';
      detail = /点位|快门|拍照点/.test(message) ? message : '摆到目标后稳住，按空格记录点位';
      tone = 'waiting';
    } else if (mode === 'replaying') {
      title = /已到拍照点|请拍照/.test(message) ? '已到达拍摄点' : '机械臂正在执行动作';
      detail = message || '执行中，请保持活动范围畅通'; tone = 'moving';
    } else {
      title = '机械臂已启动 · 等待操作'; detail = '可开始录制或回放'; tone = 'ready';
    }
    const cameraOperation = status.camera?.operation;
    if (online && Date.now() - lastReceived < 2000 && ['running','idle','disabled'].includes(mode) &&
        status.camera?.busy && cameraOperation?.action !== 'shutter') {
      detail = cameraOperation?.message || '正在处理相机连接操作';
    }
    $('#framing-cue-heading').textContent = title;
    $('#framing-cue-detail').textContent = detail;
    framingCue.dataset.state = tone;
    framingCue.hidden = false;
  }

  function renderHoldCue() {
    renderFramingCue();
    renderLiveAngles();
    const panel = $('#hold-cue');
    const preview = $('#hold-cue-preview');
    const isHold = /(?:^|\/)hold_diagnostic_/.test(status.log_path || '');
    const activeHold = isHold && status.mode === 'replaying';
    const fresh = online && Date.now() - lastReceived < 1000;
    const match = /^原位保持诊断：([0-9.]+)秒$/.exec(status.message || '');
    const liveSeconds = match ? Number(match[1]) : null;
    if (!online || status.enabled !== false) holdPreviewStarted = null;
    const demo = holdPreviewStarted !== null;
    const elapsed = demo ? (performance.now() - holdPreviewStarted) / 1000 : 0;
    const seconds = demo ? Math.max(0, 12 - elapsed) : liveSeconds;
    let phase = 'idle';
    let instruction = '等待测试，继续托稳';
    let detail = '出现松手提示前，请持续托住相机和支架。';
    let countdown = '—';
    if (demo || (activeHold && fresh && status.enabled === true && status.connected === true)) {
      phase = 'ready';
      instruction = '请继续托稳，等松手提示';
      detail = '正在确认起始位置。';
      if ((demo && elapsed >= 2) || (!demo && seconds !== null && Number.isFinite(seconds))) {
        countdown = seconds.toFixed(1) + ' 秒';
        phase = seconds > 2 ? 'release' : seconds > 0 ? 'ending' : 'done';
        instruction = phase === 'release' ? '现在缓慢松手' :
          phase === 'ending' ? '即将结束，保持当前托举状态' : '测试结束，请托稳相机';
        detail = '双手留在旁边。若相机明显下沉或抖动，立即托住并点「紧急停止」。';
      }
    } else if (activeHold) {
      phase = 'ready';
      instruction = '实时状态中断，请继续托稳';
      detail = '等待工作台恢复实时状态。';
    } else if (isHold && ['holding','error'].includes(status.mode)) {
      phase = 'done';
      instruction = '测试已结束，请托稳相机';
      detail = status.message || '请等待收尾指令。';
    }
    panel.dataset.phase = phase;
    $('#hold-cue-badge').textContent = demo ? '演示模式 · 请勿实际松手' : '无快门保持测试';
    $('#hold-cue-instruction').textContent = instruction;
    $('#hold-cue-detail').textContent = demo ? '这是界面演示。实际操作以正式测试的提示为准。' : detail;
    $('#hold-cue-seconds').textContent = countdown;
    preview.textContent = demo ? '退出演示' : '预览 10 秒倒计时';
    preview.disabled = !online || status.enabled !== false;
  }
  $('#hold-cue-preview').addEventListener('click', () => {
    if (!online || status.enabled !== false) return;
    holdPreviewStarted = holdPreviewStarted === null ? performance.now() : null;
    renderHoldCue();
  });
  setInterval(renderHoldCue, 100);

  function update(next) {
    const firstSnapshot = !Object.hasOwn(status, 'updated_ms');
    const nextTime = Number(next.updated_ms) || 0, currentTime = Number(status.updated_ms) || 0;
    const nextRevision = Number(next.revision) || 0, currentRevision = Number(status.revision) || 0;
    if (nextTime < currentTime || (nextTime === currentTime && nextRevision < currentRevision)) return false;
    status = next; online = true; lastReceived = Date.now();
    const atHome = status.mode === 'homing' || /已返回 Home|保持新 Home|Home 保持到位/.test(status.message || '');
    const reference = atHome ? status.home_q_deg : status.photo_capture?.precision_reference_deg;
    if (Array.isArray(reference) && reference.length === 6 && reference.every(Number.isFinite) &&
        (reference.some((x, i) => x !== angleTargets[i]) || !referenceCaptured ||
         $('#live-angle-title').dataset.reference !== (atHome ? 'home' : 'point'))) {
      angleTargets = reference.slice();
      referenceCaptured = true;
      $('#live-angle-title').dataset.reference = atHome ? 'home' : 'point';
      $('#live-angle-title').textContent = atHome ? '六轴实时角度 · 新 Home（相对零位）' : '六轴实时角度 · 当前打点（相对新 Home）';
      renderAngleTargets();
    }
    renderHoldCue();
    render();
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
      const message = typeof success === 'function' ? success(payload) : success;
      note(message); eventLog({message}); return payload;
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
    $('#studio-return').disabled = !online || (!enabled && status.mode !== 'starting') || status.mode === 'homing' || urgentPaths.has('/api/home-disable');
    $('#studio-home').disabled = !online || !enabled || recording() || status.mode === 'homing' || urgentPaths.has('/api/home-hold');
    // Stopping remains reachable even while another request is in flight.
    $('#studio-emergency').disabled = !online || (!enabled && status.mode !== 'starting') || urgentPaths.has('/api/stop');
    $('#direct-disable').disabled = !online || (!enabled && status.mode !== 'starting') || urgentPaths.has('/api/emergency-disable');
    $('#studio-add-group').disabled = !editable();
    renderCamera();
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
    $('#studio-play-group').disabled = playing ? !online || blocked() : !g || !ready() || recorded === 0 || (photoCount(g.blocks) > 0 && !cameraReady());
    const skipped = (g?.blocks.length || 0) - recorded;
    $('#studio-play-hint').textContent = playing ? '停止后保持使能，可继续录制或关闭机械臂。' : `${replayHint(g?.blocks || []) ? replayHint(g?.blocks || []) + ' · ' : ''}将按顺序播放 ${recorded} 个已保存动作，跳过 ${skipped} 个空动作，共 ${photoCount(g?.blocks || [])} 张照片${recorded ? '' : '；请先录制动作'}。`;
    const nextRows = JSON.stringify([g,online,blocked(),busy,enabled,status.mode,status.connected,status.dry_run,status.camera_profile,status.recording_slot,status.group_playback?.block_id,status.recording_errors,status.recording_photo_points,status.camera?.ready,status.camera?.requires_review,status.camera?.busy,status.camera?.bridge_online,status.camera?.device_online,status.camera?.connected]);
    if (nextRows !== rowsKey) {
      $('#studio-blocks').innerHTML = g ? g.blocks.map((block,index) => {
        const rec = isRecording(block), selected = String(status.group_playback?.block_id) === String(block.id), trajectory = block.trajectory;
        const error = recordingError(block.slot), precision = precisionReplay(trajectory);
        const pointReplay = pointMode() && trajectory?.camera_profile === 'heavy' && photoPoints(trajectory).length > 0;
        const hint = precision ? playbackHint() : pointMode() && trajectory && !pointReplay
          ? '请在 950 克配置下录制并按空格打点' : replayHint([block]);
        const points = rec ? (status.recording_photo_points || []) : photoPoints(trajectory);
        const canDeletePoint = online && !blocked() && (rec || editable());
        return `<div class="action-row ${rec || selected ? 'is-active':''}">
          <span class="action-index">${String(index+1).padStart(2,'0')}</span>
          <div class="action-content">
            <input class="action-name" data-name="${esc(block.id)}" aria-label="动作 ${index+1} 名称" value="${esc(block.name)}" maxlength="60" ${!editable()?'disabled':''}>
            <div class="action-meta"><span class="record-dot"></span><span class="${rec?'recording-label':''}" ${rec?'id="recording-meta"':''}>${rec ? '正在录制' : trajectory ? `已保存 · ${Number(trajectory.duration_s).toFixed(1)} 秒 · ${Number(trajectory.sample_count)||0} 点` : '暂无已保存录制'}</span><span>J1–J6 轨迹</span></div>
            <p class="photo-point-count">${points.length} 个拍照点${rec ? (pointMode() ? ' · 空格记点并拍照' : ' · 仅记录位置') : ''}</p>
            ${points.length ? `<div class="photo-points">${points.map((point, pointIndex) => `<span class="photo-point">拍照点 ${pointIndex+1} · ${Number(point.t || 0).toFixed(1)} 秒<button type="button" class="delete-photo-point" data-delete-point="${esc(point.id)}" data-point-block="${esc(block.id)}" aria-label="删除拍照点 ${pointIndex+1}" ${canDeletePoint ? '' : 'disabled'}>×</button></span>`).join('')}</div>` : ''}
            ${rec ? `<p class="photo-hotkey">按 <kbd>空格</kbd>${pointMode() ? '保存六轴位置并拍照，请保持姿势至结果出现' : '保存六轴位置'}。</p>` : ''}
            ${error ? `<p class="recording-error" role="status">录制未保存：${esc(error)}${trajectory ? '；原有已保存轨迹仍保留，播放将使用原轨迹。' : '；请调整姿态后重新录制。'}</p>` : ''}
          </div>
          <div class="action-tools">
            <button type="button" class="small-button" data-record="${esc(block.id)}" ${rec ? (!online||blocked()?'disabled':'') : (!ready()?'disabled':'')}>${rec?'停止录制':trajectory?'重新录制':'开始录制'}</button>
            ${rec ? `<button type="button" class="small-button mark-photo-point" data-mark-point="${esc(block.id)}" ${online && !blocked() ? '' : 'disabled'}>设置拍照点</button>` : ''}
            ${trajectory ? `<button type="button" class="small-button" data-play="${esc(block.id)}" ${!ready() || !!hint ? 'disabled':''} title="${esc(hint || (precision ? '到首个打点测试六轴 ±0.5°，不触发快门' : `播放已保存轨迹，拍摄 ${photoPoints(trajectory).length} 张照片`))}">${precision ? '无快门精度回放' : pointMode() ? '按点回放' : '单独播放'}</button>${hint ? `<span class="play-hint">${esc(hint)}</span>` : ''}` : ''}
            <button type="button" class="delete-action" data-remove="${esc(block.id)}" aria-label="删除动作 ${index+1}" ${!editable()||g.blocks.length<=1?'disabled':''}>×</button>
          </div>
        </div>`;
      }).join('') : '<p class="empty-state">正在读取拍摄组…</p>';
      rowsKey = nextRows;
    }
    if ($('#recording-meta')) $('#recording-meta').textContent = `正在录制 · ${Number(status.recording_samples)||0} 点`;
    $('#studio-order').textContent = g?.blocks.map((block,index)=>block.trajectory ? String(index+1) : null).filter(Boolean).join(' → ') || '暂无已保存动作';
    const playback = status.group_playback;
    $('#studio-progress').hidden = !playing;
    if (playback) {
      const phase = {transition:'移动到动作起点',playing:'播放动作',waiting:'等待下一个动作'}[playback.phase] || playback.phase;
      $('#studio-progress-label').textContent = `整组播放 · ${playback.index} / ${playback.total} · ${phase}${playback.phase==='waiting'?` · 剩余 ${Math.ceil(Number(playback.remaining_s)||0)} 秒`:''}`;
      $('#studio-progress-fill').style.width = `${Math.min(100,Math.max(0,(Number(playback.index)-1)/Math.max(1,Number(playback.total))*100))}%`;
    } else if (playing) { $('#studio-progress-label').textContent = status.message?.startsWith('原位保持诊断：') ? status.message : '正在播放单个动作…'; $('#studio-progress-fill').style.width = '0%'; }
    const capture = status.photo_capture;
    $('#photo-capture-progress').hidden = !capture;
    if (capture) {
      const phase = {settling:'停稳中',triggering:'正在发出快门指令',waiting_result:'等待相机结果',post_delay:'拍照后停留',complete:'拍照点已完成',error:'拍照异常'}[capture.phase] || capture.phase;
      $('#photo-capture-progress').dataset.error = String(capture.phase === 'error');
      $('#photo-capture-progress').textContent = capture.precision_reference_deg && !capture.phase
        ? status.message
        : `拍照 ${capture.index || '—'} / ${capture.total || '—'} · ${phase}${capture.message ? ' · '+capture.message : ''}`;
    }
    $('#runtime-summary').textContent = `控制状态：${status.mode || '未知'} · 电机连接：${!online?'未知':status.connected?'已连接':'未连接'} · 日志：${status.log_path || '暂无'}`;
    const profile = status.camera_profiles?.[status.camera_profile];
    const grams = Number.isFinite(profile?.total_camera_kg) ? Math.round(profile.total_camera_kg * 1000) : null;
    $('#payload-weight').textContent = grams === null ? '—' : String(grams);
    $('#payload-rated').value = grams === null ? '' : String(grams);
    $('#payload-mode').textContent = profile?.label || '等待负载配置';
    $('#payload-note').textContent = profile?.note || '';
    $('#runtime-profile').textContent = `负载配置：${profile?.label || '等待配置'}${pointMode() ? ' · 单动作按点拍照' : ''} · 补偿比例：${(status.joint_percent || [80,80,80,80,80,80]).join(' / ')}% · 速度：${playbackSpeed * 100}%`;
  }
  function renderCamera() {
    const camera = status.camera || {}, usable = editable() && !camera.busy;
    $('#camera-connect').disabled = !usable || !camera.bridge_online || !camera.device_online || (!!camera.connected && !camera.requires_review);
    $('#camera-disconnect').disabled = !usable || !camera.bridge_online || !camera.connected;
    $('#camera-status').textContent = !online ? '相机状态未知 · 服务未连接' : `ESP32 ${camera.device_online ? '在线' : '离线'} · 蓝牙${camera.connected ? '已连接' : '未连接'}${camera.camera_name ? ' · '+camera.camera_name : ''}${camera.busy ? ' · 处理中' : ''}`;
    $('#camera-operation').textContent = camera.requires_review ? '拍照结果未知，请先人工核实，重新连接确认后才能继续。不会自动补拍。' : camera.operation?.message || (camera.ready ? '相机已就绪' : '首次配对请使用原有快门网页完成。');
    $('#camera-error').hidden = !camera.error;
    $('#camera-error').textContent = camera.error || '';
    const delay = $('#camera-post-delay');
    if (document.activeElement !== delay) delay.value = camera.post_delay_s ?? 1;
    delay.disabled = !usable;
    $('#camera-save-settings').disabled = !usable;
    $('#camera-status').title = camera.firmware ? `固件 ${camera.firmware}` : '固件状态未知';
  }
  function currentRecordingBlock() {
    for (const g of groups()) {
      const block = g.blocks.find(isRecording);
      if (block) return {group_id:g.id, block_id:block.id};
    }
    return null;
  }
  function markPhotoPoint() {
    const target = currentRecordingBlock();
    if (!target || !online || blocked() || !status.enabled) return false;
    if (pointMode() && !cameraReady()) { note(cameraHint(), true); return true; }
    if (pointMode()) note('正在记录点位并发送快门，请保持姿势');
    mutate('/api/blocks/photo-points', {...target, point_mode:pointMode()}, payload => pointMode()
      ? '六轴点位已记录，快门指令已完成，请查看相机'
      : `已记录拍照点 ${payload.recording_photo_points?.length || 0}`);
    return true;
  }
  $('#camera-connect').onclick = () => {
    if (!editable() || status.camera?.busy) return;
    if (status.camera?.requires_review && !confirm('上次拍照结果未知。请检查相机照片，确认无需补拍后重新连接。继续确认并连接？')) return;
    mutate('/api/camera/connect',{},'相机连接请求已提交，请查看相机状态');
  };
  $('#camera-disconnect').onclick = () => { if (editable() && !status.camera?.busy) mutate('/api/camera/disconnect',{},'相机断开请求已提交'); };
  $('#camera-save-settings').onclick = () => {
    if (!editable() || status.camera?.busy) return;
    const value = Number($('#camera-post-delay').value);
    if ($('#camera-post-delay').value === '' || !Number.isFinite(value) || value < 0 || value > 10) { note('拍照后停留请填写 0–10 秒',true); return; }
    mutate('/api/camera/settings',{post_delay_s:value},'拍照后停留设置已保存');
  };
  let consumedSpace = false;
  document.addEventListener('keydown', event => {
    if (event.code !== 'Space' && event.key !== ' ') return;
    if (event.repeat) { if (consumedSpace) event.preventDefault(); return; }
    if (event.altKey || event.ctrlKey || event.metaKey || event.shiftKey || event.isComposing) return;
    const target = event.target;
    if (target?.isContentEditable || target?.closest?.('input,textarea,select,[contenteditable]:not([contenteditable="false"]),[role="textbox"]')) return;
    if (markPhotoPoint()) { event.preventDefault(); consumedSpace = true; return; }
    if (status.camera_profile !== 'unloaded_50g' || !online || blocked() || !status.connected ||
        !status.enabled || status.mode !== 'running' || Date.now() - lastReceived >= 500 ||
        !Array.isArray(status.q_deg) || status.q_deg.length !== 6 || !status.q_deg.every(Number.isFinite)) return;
    angleTargets = status.q_deg.slice();
    referenceCaptured = true;
    $('#live-angle-title').textContent = '六轴实时角度 · 空格记录点';
    renderAngleTargets();
    renderLiveAngles();
    note('已记录六轴参考角度');
    event.preventDefault();
    consumedSpace = true;
  });
  document.addEventListener('keyup', event => {
    if ((event.code === 'Space' || event.key === ' ') && consumedSpace) { event.preventDefault(); consumedSpace = false; }
  });
  function ids(blockId) { return {group_id:group().id, ...(blockId == null ? {} : {block_id:group().blocks.find(block=>String(block.id)===String(blockId)).id})}; }
  $('#studio-start').onclick = () => { if (confirm('启动后机械臂会从当前位置自动返回新 Home，然后固定保持。请确认整段回程畅通、人员离开运动范围。')) mutate('/api/start',{camera_profile:status.camera_profile || 'heavy',joint_percent:status.joint_percent || [80,80,80,80,80,80]},'启动请求已提交，请等待设备确认'); };
  $('#studio-home').onclick = () => {
    if (confirm('返回 Home 零位并保持使能？请确认回程畅通。')) urgent('/api/home-hold','已请求回零点，完成后保持使能');
  };
  $('#studio-return').onclick = () => { if (confirm('中止当前动作，回 Home 后失能？请确认回程畅通。')) urgent('/api/home-disable','已请求返回 Home，完成后将失能'); };
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
  $('#studio-play-group').onclick = () => {
    if (status.group_playback || ['replaying','replay_starting','group_waiting'].includes(status.mode)) { mutate('/api/stop',{},'已请求停止播放并保持使能'); return; }
    const g = group(); if (!g || !ready()) return;
    const saved = g.blocks.filter(block=>block.trajectory), empty = g.blocks.filter(block=>!block.trajectory);
    if (!saved.length) return;
    if (photoCount(saved) && !cameraReady()) { note(cameraHint(),true); return; }
    const order = g.blocks.flatMap((block,index)=>block.trajectory ? [`${index+1}. ${block.name}${recordingError(block.slot) ? '（本次录制失败，使用原有已保存轨迹）' : ''}`] : []).join(' → ');
    const skip = empty.length ? `\n将跳过 ${empty.length} 个空动作：${empty.map(block=>block.name).join('、')}。` : '\n组内全部动作均有已保存轨迹，不跳过动作。';
    if (confirm(`按顺序播放「${g.name}」的 ${saved.length} 个已保存动作？\n执行顺序：${order}${skip}\n回放速度：${playbackSpeed * 100}%。本组将拍摄 ${photoCount(saved)} 张照片；相机状态：${cameraReady() ? '已就绪' : '未就绪（本组无拍照点，不影响播放）'}。\n动作间隔 ${g.interval_s} 秒，完成后保持使能。`)) mutate('/api/groups/play',{...ids(),speed:playbackSpeed},'整组播放已提交');
  };
  $('#studio-blocks').onchange = event => { const input=event.target.closest('[data-name]'); if(input && editable()){const name=input.value.trim();if(!name){rowsKey='';render();return;}mutate('/api/blocks/update',{...ids(input.dataset.name),name},'动作名称已保存');} };
  $('#studio-blocks').onclick = event => {
    const mark = event.target.closest('[data-mark-point]'), deletePoint = event.target.closest('[data-delete-point]');
    if (mark) { markPhotoPoint(); return; }
    if (deletePoint) {
      const block = group()?.blocks.find(item=>String(item.id)===deletePoint.dataset.pointBlock);
      if (block && online && !blocked() && (isRecording(block) || editable())) mutate('/api/blocks/photo-points/delete',{...ids(block.id),point_id:deletePoint.dataset.deletePoint},'拍照点已删除');
      return;
    }
    const remove=event.target.closest('[data-remove]'), record=event.target.closest('[data-record]'), play=event.target.closest('[data-play]');
    if(remove && editable()){if(confirm('删除这个动作块？此操作不可撤销。'))mutate('/api/blocks/delete',ids(remove.dataset.remove),'动作块已删除');return;}
    if(record){const block=group()?.blocks.find(item=>String(item.id)===record.dataset.record);if(!block)return;if(isRecording(block)){mutate('/api/blocks/stop-recording',ids(block.id),'动作录制已保存');return;}if(!ready())return;if(pointMode() && !cameraReady()){note('请先连接蓝牙相机，再开始录制和打点。'+cameraHint(),true);return;}if(!confirm(block.trajectory ? '请托稳机械臂。重新录制会切换手动摆位，并替换此动作已有轨迹，继续？' : '请先托稳机械臂。开始录制将切换手动摆位模式，继续？'))return;mutate('/api/blocks/record',ids(block.id),'正在录制，请手动拖动机械臂，完成后点击停止录制');return;}
    if (play && ready()) {
      const block = group()?.blocks.find(item=>String(item.id)===play.dataset.play);
      if (!block?.trajectory) return;
      if (pointMode()) {
        if (block.trajectory.camera_profile !== 'heavy' || !photoPoints(block.trajectory).length) {
          note('请在 950 克配置下录制并按空格打点', true); return;
        }
        if (!cameraReady()) { note(cameraHint(), true); return; }
        if (confirm('开始按点回放？六轴同时在 ±0.5° 内持续 2 秒才拍照；最后一张拍完保持 2 秒，再回 Home 并保持使能。请确认往返路线畅通、人员离开运动范围。')) {
          mutate('/api/blocks/play',{...ids(block.id),speed:playbackSpeed,point_mode:true},'按点回放已提交');
        }
        return;
      }
      if (precisionReplay(block.trajectory)) {
        if (confirm('开始无快门精度回放？机械臂会走到第一个打点，尝试微调并保持。请确认路径畅通、人员离开运动范围。')) {
          mutate('/api/blocks/play',{...ids(block.id),speed:playbackSpeed},'无快门精度回放已提交');
        }
        return;
      }
      if (photoPoints(block.trajectory).length && !cameraReady()) { note(cameraHint(),true); return; }
      const previous = recordingError(block.slot) ? '\n注意：最近一次录制未保存，将播放原有已保存轨迹。' : '';
      if (confirm(`播放「${block.name}」？机械臂将先移动到记录起点，再执行轨迹。\n回放速度：${playbackSpeed * 100}%。将拍摄 ${photoPoints(block.trajectory).length} 张照片；相机状态：${cameraReady() ? '已就绪' : '未就绪（此动作无拍照点，不影响播放）'}。${previous}`)) mutate('/api/blocks/play',{...ids(play.dataset.play),speed:playbackSpeed},'动作播放已提交');
    }
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
