const el = (id) => document.getElementById(id);

function number(value, digits = 1, suffix = '') {
  return Number.isFinite(value) ? `${value.toFixed(digits)}${suffix}` : '—';
}
function degrees(value) { return number(value * 180 / Math.PI, 1, '°'); }
function setBadge(node, text, good) {
  node.textContent = text;
  node.className = `badge ${good ? 'good' : 'offline'}`;
}
function metric(label, value) {
  return `<div class="metric"><span>${label}</span><strong>${value ?? '—'}</strong></div>`;
}

function renderCameras(cameras) {
  const container = el('cameras');
  const entries = Object.entries(cameras || {});
  if (!entries.length) {
    container.innerHTML = '<div class="empty">未配置摄像头。请在采集配置中增加 cameras。</div>';
    el('camera-summary').textContent = '未配置摄像头';
    return;
  }
  el('camera-summary').textContent = `${entries.length} 路摄像头`;
  for (const [name, camera] of entries) {
    let card = document.getElementById(`camera-${name}`);
    if (!card) {
      card = document.createElement('figure');
      card.id = `camera-${name}`;
      card.className = 'camera';
      card.innerHTML = `<img alt="${name} 实时画面"><figcaption><strong>${name}</strong><span></span></figcaption>`;
      container.appendChild(card);
    }
    const caption = card.querySelector('span');
    caption.textContent = camera.healthy ? `${camera.width}×${camera.height} · ${number(camera.age_ms, 0, ' ms')}` : (camera.last_error || '无画面');
    card.classList.toggle('camera-error', !camera.healthy);
    if (camera.healthy) {
      const image = card.querySelector('img');
      if (!image.dataset.streamStarted) {
        image.src = `/stream/${encodeURIComponent(name)}?t=${Date.now()}`;
        image.dataset.streamStarted = 'true';
      }
    }
  }
}

function render(payload) {
  const collector = payload.collector || {};
  setBadge(el('collector-status'), collector.connected ? `采集进程在线 · ${number(collector.age_ms, 0, ' ms')}` : '采集进程未连接', collector.connected);
  const collection = payload.collection || {};
  const state = collection.state || 'IDLE';
  setBadge(el('state'), state, state === 'TELEOP' || state === 'RECORDING');
  el('collection-metrics').innerHTML = [
    metric('Episode', collection.episode_index), metric('本段帧数', collection.frame_count),
    metric('本段时长', number(collection.elapsed_s, 1, ' s')), metric('实际 FPS', number(collection.actual_fps, 1)),
    metric('采样目标', number(collection.target_fps, 0, ' FPS')), metric('写入 P95', number(collection.add_frame_p95_ms, 1, ' ms')),
    metric('数据目录', collection.dataset_root || '尚未创建'), metric('磁盘可用', collection.free_disk_gb == null ? '—' : number(collection.free_disk_gb, 1, ' GB')),
  ].join('');

  renderCameras(payload.cameras);
  const robot = payload.robot || {};
  const healthy = robot.connection === 'connected' && !robot.error;
  const rightStatus = robot.right_status || {};
  const robotLabel = healthy
    ? `D3 已连接 · 伺服${rightStatus.servo === 1 ? '已使能' : '未知'} · ${rightStatus.fsm_state || '模式未知'}`
    : (robot.error || (robot.connection === 'dry_run' ? 'DRY RUN' : 'D3 未连接'));
  setBadge(el('robot-health'), robotLabel, healthy);
  const exo = payload.exoskeleton || {};
  const rows = Array.from({length: 7}, (_, i) => {
    const action = (exo.right_action_rad || [])[i];
    const command = (robot.command_joint_rad || [])[i];
    const feedback = (robot.feedback_joint_rad || [])[i];
    const error = Number.isFinite(command) && Number.isFinite(feedback) ? Math.abs(command-feedback) * 180 / Math.PI : NaN;
    const margin = (robot.limit_margin_rad || [])[i];
    return `<tr><td>J${i}</td><td>${degrees(action)}</td><td>${degrees(command)}</td><td>${degrees(feedback)}</td><td class="${error > 5 ? 'warn' : ''}">${number(error, 1, '°')}</td><td>${degrees(margin)}</td></tr>`;
  });
  el('joints').innerHTML = rows.join('');
  const events = payload.events || [];
  el('events').innerHTML = events.length ? events.slice(0, 20).map((event) => `<li><time>${event.time || '—'}</time>${event.message || event}</li>`).join('') : '<li>暂无事件</li>';
}

function connect() {
  const ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws/status`);
  ws.onmessage = (event) => render(JSON.parse(event.data));
  ws.onclose = () => setTimeout(connect, 1000);
}
connect();
