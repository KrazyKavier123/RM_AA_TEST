'use strict';
const get = id => document.getElementById(id);
const number = (v, digits = 1) => typeof v === 'number' && Number.isFinite(v) ? v.toFixed(digits) : '—';
const vector = (v, digits) => Array.isArray(v) ? v.map(x => number(x, digits)).join(', ') : '—';
const degrees = radians => typeof radians === 'number' ? number(radians * 180 / Math.PI, 2) : '—';
function row(values) {
  const tr = document.createElement('tr');
  for (const value of values) {
    const td = document.createElement('td'); td.textContent = String(value); tr.appendChild(td);
  }
  return tr;
}
const imageUrls = {};
async function refreshImage(id, route) {
  try {
    const response = await fetch(route, {cache: 'no-store', signal: AbortSignal.timeout(2000)});
    if (!response.ok) return;
    const url = URL.createObjectURL(await response.blob());
    get(id).src = url;
    if (imageUrls[id]) URL.revokeObjectURL(imageUrls[id]);
    imageUrls[id] = url;
  } catch (_) { /* Status polling reports connection loss. */ }
}
async function refresh() {
  try {
    const response = await fetch('/api/status', {cache: 'no-store', signal: AbortSignal.timeout(2000)});
    if (!response.ok) throw new Error('HTTP error');
    const s = await response.json();
    get('connection').textContent = s.online ? '数据在线' : '数据中断 / 等待';
    get('connection').className = 'badge' + (s.online ? ' online' : '');
    get('capture').textContent = number(s.capture_fps);
    get('processing').textContent = number(s.processing_fps);
    get('latency').textContent = number(s.processing_ms, 2);
    get('age').textContent = number(s.frame_age_ms);
    get('dropped').textContent = s.dropped ?? '—';
    get('resets').textContent = s.tracker_resets ?? '—';
    get('source').textContent = (s.mode ?? '等待来源') + ' · 帧 ' + (s.sequence ?? '—');
    get('notice').textContent = !s.online ? '视觉数据已停止更新，表格与图像可能是最后一帧。' :
      s.mode === 'web_fixture' ? '页面演示数据：未运行 C++ 视觉算法，性能指标和 PnP 数据留空。' :
      s.synthetic_calibration ? '合成场景：使用虚拟相机参数；真实相机必须单独标定。' :
      !s.calibration_valid ? '相机尚未标定，或内参与图像分辨率/坐标系不符；PnP 输出已禁用。' :
      '相机观测坐标：X 向右、Y 向下、Z 向前；yaw 向右为正，pitch 向上为正。';
    const detections = Array.isArray(s.detections) ? s.detections : [];
    get('count').textContent = detections.length + ' 个候选';
    get('detections').replaceChildren(...detections.map(d => row([
      d.index, d.label + ' / ' + (d.size === 'large' ? '大板' : '小板'),
      number(d.confidence * 100) + '%', d.accepted ? '通过' : '待确认',
      vector(d.pose?.position_m, 3), degrees(d.pose?.bearing_yaw_rad),
      degrees(d.pose?.bearing_pitch_rad), number(d.pose?.reprojection_rmse_px, 2),
    ])));
    get('empty').hidden = detections.length > 0;
    const tracks = Array.isArray(s.image_tracks) ? s.image_tracks : [];
    get('tracks').replaceChildren(...tracks.map(t => row([
      t.id, t.label || '未知', t.observed ? '本帧匹配' : '短时缺测',
      vector(t.position_px, 1), vector(t.velocity_px_s, 1),
    ])));
    if (s.online) await Promise.all([refreshImage('camera', '/frame.jpg'), refreshImage('mask', '/mask.jpg')]);
  } catch (_) {
    get('connection').textContent = '连接已断开'; get('connection').className = 'badge';
    get('notice').textContent = '无法连接 Web 服务，保留最后一帧供检查。';
  } finally { setTimeout(refresh, 100); }
}
refresh();
