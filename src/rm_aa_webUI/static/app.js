'use strict';
const $ = id => document.getElementById(id);
const charts = new Map();
let paused = false, cursor = 0, epoch = '', activePage = 'monitor', isDemo = false;
let file = null, baseContent = '', changes = {}, dirty = false, editorMode = 'form';
const number = value => Number.isFinite(value) ? value.toLocaleString('en-US', {maximumFractionDigits: 4, useGrouping: false}) : '—';

function notice(text, error = false) {
  $('notice').hidden = false; $('notice').textContent = text; $('notice').className = error ? 'error' : '';
}
async function api(url, options = {}) {
  const response = await fetch(url, {...options, headers: {'Content-Type': 'application/json', ...options.headers}});
  const data = await response.json();
  if (!response.ok) throw new Error(typeof data.detail === 'string' ? data.detail : JSON.stringify(data.detail));
  return data;
}
function setDirty(value) {
  dirty = value; $('dirty').textContent = file ? (value ? '有未保存的修改' : '与文件一致') : '未选择文件';
  $('save').disabled = !file || !value;
}
function newChart(key) {
  const card = document.createElement('article'); card.className = 'chart-card'; card.dataset.metric = key;
  const head = document.createElement('div'); head.className = 'chart-head';
  const name = document.createElement('h2'); name.className = 'chart-name'; name.textContent = key;
  const actions = document.createElement('div'); actions.className = 'chart-actions';
  const csv = document.createElement('button'); csv.textContent = 'CSV'; csv.setAttribute('aria-label', `导出 ${key} CSV`);
  const collapse = document.createElement('button'); collapse.textContent = '−'; collapse.setAttribute('aria-label', `折叠 ${key}`);
  const value = document.createElement('div'); value.className = 'chart-value';
  const age = document.createElement('div'); age.className = 'chart-age';
  const canvas = document.createElement('canvas'); canvas.setAttribute('aria-label', `${key} 曲线`);
  actions.append(csv, collapse); head.append(name, actions); card.append(head, value, age, canvas); $('charts').append(card);
  const chart = {key, card, value, age, canvas, history: []}; charts.set(key, chart);
  csv.onclick = () => exportCSV([chart]);
  collapse.onclick = () => { const hidden = card.classList.toggle('collapsed'); collapse.textContent = hidden ? '+' : '−'; collapse.setAttribute('aria-label', `${hidden ? '展开' : '折叠'} ${key}`); draw(chart); };
  return chart;
}
function draw(chart) {
  if (chart.card.hidden || chart.card.classList.contains('collapsed')) return;
  const all = chart.history; if (!all.length) return;
  const seconds = Number($('timeWindow').value), end = all.at(-1).t;
  const points = seconds ? all.filter(p => p.t >= end - seconds * 1000) : all;
  const canvas = chart.canvas, width = canvas.clientWidth, height = canvas.clientHeight;
  if (!width) return;
  const scale = window.devicePixelRatio || 1;
  canvas.width = width * scale; canvas.height = height * scale;
  const ctx = canvas.getContext('2d'); ctx.scale(scale, scale);
  const start = seconds ? end - seconds * 1000 : points[0].t - 1;
  let low = Infinity, high = -Infinity;
  for (const point of points) { low = Math.min(low, point.v); high = Math.max(high, point.v); }
  const pad = Math.max((high - low) * 0.12, Math.abs(high) * 0.005, 0.001); low -= pad; high += pad;
  const left = 47, right = width - 7, top = 10, bottom = height - 22;
  ctx.font = '10px Consolas, monospace'; ctx.textBaseline = 'middle';
  for (let i = 0; i < 3; i++) {
    const y = top + (bottom - top) * i / 2;
    ctx.strokeStyle = '#2b343e'; ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(right, y); ctx.stroke();
    ctx.fillStyle = '#8493a3'; ctx.fillText(number(high - (high - low) * i / 2).slice(0, 8), 0, y);
  }
  ctx.fillStyle = '#8493a3'; ctx.textBaseline = 'bottom';
  ctx.fillText(new Date(start).toLocaleTimeString('zh-CN', {hour12: false}), left, height);
  ctx.textAlign = 'right'; ctx.fillText(new Date(end).toLocaleTimeString('zh-CN', {hour12: false}), right, height); ctx.textAlign = 'left';
  ctx.strokeStyle = '#79d7bd'; ctx.lineWidth = 1.5; ctx.beginPath();
  // Pixel buckets retain min/max spikes; CSV always uses the original retained samples.
  let bucket = -1, min = Infinity, max = -Infinity;
  const flush = () => { if (bucket < 0) return; for (const v of [min, max]) ctx.lineTo(left + bucket, bottom - (v - low) / (high - low) * (bottom - top)); };
  for (const point of points) {
    const x = Math.round((point.t - start) / Math.max(1, end - start) * (right - left));
    if (x !== bucket) { flush(); bucket = x; min = max = point.v; } else { min = Math.min(min, point.v); max = Math.max(max, point.v); }
  }
  flush(); ctx.stroke();
}
function render() {
  if (paused || activePage !== 'monitor') return;
  let visible = 0;
  for (const chart of charts.values()) {
    chart.card.hidden = !!$('sourceFilter').value && !chart.key.startsWith($('sourceFilter').value + '.');
    if (chart.card.hidden) continue;
    visible++;
    const last = chart.history.at(-1), age = Math.max(0, (Date.now() - last.t) / 1000);
    chart.value.textContent = number(last.v); chart.age.textContent = age < 2 ? '实时更新' : `${age.toFixed(0)} 秒前 · 最后一次数据`;
    chart.card.classList.toggle('stale', age >= 2); draw(chart);
  }
  $('metricCount').textContent = `${visible} 个指标`; $('empty').hidden = visible > 0;
}
function csvCell(text) { text = String(text); if (/^[=+@]/.test(text)) text = "'" + text; return '"' + text.replaceAll('"', '""') + '"'; }
function exportCSV(selected = [...charts.values()]) {
  const rows = ['metric,timestamp_ms,time_iso,value'];
  for (const chart of selected) for (const point of chart.history) rows.push(`${csvCell(chart.key)},${point.t},${new Date(point.t).toISOString()},${point.v}`);
  const url = URL.createObjectURL(new Blob(['\ufeff' + rows.join('\r\n')], {type: 'text/csv;charset=utf-8'}));
  const link = document.createElement('a'); link.href = url; link.download = `rm-aa-${Date.now()}.csv`; link.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
}
const imageTimes = {};
function connect() {
  const socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws?after=${cursor}`);
  socket.onopen = () => { $('connection').textContent = '服务已连接'; $('connection').className = 'status online'; };
  socket.onmessage = event => {
    const data = JSON.parse(event.data);
    if (epoch && epoch !== data.epoch) { charts.clear(); $('charts').replaceChildren(); cursor = 0; }
    epoch = data.epoch;
    for (const sample of data.samples) {
      if (sample.id <= cursor) continue;
      const chart = charts.get(sample.key) || newChart(sample.key);
      chart.history.push(sample); if (chart.history.length > 5000) chart.history.splice(0, chart.history.length - 5000);
    }
    cursor = data.cursor;
    $('sources').replaceChildren();
    for (const [source, updated] of Object.entries(data.sources)) {
      const badge = document.createElement('span'); const live = data.now - updated < 2000;
      badge.className = 'source-badge' + (live ? ' live' : ''); badge.textContent = `${source} · ${live ? '在线' : '无新数据'}`; $('sources').append(badge);
      if (![...$('sourceFilter').options].some(option => option.value === source)) $('sourceFilter').add(new Option(source, source));
    }
    $('rosState').textContent = `HTTP · WebSocket / ROS ${data.ros}`;
    if (!paused) for (const [kind, time] of Object.entries(data.images)) {
      if (imageTimes[kind] !== time) { $(kind === 'image' ? 'cameraImage' : 'maskImage').src = `/api/frames/${kind}?t=${time}`; imageTimes[kind] = time; }
    }
    render();
  };
  socket.onclose = () => { $('connection').textContent = '连接断开 · 重连中'; $('connection').className = 'status offline'; setTimeout(connect, 1500); };
}
function monitorNote() {
  $('monitorNote').textContent = (isDemo ? '演示数据 · 不代表真实视觉或云台性能。' : '') + (paused ? '显示已暂停，仍在接收并缓存数据。' : '每个指标保留最近 5000 个样本，CSV 导出保留的完整数据。');
}
$('pause').onclick = () => { paused = !paused; $('pause').textContent = paused ? '继续显示' : '暂停显示'; monitorNote(); render(); };
$('clear').onclick = () => { charts.clear(); $('charts').replaceChildren(); $('metricCount').textContent = '0 个指标'; $('empty').hidden = false; };
$('export').onclick = () => exportCSV();
$('timeWindow').onchange = render; $('sourceFilter').onchange = render;
window.addEventListener('resize', render);
setInterval(render, 1000);

function showFile(data) {
  file = data; baseContent = data.content; changes = {}; $('sourceEditor').value = data.content;
  $('fileInfo').textContent = data.path; setDirty(false); renderFields();
}
function renderFields() {
  $('fields').replaceChildren();
  for (const field of file.fields) {
    const row = document.createElement('label'); row.className = 'field-row'; row.dataset.search = field.label.toLowerCase();
    const label = document.createElement('span'); label.className = 'field-label';
    const parts = field.label.split(' / '), name = document.createElement('strong'); name.textContent = parts.pop();
    const path = document.createElement('span'); path.className = 'muted'; path.textContent = parts.join(' / '); label.append(name, path);
    const type = document.createElement('span'); type.className = 'field-type'; type.textContent = field.type;
    const input = document.createElement(field.type === 'bool' ? 'select' : 'input');
    input.setAttribute('aria-label', field.label); input.dataset.key = field.key;
    if (field.type === 'bool') { input.add(new Option('true', 'true')); input.add(new Option('false', 'false')); }
    else { input.type = ['int', 'float'].includes(field.type) ? 'number' : 'text'; if (input.type === 'number') input.step = field.type === 'int' ? '1' : 'any'; }
    input.value = field.value === null ? 'null' : String(field.value); input.disabled = field.type === 'null';
    input.oninput = () => {
      const value = field.type === 'bool' ? input.value === 'true' : ['int', 'float'].includes(field.type) ? (input.value === '' ? null : Number(input.value)) : input.value;
      if (value === field.value) delete changes[field.key]; else changes[field.key] = value;
      row.classList.toggle('changed', Object.hasOwn(changes, field.key)); setDirty(Object.keys(changes).length > 0 || baseContent !== file.content);
    };
    row.append(label, type, input); $('fields').append(row);
  }
  if (!file.fields.length) { const p = document.createElement('p'); p.className = 'placeholder muted'; p.textContent = '此文件没有可直接调整的常量，请使用源文件编辑。'; $('fields').append(p); }
  filterFields();
}
function filterFields() { const value = $('paramSearch').value.toLowerCase(); for (const row of $('fields').querySelectorAll('.field-row')) row.hidden = !row.dataset.search.includes(value); }
$('paramSearch').oninput = filterFields;
async function preview() {
  if (editorMode === 'form' && !$('fields').querySelector('input:invalid')) {
    return api('/api/config/preview', {method: 'POST', body: JSON.stringify({path: file.path, content: baseContent, changes})});
  }
  if (editorMode === 'form') throw new Error('请填写有效数值；整数参数不能包含小数。');
  return api('/api/config/preview', {method: 'POST', body: JSON.stringify({path: file.path, content: $('sourceEditor').value})});
}
async function switchEditor(mode) {
  if (mode === editorMode) return;
  try {
    if (file) {
      const data = await preview(); baseContent = data.content; file.fields = data.fields; changes = {};
      $('sourceEditor').value = baseContent; renderFields(); setDirty(baseContent !== file.content);
    }
    editorMode = mode; $('fields').hidden = mode !== 'form'; $('sourceEditor').hidden = mode !== 'source'; $('paramSearch').hidden = mode !== 'form';
    $('formTab').classList.toggle('selected', mode === 'form'); $('sourceTab').classList.toggle('selected', mode === 'source');
  } catch (error) { notice(error.message, true); }
}
$('formTab').onclick = () => switchEditor('form'); $('sourceTab').onclick = () => switchEditor('source');
$('sourceEditor').oninput = () => setDirty($('sourceEditor').value !== file?.content);
async function loadFile(path) {
  if (!path) return;
  try { showFile(await api(`/api/config?path=${encodeURIComponent(path)}`)); $('notice').hidden = true; }
  catch (error) { notice(error.message, true); }
}
$('configFile').onchange = async () => {
  if (dirty && !confirm('当前修改尚未保存，切换文件将放弃这些修改。')) { $('configFile').value = file.path; return; }
  await loadFile($('configFile').value);
};
$('reloadFile').onclick = () => { if (!dirty || confirm('重新加载会放弃未保存修改。')) loadFile($('configFile').value); };
$('save').onclick = async () => {
  $('save').disabled = true;
  try {
    const prepared = await preview();
    const saved = await api('/api/config', {method: 'PUT', body: JSON.stringify({path: file.path, revision: file.revision, content: prepared.content})});
    showFile(saved); notice(saved.message);
  } catch (error) { notice(error.message, true); $('save').disabled = false; }
};
document.querySelectorAll('[data-page]').forEach(button => button.onclick = () => {
  activePage = button.dataset.page;
  $('notice').hidden = true;
  document.querySelectorAll('[data-page]').forEach(tab => tab.classList.toggle('active', tab === button));
  $('monitorPage').hidden = activePage !== 'monitor'; $('configPage').hidden = activePage !== 'config'; render();
});
window.addEventListener('beforeunload', event => { if (dirty) { event.preventDefault(); event.returnValue = ''; } });
async function init() {
  try {
    const catalog = await api('/api/configs');
    for (const path of catalog.files) $('configFile').add(new Option(path, path));
    const status = await api('/api/status');
    isDemo = status.demo; monitorNote();
  } catch (error) { notice(error.message, true); }
  connect();
}
init();
