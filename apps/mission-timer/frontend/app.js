/* Mission Timer & Scoreboard - frontend only MVP
 * Agreed defaults: Dr mode1 (All return else 0.5x), Pi half-up, 2 decimals, Ca manual, Cm=1.0, All-return policy
 */
(function () {
  const $ = (sel) => document.querySelector(sel);
  const $$ = (sel) => Array.from(document.querySelectorAll(sel));

  const state = {
    version: 1,
    displayDecimals: 2,
    piRounding: 'half-up', // 'half-up' | 'floor' | 'ceil'
    mission: {
      durationSec: 900, // default 15min
      startedAtMs: null,
      paused: true,
      elapsedOffsetMs: 0, // accumulated when paused/resumed
      caApply: false,
      caValue: 1.0,
      cmValue: 1.0,
      dcValue: 0.0,
      history: [],
    },
    robots: [
      { id: id(), name: 'Robot A', returned: true },
      { id: id(), name: 'Robot B', returned: true },
    ],
    tasks: [],
    bridge: {
      url: (location.protocol === 'https:' ? 'wss://localhost:8443/ws' : 'ws://localhost:8000/ws'),
      connected: false,
      ws: null,
      stats: { qr: 0, robot_updates: 0 },
    }
  };
  // expose state/render hooks for bridge helpers
  window.__app_state = state;
  window.__app_render = () => render();

  // Utilities
  function id() { return Math.random().toString(36).slice(2, 9); }
  function clamp(x, lo, hi) { return Math.max(lo, Math.min(hi, x)); }
  function fmtTime(sec) {
    sec = Math.max(0, Math.floor(sec));
    const m = Math.floor(sec / 60).toString().padStart(2, '0');
    const s = (sec % 60).toString().padStart(2, '0');
    return `${m}:${s}`;
  }
  function round(value, decimals, mode) {
    const f = Math.pow(10, decimals);
    if (mode === 'floor') return Math.floor(value * f) / f;
    if (mode === 'ceil') return Math.ceil(value * f) / f;
    // half-up
    return Math.round(value * f) / f;
  }
  function save() { localStorage.setItem('mission-timer-state', JSON.stringify(state)); }
  function load() {
    try {
      const s = localStorage.getItem('mission-timer-state');
      if (!s) return;
      const data = JSON.parse(s);
      // shallow assign with guards
      Object.assign(state, {
        displayDecimals: data.displayDecimals ?? 2,
        piRounding: data.piRounding ?? 'half-up',
      });
      Object.assign(state.mission, {
        durationSec: data.mission?.durationSec ?? 900,
        startedAtMs: null, // do not resume automatically
        paused: true,
        elapsedOffsetMs: 0,
        caApply: !!data.mission?.caApply,
        caValue: data.mission?.caValue ?? 1.0,
        cmValue: data.mission?.cmValue ?? 1.0,
        dcValue: data.mission?.dcValue ?? 0.0,
        history: Array.isArray(data.mission?.history) ? data.mission.history : [],
      });
      state.robots = Array.isArray(data.robots) ? data.robots.map(r => ({ id: id(), name: r.name, returned: !!r.returned })) : state.robots;
      state.tasks = Array.isArray(data.tasks) ? data.tasks.map(t => ({
        id: id(),
        enabled: t.enabled ?? true,
        category: t.category ?? 'M',
        typeK: Number(t.typeK ?? 1),
        detailK: Number(t.detailK ?? 1),
        Ce: Number(t.Ce ?? 1),
        Cr: Number(t.Cr ?? 1),
        Dt: Number(t.Dt ?? 0),
        note: String(t.note ?? ''),
      })) : [];
    } catch (e) {
      console.warn('load state failed', e);
    }
  }

  // Timer
  function now() { return performance.now(); }
  function elapsedMs() {
    if (state.mission.paused || state.mission.startedAtMs == null) return state.mission.elapsedOffsetMs;
    return state.mission.elapsedOffsetMs + (now() - state.mission.startedAtMs);
  }
  function remainingSec() {
    return Math.max(0, state.mission.durationSec - Math.floor(elapsedMs() / 1000));
  }
  function start() {
    if (!state.mission.paused) return;
    state.mission.startedAtMs = now();
    state.mission.paused = false;
    save();
  }
  function pause() {
    if (state.mission.paused) return;
    state.mission.elapsedOffsetMs = elapsedMs();
    state.mission.startedAtMs = null;
    state.mission.paused = true;
    save();
  }
  function resetTimer() {
    state.mission.startedAtMs = null;
    state.mission.elapsedOffsetMs = 0;
    state.mission.paused = true;
    save();
  }
  function applyPreset(sec) {
    state.mission.durationSec = sec;
    resetTimer();
    save();
  }

  // Scoring
  function taskPt(t) { return 10 * Number(t.typeK) * Number(t.detailK); }
  function taskScore(t) {
    if (!t.enabled) return 0;
    return taskPt(t) * Number(t.Cr) * Number(t.Ce) - Number(t.Dt);
  }
  function tasksSum() { return state.tasks.reduce((s, t) => s + taskScore(t), 0); }
  function caFactor() { return state.mission.caApply ? Number(state.mission.caValue) : 1.0; }
  function cmFactor() { return Number(state.mission.cmValue); }
  function dcValue() { return Number(state.mission.dcValue); }
  function allReturned() { return state.robots.length === 0 || state.robots.every(r => !!r.returned); }
  function piPoints() {
    const seconds = remainingSec();
    const raw = seconds * (2 / 60);
    return round(raw, state.displayDecimals, state.piRounding);
  }
  function currentScoreNoPi() {
    const base = tasksSum() * caFactor() * cmFactor();
    // Apply Dr mode1 at display stage? Current (no Pi) shows Dr effect too
    const withDc = base - dcValue();
    const withDr = allReturned() ? withDc : withDc * 0.5;
    return withDr;
  }
  function ifFinishNowScore() {
    const base = tasksSum() * caFactor() * cmFactor() + piPoints();
    const withDc = base - dcValue();
    const withDr = allReturned() ? withDc : withDc * 0.5;
    return withDr;
  }

  // UI binding
  function render() {
    // time
    $('#remaining').textContent = fmtTime(remainingSec());
    $('#elapsed').textContent = fmtTime(state.mission.durationSec - remainingSec());
    $('#startPause').textContent = state.mission.paused ? 'Start' : 'Pause';
    // scores
    const dec = state.displayDecimals;
    $('#tasksSum').textContent = round(tasksSum(), dec, 'half-up').toFixed(dec);
    $('#currentScore').textContent = round(currentScoreNoPi(), dec, 'half-up').toFixed(dec);
    $('#finishNowScore').textContent = round(ifFinishNowScore(), dec, 'half-up').toFixed(dec);
    // Dr
    const dr = $('#drStatus');
    if (allReturned()) {
      dr.textContent = 'All returned';
      dr.className = 'score risk ok';
    } else {
      dr.textContent = 'Penalty: 0.5x applied';
      dr.className = 'score risk bad';
    }
    // factors
    $('#caApply').checked = !!state.mission.caApply;
    $('#caValue').value = String(state.mission.caValue);
    $('#cmValue').value = String(state.mission.cmValue);
    $('#dcValue').value = String(state.mission.dcValue);
    // robots
    const list = $('#robotsList');
    list.innerHTML = '';
    state.robots.forEach((r) => {
      const div = document.createElement('div');
      div.className = 'robot-chip';
      div.innerHTML = `
        <input type="checkbox" ${r.returned ? 'checked' : ''} data-id="${r.id}" />
        <span>${r.name}</span>
        <button data-del="${r.id}">×</button>
      `;
      list.appendChild(div);
    });
    list.querySelectorAll('input[type="checkbox"]').forEach(cb => {
      cb.addEventListener('change', (e) => {
        const id = e.target.getAttribute('data-id');
        const rob = state.robots.find(r => r.id === id);
        if (rob) { rob.returned = e.target.checked; changed(); }
      });
    });
    list.querySelectorAll('button[data-del]').forEach(btn => {
      btn.addEventListener('click', () => {
        const idv = btn.getAttribute('data-del');
        state.robots = state.robots.filter(r => r.id !== idv);
        changed();
      });
    });
    // tasks
    const tbody = $('#tasksBody');
    tbody.innerHTML = '';
    state.tasks.forEach((t) => {
      const tr = document.createElement('tr');
      tr.innerHTML = `
        <td><input type="checkbox" data-field="enabled" ${t.enabled ? 'checked' : ''} data-id="${t.id}" /></td>
        <td><input type="text" data-field="category" value="${t.category}" data-id="${t.id}" size="2"/></td>
        <td><input type="number" step="0.1" min="0" data-field="typeK" value="${t.typeK}" data-id="${t.id}"/></td>
        <td><input type="number" step="0.1" min="0" data-field="detailK" value="${t.detailK}" data-id="${t.id}"/></td>
        <td><input type="number" step="0.1" min="0" data-field="Ce" value="${t.Ce}" data-id="${t.id}"/></td>
        <td><input type="number" step="0.1" min="0" data-field="Cr" value="${t.Cr}" data-id="${t.id}"/></td>
        <td><input type="number" step="0.1" min="0" data-field="Dt" value="${t.Dt}" data-id="${t.id}"/></td>
        <td><input type="text" data-field="note" value="${t.note}" data-id="${t.id}"/></td>
        <td><button data-remove="${t.id}">Del</button></td>
      `;
      tbody.appendChild(tr);
    });
    tbody.querySelectorAll('input').forEach(inp => {
      inp.addEventListener('change', () => {
        const id = inp.getAttribute('data-id');
        const field = inp.getAttribute('data-field');
        const row = state.tasks.find(x => x.id === id);
        if (!row) return;
        if (field === 'enabled') row.enabled = inp.checked;
        else if (field === 'category') row.category = inp.value.toUpperCase().slice(0, 1);
        else if (['typeK','detailK','Ce','Cr','Dt'].includes(field)) row[field] = Number(inp.value);
        else if (field === 'note') row.note = inp.value;
        changed();
      });
    });
    tbody.querySelectorAll('button[data-remove]').forEach(btn => {
      btn.addEventListener('click', () => {
        const idv = btn.getAttribute('data-remove');
        state.tasks = state.tasks.filter(x => x.id !== idv);
        changed();
      });
    });
    // bridge
    $('#bridgeUrl').value = state.bridge.url;
    $('#bridgeStatus').textContent = state.bridge.connected ? 'connected' : 'disconnected';
    $('#bridgeStats').textContent = state.bridge.connected ? ` | qr: ${state.bridge.stats.qr}, robot updates: ${state.bridge.stats.robot_updates}` : '';
  }

  function changed() { save(); render(); }

  // Event wiring
  function wire() {
    // presets
    $$('.preset-group button[data-preset]').forEach(btn => {
      btn.addEventListener('click', () => { applyPreset(Number(btn.dataset.preset)); render(); });
    });
    $('#applyCustom').addEventListener('click', () => {
      const v = Number($('#customDuration').value);
      if (isFinite(v) && v > 0) { applyPreset(Math.floor(v)); render(); }
    });
    // controls
    $('#startPause').addEventListener('click', () => { state.mission.paused ? start() : pause(); render(); });
    $('#reset').addEventListener('click', () => { if (confirm('Reset timer?')) { resetTimer(); render(); } });
    $('#retry').addEventListener('click', () => {
      if (!confirm('Retry: reset current mission score to 0?')) return;
      // Save snapshot
      state.mission.history.push({
        timestamp: new Date().toISOString(),
        tasks: JSON.parse(JSON.stringify(state.tasks)),
        robots: JSON.parse(JSON.stringify(state.robots)),
        scoreBeforeReset: currentScoreNoPi(),
      });
      // Clear scores by disabling all tasks (keep rows)
      state.tasks.forEach(t => { t.enabled = false; });
      changed();
    });
    // settings
    $('#settingsBtn').addEventListener('click', () => $('#settingsDialog').showModal());
    $('#saveSettings').addEventListener('click', (e) => {
      e.preventDefault();
      state.displayDecimals = clamp(Number($('#displayDecimals').value) || 2, 0, 3);
      state.piRounding = $('#piRounding').value;
      save();
      $('#settingsDialog').close();
      render();
    });
    // factors
    $('#caApply').addEventListener('change', (e) => { state.mission.caApply = e.target.checked; changed(); });
    $('#caValue').addEventListener('change', (e) => { state.mission.caValue = clamp(Number(e.target.value)||1,1,3); changed(); });
    $('#cmValue').addEventListener('change', (e) => { state.mission.cmValue = clamp(Number(e.target.value)||1,1,1.2); changed(); });
    $('#dcValue').addEventListener('change', (e) => { state.mission.dcValue = Math.max(0, Number(e.target.value)||0); changed(); });
    // robots
    $('#addRobot').addEventListener('click', () => {
      const name = ($('#newRobotName').value || '').trim();
      if (!name) return;
      state.robots.push({ id: id(), name, returned: false });
      $('#newRobotName').value = '';
      changed();
    });
    // tasks
    $('#addTask').addEventListener('click', addTask);
    document.addEventListener('keydown', (e) => {
      if (e.key.toLowerCase() === 'n') { addTask(); }
      if (e.key === ' ') { e.preventDefault(); state.mission.paused ? start() : pause(); render(); }
      if (e.key.toLowerCase() === 'r') { $('#retry').click(); }
    });

    // bridge controls
    $('#bridgeConnect').addEventListener('click', () => bridgeConnect());
    $('#bridgeDisconnect').addEventListener('click', () => bridgeDisconnect());
    $('#bridgeUrl').addEventListener('change', (e) => { state.bridge.url = e.target.value.trim(); save(); });
  }

  function addTask() {
    state.tasks.push({
      id: id(), enabled: true, category: 'M', typeK: 1.0, detailK: 1.0, Ce: 1.0, Cr: 1.0, Dt: 0.0, note: ''
    });
    changed();
  }

  // Ticker
  function tick() {
    if (!state.mission.paused && state.mission.startedAtMs != null) {
      render();
      if (remainingSec() <= 0) { pause(); }
    }
    requestAnimationFrame(tick);
  }

  // Init
  load();
  wire();
  render();
  requestAnimationFrame(tick);
})();

// Bridge logic
function bridgeConnect() {
  const url = document.querySelector('#bridgeUrl').value.trim() || 'ws://localhost:8000/ws';
  const st = window.stateBridge = (window.stateBridge || {});
  if (st.ws) { try { st.ws.close(); } catch {} }
  const ws = new WebSocket(url);
  st.ws = ws;
  const appState = getAppState();
  appState.bridge.connected = false;
  appState.bridge.stats = { qr: 0, robot_updates: 0 };
  renderApp();

  ws.onopen = () => {
    appState.bridge.connected = true;
    renderApp();
  };
  ws.onclose = () => {
    appState.bridge.connected = false;
    renderApp();
  };
  ws.onerror = () => {
    appState.bridge.connected = false;
    renderApp();
  };
  ws.onmessage = (ev) => {
    try {
      const msg = JSON.parse(ev.data);
      handleBridgeEvent(msg);
    } catch {}
  };
}

function bridgeDisconnect() {
  const st = window.stateBridge;
  if (st && st.ws) { try { st.ws.close(); } catch {} }
}

function handleBridgeEvent(evt) {
  const appState = getAppState();
  switch (evt.type) {
    case 'snapshot': {
      // merge robots by name
      const names = new Set(appState.robots.map(r => r.name));
      evt.robots.forEach(r => {
        if (!names.has(r.name)) {
          appState.robots.push({ id: Math.random().toString(36).slice(2,9), name: r.name, returned: !!r.returned });
        } else {
          const rr = appState.robots.find(x => x.name === r.name);
          if (rr) rr.returned = !!r.returned;
        }
      });
      appState.bridge.connected = true;
      renderApp();
      break;
    }
    case 'robot_returned': {
      const rr = appState.robots.find(x => x.name === evt.name);
      if (rr) rr.returned = !!evt.returned;
      appState.bridge.stats.robot_updates++;
      renderApp();
      break;
    }
    case 'qr': {
      appState.bridge.stats.qr++;
      renderApp();
      break;
    }
    default:
      break;
  }
}

function getAppState() {
  // Access the closure state safely by re-reading from localStorage isn't ideal.
  // Instead, we expose helpers bound in closure via window hooks at load time.
  return window.__app_state;
}

function renderApp() {
  const fn = window.__app_render;
  if (typeof fn === 'function') fn();
}
