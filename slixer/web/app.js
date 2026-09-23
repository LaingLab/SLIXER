// Slixer in the browser: one socket carries the arm's state down and your instructions up, and each tab
// is a panel that draws from the same state.
//
// The mode decides what the model means. In Watch it mirrors the real arm; in Plan it is a virtual arm
// you can pose and rehearse on without sending anything; in Drive it moves the real arm, and where the
// real arm actually is shows as a blue ghost whenever it lags. A selected program step shows in amber.

import { ArmScene } from '/static/scene.js';
import { $, degrees, radians, remember, SHORT_LABELS, throttle } from '/static/util.js';
import { JointsPanel } from '/static/panels/joints.js';
import { ProgramPanel } from '/static/panels/program.js';
import { PartsPanel } from '/static/panels/parts.js';
import { CameraPanel } from '/static/panels/camera.js';
import { SetupPanel } from '/static/panels/setup.js';

const HINTS = {
  watch: 'watching the real arm · switch to Plan to pose the model',
  plan: 'drag the orange handle to pose the virtual arm · double-click a part to reach for it · the real arm stays where it is',
  drive: 'drag the orange handle to move the real arm · Esc stops',
};

class App {
  constructor() {
    this.scene = new ArmScene($('canvas-holder'));
    this.model = null;
    this.state = null;
    this.socket = null;
    this.mode = 'watch';
    this.panels = [];
    this.listeners = new Map();  // reply type -> handlers
  }

  async start() {
    this.model = await (await fetch('/api/model')).json();
    $('version').textContent = this.model.version || '';
    await this.scene.load(this.model);

    this.joints = new JointsPanel(this);
    this.program = new ProgramPanel(this);
    this.parts = new PartsPanel(this);
    this.camera = new CameraPanel(this);
    this.setup = new SetupPanel(this);
    this.panels = [this.joints, this.program, this.parts, this.camera, this.setup];
    await Promise.all(this.panels.map((p) => p.load?.()));

    this.wireHeader();
    this.wireTabs();
    this.wireViewport();
    this.wireKeys();
    this.connect();
    // A connection that has quietly died -- a network dropped mid-way, say -- can take a long time to report
    // itself closed. The server sends state twenty times a second, so three seconds of silence means it's
    // gone: say so now, and start again.
    setInterval(() => {
      if (this.socket?.readyState === WebSocket.OPEN && performance.now() - this.lastHeard > 3000) this.lost(this.socket);
    }, 500);
  }

  // ---- the link to the server -------------------------------------------------

  connect() {
    const url = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`;
    const socket = new WebSocket(url);
    this.socket = socket;
    this.lastHeard = performance.now();
    socket.onmessage = (event) => {
      this.lastHeard = performance.now();
      this.receive(JSON.parse(event.data));
    };
    socket.onopen = () => {
      this.lastHeard = performance.now();
      if (this.pendingStop) {
        // STOP was pressed while there was no connection: it goes before anything else.
        socket.send(JSON.stringify({ do: 'stop' }));
        this.pendingStop = false;
        this.say('connected -- and sent the STOP pressed while the connection was down');
      } else {
        this.say('connected');
      }
    };
    socket.onclose = () => this.lost(socket);
  }

  // The connection has gone, or stopped answering. Said plainly: this page can't reach the server, which
  // may still be driving the arm -- for another page, or running a program -- so nothing can be promised
  // about the arm from here.
  lost(socket) {
    if (socket !== this.socket) return;  // an old connection, already dealt with
    this.socket = null;
    socket.onclose = null;
    socket.onmessage = null;
    try { socket.close(); } catch { /* already gone */ }
    this.say('lost the server — reconnecting');
    this.showBanner('Lost the connection to Slixer, reconnecting. Until it is back this page can\u2019t send '
      + 'anything, STOP included: if the arm must stop now, switch off its power.', 'bad');
    setTimeout(() => this.connect(), 1500);
  }

  send(message) {
    if (this.socket?.readyState === WebSocket.OPEN) this.socket.send(JSON.stringify(message));
  }

  on(type, handler) {
    if (!this.listeners.has(type)) this.listeners.set(type, []);
    this.listeners.get(type).push(handler);
  }

  receive(message) {
    if (message.type === 'state') return this.onState(message);
    if (message.text) this.say(message.text);
    if (message.type === 'ik' && !message.reached) {
      this.say(`can't quite reach that: ${(message.error * 1000).toFixed(0)} mm short`);
    }
    for (const handler of this.listeners.get(message.type) || []) handler(message);
  }

  say(text) {
    $('status').textContent = text;
  }

  // ---- state from the server -------------------------------------------------

  onState(state) {
    this.state = state;
    if (state.mode !== this.mode) this.setModeLook(state.mode);

    this.scene.setPose(state.commanded);
    this.scene.setHandleVisible(state.movable);
    // The ghost is the real arm: shown in Drive when it lags, and in Plan so you can see where the
    // hardware is while the model is off being virtual.
    let ghost = null;
    if (state.real && state.mode !== 'watch') {
      const gap = Math.max(...state.real.map((v, i) => Math.abs(v - state.commanded[i])));
      if (state.mode === 'plan' || gap > radians(2)) ghost = state.real;
    }
    this.scene.setGhost(ghost);

    this.updatePills(state);
    this.updateBanner(state);

    const tip = state.tip_commanded.map((v) => (v * 1000).toFixed(0));
    $('tip-readout').textContent = `gripper tip  x ${tip[0]}  y ${tip[1]}  z ${tip[2]} mm`;
    $('pose-readout').textContent = this.model.motors.map((name, i) => {
      const value = i < 5 ? `${state.arm_pose[i].toFixed(0)}°` : `${state.arm_pose[5].toFixed(0)}%`;
      return `${SHORT_LABELS[name]} ${value}`;
    }).join('   ');

    for (const panel of this.panels) panel.onState?.(state);
  }

  updatePills(state) {
    const arm = $('pill-arm');
    let text;
    let tone;
    if (!state.connected) {
      [text, tone] = ['no arm on the network', 'idle'];
    } else if (state.fault) {
      [text, tone] = [`${state.fault} (${state.fault_joint})`, 'bad'];
    } else if (state.source === 'leader') {
      [text, tone] = ['only the leader is on Wi-Fi', 'warn'];
    } else if (state.mode === 'drive') {
      [text, tone] = ['driven by Slixer', 'good'];
    } else if (state.leader_steering) {
      [text, tone] = ['following the leader arm', 'good'];
    } else {
      [text, tone] = [state.arm_state, 'good'];
    }
    arm.textContent = text;
    arm.className = `pill ${tone}`;
    arm.title = state.address ? `arm at ${state.address}` : '';

    const camera = $('pill-camera');
    if (!state.camera.configured) {
      camera.textContent = 'no camera';
      camera.className = 'pill idle';
    } else if (state.camera.connected) {
      camera.textContent = 'camera';
      camera.className = 'pill good';
    } else {
      camera.textContent = 'camera not answering';
      camera.className = 'pill bad';
    }
    camera.title = state.camera.host || '';
    $('pill-rate').textContent = `${Math.round(state.rate)} Hz`;
  }

  updateBanner(state) {
    if (state.warning) return this.showBanner(state.warning, 'bad');
    if (this.bannerFromServer) this.showBanner(null);
  }

  showBanner(text, tone = 'bad') {
    const banner = $('banner');
    this.bannerFromServer = Boolean(text);
    banner.classList.toggle('hidden', !text);
    banner.className = text ? `banner ${tone}` : 'hidden';
    banner.textContent = text || '';
  }

  // ---- the mode ---------------------------------------------------------------

  setModeLook(mode) {
    this.mode = mode;
    document.body.className = `mode-${mode}`;
    for (const button of document.querySelectorAll('.modes button')) {
      button.classList.toggle('active', button.dataset.mode === mode);
    }
    $('mode-badge').textContent = { watch: 'WATCH', plan: 'PLAN · virtual arm', drive: 'DRIVE · real arm' }[mode];
    $('hint').textContent = HINTS[mode];
  }

  wireHeader() {
    for (const button of document.querySelectorAll('.modes button')) {
      button.addEventListener('click', () => {
        if (button.dataset.mode !== this.mode) this.send({ do: 'mode', mode: button.dataset.mode });
      });
    }
    $('estop').addEventListener('click', () => this.stop());
    this.setModeLook('watch');
  }

  stop() {
    // What the page itself still has on its way goes first -- the rest of a drag, a move the throttle is
    // holding back, a slider still under the pointer -- or it would follow the STOP to the server and set
    // the arm moving again straight after.
    this.scene.cancelDrag();
    this.scene.onDrag.cancel?.();
    this.joints.cancel();
    if (this.socket?.readyState === WebSocket.OPEN) {
      this.socket.send(JSON.stringify({ do: 'stop' }));
      return;
    }
    this.pendingStop = true;  // sent the moment the connection is back
    this.say('not connected: STOP will be sent as soon as the connection is back. '
      + 'If the arm must stop now, switch off its power');
  }

  // ---- tabs, viewport, keys --------------------------------------------------

  wireTabs() {
    const show = (name) => {
      document.querySelectorAll('.tabs button').forEach((b) => b.classList.toggle('active', b.dataset.tab === name));
      document.querySelectorAll('.tab').forEach((tab) => tab.classList.toggle('hidden', tab.id !== `tab-${name}`));
      this.activeTab = name;
      remember.set('tab', name);
      for (const panel of this.panels) panel.onShown?.(name);
      this.scene.resize();
    };
    for (const button of document.querySelectorAll('.tabs button')) {
      button.addEventListener('click', () => show(button.dataset.tab));
    }
    show(remember.get('tab', 'joints'));
  }

  wireViewport() {
    $('btn-home').addEventListener('click', () => this.scene.frame());
    $('btn-ghost').addEventListener('click', (event) => {
      this.scene.showGhost = !this.scene.showGhost;
      event.target.classList.toggle('on', this.scene.showGhost);
    });
    this.scene.onDrag = throttle((xyz) => this.send({ do: 'ik', xyz }), 40);
  }

  wireKeys() {
    document.addEventListener('keydown', (event) => {
      // Escape stops, whatever has the keyboard: a stop key that only works outside text boxes is a stop
      // key that fails the one time it's needed.
      if (event.key === 'Escape') {
        this.stop();
        return;
      }
      // Shortcuts are held back only while typing: sliders and tick-boxes aren't typing.
      const typing = event.target.matches('input:not([type=range]):not([type=checkbox]), select, textarea');
      if (typing || event.ctrlKey || event.metaKey || event.altKey) return;
      for (const panel of this.panels) {
        if (panel.onKey?.(event)) {
          event.preventDefault();
          return;
        }
      }
    });
    window.addEventListener('beforeunload', (event) => {
      if (this.program?.dirty) {
        event.preventDefault();
        event.returnValue = '';
      }
    });
  }

  // ---- the arm's numbers and the model's -------------------------------------
  //
  // Programs store the arm's own numbers. To draw one, it has to go through the current calibration,
  // exactly as the server does -- sign, offset, and the gripper's percentage spread over its angles.

  armToModel(arm) {
    const mapping = this.state?.mapping;
    if (!mapping) return null;
    const body = this.model.motors.slice(0, 5).map((name, i) => {
      const joint = mapping.joints[name];
      return radians(joint.sign * arm[i] + joint.offset_deg);
    });
    const { closed_deg: shut, open_deg: open } = mapping.gripper;
    return [...body, radians(shut + (Math.max(0, Math.min(100, arm[5])) / 100) * (open - shut))];
  }

  modelToDegrees(pose) {
    return pose.map((v) => degrees(v));
  }
}

const app = new App();
app.start();

// Reachable from the browser console: `slixer.state` is everything the arm last reported, and
// `slixer.scene` is the 3D scene -- handy when working out why something looks wrong.
window.slixer = app;
