// The camera beside the arm, and what the model sees in it.
//
// The picture is the Pi's own JPEG stream, passed through untouched. A YOLO model can run on it, on this
// PC's GPU in a process of its own: what it finds is outlined over the picture, and a program can wait for
// it. Training pictures for teaching a model your own things are collected here too.

import { $, escape, remember, settled, touch } from '/static/util.js';

// A steady colour for each kind of thing the model knows, from its name.
function classColour(label) {
  let hash = 0;
  for (const ch of label) hash = (hash * 31 + ch.charCodeAt(0)) >>> 0;
  return `hsl(${hash % 360}, 85%, 58%)`;
}

export class CameraPanel {
  constructor(app) {
    this.app = app;
    this.wire();
    this.wireInset();
    this.loadModels();
  }

  // Everything a "wait for" step can wait on: whatever the running model knows.
  labels() {
    const vision = this.app.state?.vision;
    return vision?.model_on ? [...new Set(vision.model?.names || [])].sort() : [];
  }

  async loadModels() {
    try {
      const { models, installed } = await (await fetch('/api/models')).json();
      const select = $('model-name');
      const current = this.app.state?.vision?.model?.model || select.value;
      this.models = new Map(models.map((m) => [m.name, m]));
      select.innerHTML = models.map((m) => {
        const notes = [];
        if (m.open) notes.push('finds things by name');
        if (!m.stock) notes.push('yours');
        if (m.stock && !m.downloaded) notes.push('downloads first time');
        return `<option value="${escape(m.name)}">${escape(m.name)}${notes.length ? ` (${notes.join(', ')})` : ''}</option>`;
      }).join('');
      if (current) select.value = current;
      this.modelsInstalled = installed;
      this.labelClasses();
    } catch {
      $('model-section').classList.add('hidden');  // an older server without models: nothing to show
    }
  }

  // The stream is only asked for when something shows it; a fresh query forces a reconnect.
  startStream(image, force = false) {
    if (image.src && !force) return;
    image.src = `/camera.mjpg?t=${Date.now()}`;
  }

  onShown(tab) {
    if (tab === 'camera') {
      this.startStream($('camera-image'));
      this.loadModels();  // a model trained since the page opened shows up without a reload
    }
  }

  // Open-vocabulary models find whatever they're told to by name, so the box is what to look for; for the
  // others it only narrows down what they already know.
  labelClasses() {
    const open = this.models?.get($('model-name').value)?.open;
    $('model-classes-label').textContent = open ? 'look for' : 'only';
    $('model-classes').placeholder = open ? 'what to find, e.g. well plate, pipette tip, glove'
      : 'everything it knows (or e.g. cup, bottle)';
  }

  sendModel(on) {
    const classes = $('model-classes').value.split(',').map((c) => c.trim()).filter(Boolean);
    this.app.send({
      do: 'model.set', on, model: $('model-name').value,
      conf: Number($('model-conf').value) / 100, classes: classes.length ? classes : null,
    });
  }

  capture() {
    if (!this.app.state?.camera?.connected) return this.app.say('no picture from the camera to capture');
    this.app.send({ do: 'dataset.capture' });
    return true;
  }

  onKey(event) {
    if (event.key === 'c' || event.key === 'C') return this.capture();
    return false;
  }

  // ---- drawing -------------------------------------------------------------------

  // What the model found: an outline where the model draws one (the -seg models), a box where it doesn't.
  // One colour per kind of thing, the same every time, so "cup" is always the same green.
  drawBoxes(holder, found) {
    const outlines = found.filter((hit) => hit.polygon).map((hit) => {
      const c = classColour(hit.label);
      const points = hit.polygon.map(([x, y]) => `${x},${y}`).join(' ');
      return `<polygon points="${points}" style="fill:${c};stroke:${c}"/>`;
    });
    const labels = found.map((hit) => {
      const [x0, y0, x1, y1] = hit.box;
      const c = classColour(hit.label);
      const frame = hit.polygon ? 'found outlined' : 'found';
      return `<div class="${frame}" style="left:${x0 * 100}%;top:${y0 * 100}%;width:${(x1 - x0) * 100}%;`
        + `height:${(y1 - y0) * 100}%;border-color:${c}"><span style="background:${c}">`
        + `${escape(hit.label)} ${Math.round(hit.score * 100)}%</span></div>`;
    });
    holder.innerHTML = (outlines.length ? `<svg class="outlines" viewBox="0 0 1 1" preserveAspectRatio="none">${outlines.join('')}</svg>` : '')
      + labels.join('');
  }

  onState(state) {
    const camera = state.camera;
    const host = $('camera-host');
    if (!host.value && camera.host && document.activeElement !== host) host.value = camera.host.split(':')[0];
    $('camera-placeholder').classList.toggle('hidden', camera.connected);

    const vision = state.vision;
    if (!vision) return;
    const found = vision.model_on ? (vision.found || []) : [];
    this.drawBoxes($('camera-overlay'), found);
    this.drawBoxes($('inset-overlay'), found);
    this.showModel(vision);
    this.showDataset(state.dataset);
  }

  showModel(vision) {
    const model = vision.model;
    $('model-section').classList.toggle('hidden', !model);
    if (!model) return;
    if (settled($('model-on'))) $('model-on').checked = vision.model_on;
    let note;
    if (!model.installed) {
      note = 'Models need the vision extra: run uv sync --extra vision, then restart Slixer.';
    } else if (model.state === 'failed') {
      note = `Stopped: ${model.problem}`;
    } else if (model.state === 'starting') {
      note = `Loading ${model.model}\u2026 (the first time, it downloads)`;
    } else if (model.state === 'running') {
      note = `${model.model} on the ${model.device}: ${Math.round(model.ms)} ms a picture, `
        + `${Math.round(model.rate)} a second \u00b7 knows ${model.names.length} kind${model.names.length === 1 ? '' : 's'} of thing`
        + (vision.found?.length ? ` \u00b7 sees ${[...new Set(vision.found.map((f) => f.label))].join(', ')}` : '');
    } else {
      note = 'A YOLO model, running on this PC\u2019s GPU in a process of its own. What it finds is outlined on '
        + 'the picture and counts for wait-for steps.';
    }
    $('model-note').textContent = note;
    $('model-note').classList.toggle('bad', model.state === 'failed');
  }

  showDataset(dataset) {
    $('dataset-section').classList.toggle('hidden', !dataset);
    if (!dataset) return;
    const here = dataset.datasets.find((d) => d.name === dataset.name);
    const count = here ? `${dataset.name}: ${here.images} pictures, ${here.labelled} with labels` : `${dataset.name}: none yet`;
    $('dataset-note').textContent = dataset.note ? `${dataset.note} \u00b7 ${count}` : count;
    if (settled($('capture-every')) && document.activeElement !== $('capture-every')) $('capture-every').value = String(dataset.auto_every || 0);
    if (document.activeElement !== $('dataset-name') && !this.nameTouched) $('dataset-name').value = dataset.name;
  }

  // ---- wiring --------------------------------------------------------------------

  wire() {
    $('camera-connect').addEventListener('click', () => {
      const host = $('camera-host').value.trim();
      if (!host) return this.app.say('type the Pi’s address first');
      this.app.send({ do: 'camera', host });
      this.startStream($('camera-image'), true);
      if ($('inset-image').src) this.startStream($('inset-image'), true);
    });
    $('camera-host').addEventListener('keydown', (event) => {
      if (event.key === 'Enter') $('camera-connect').click();
    });
    $('model-on').addEventListener('change', (event) => {
      touch(event.target);
      this.sendModel(event.target.checked);
    });
    $('model-name').addEventListener('change', () => {
      this.labelClasses();
      if ($('model-on').checked) this.sendModel(true);
    });
    $('model-conf').addEventListener('input', () => { $('model-conf-value').textContent = `${$('model-conf').value}%`; });
    $('model-conf').addEventListener('change', () => { if ($('model-on').checked) this.sendModel(true); });
    $('model-classes').addEventListener('change', () => { if ($('model-on').checked) this.sendModel(true); });
    $('capture').addEventListener('click', () => this.capture());
    $('capture-every').addEventListener('change', (event) => {
      touch(event.target);
      this.app.send({ do: 'dataset.set', auto_every: Number(event.target.value) });
    });
    $('dataset-prelabel').addEventListener('change', (event) => this.app.send({ do: 'dataset.set', prelabel: event.target.checked }));
    $('dataset-name').addEventListener('change', (event) => {
      this.nameTouched = false;
      this.app.send({ do: 'dataset.set', name: event.target.value.trim() });
    });
    $('dataset-name').addEventListener('input', () => { this.nameTouched = true; });
  }

  // The camera over the model: draggable anywhere, resized from its top-left corner. It keeps its size
  // and place between visits, because the right size depends on what you are doing -- a thumbnail while
  // you work on the arm, most of the window while you line a gripper up with something.
  wireInset() {
    const inset = $('camera-inset');
    const grip = $('inset-grip');
    const MIN = 140;
    const place = (width, right, bottom) => {
      const limit = $('viewport').getBoundingClientRect();
      // Keep it on screen whatever the window has done since: a panel remembered from a wide monitor must
      // not come back off the edge of a laptop.
      width = Math.max(MIN, Math.min(width, limit.width - 24));
      right = Math.max(0, Math.min(right, limit.width - width));
      bottom = Math.max(0, Math.min(bottom, limit.height - 80));
      inset.style.width = `${width}px`;
      inset.style.right = `${right}px`;
      inset.style.bottom = `${bottom}px`;
      remember.set('inset', { width, right, bottom });
    };
    const saved = remember.get('inset', { width: 300, right: 12, bottom: 12 });
    place(saved.width, saved.right, saved.bottom);

    const drag = (element, onMove) => {
      element.addEventListener('pointerdown', (event) => {
        if (event.target.id === 'inset-close') return;
        event.preventDefault();
        event.stopPropagation();
        const box = inset.getBoundingClientRect();
        const limit = $('viewport').getBoundingClientRect();
        const start = { x: event.clientX, y: event.clientY, width: box.width, right: limit.right - box.right, bottom: limit.bottom - box.bottom };
        element.setPointerCapture(event.pointerId);
        const move = (moved) => onMove(start, moved.clientX - start.x, moved.clientY - start.y);
        const done = () => {
          element.removeEventListener('pointermove', move);
          element.removeEventListener('pointerup', done);
          element.removeEventListener('pointercancel', done);
        };
        element.addEventListener('pointermove', move);
        element.addEventListener('pointerup', done);
        element.addEventListener('pointercancel', done);
      });
    };
    drag(grip, (start, dx, dy) => place(start.width - dx, start.right, start.bottom - dy));
    drag(inset, (start, dx, dy) => place(start.width, start.right - dx, start.bottom - dy));
    window.addEventListener('resize', () => {
      const box = inset.getBoundingClientRect();
      const limit = $('viewport').getBoundingClientRect();
      if (box.width) place(box.width, limit.right - box.right, limit.bottom - box.bottom);
    });

    $('btn-inset').addEventListener('click', () => {
      inset.classList.toggle('hidden');
      if (!inset.classList.contains('hidden')) {
        this.startStream($('inset-image'));
        const box = inset.getBoundingClientRect();
        const limit = $('viewport').getBoundingClientRect();
        place(box.width, limit.right - box.right, limit.bottom - box.bottom);
      }
    });
    $('inset-close').addEventListener('click', () => inset.classList.add('hidden'));
  }
}
