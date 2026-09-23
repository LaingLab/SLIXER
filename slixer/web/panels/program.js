// The program editor.
//
// A program is a list of steps. Select one to edit it below the list; a selected move is drawn in amber so
// you can see where it goes without going there. New steps go in after the selected one, so a program can
// be built in any order. Moves store the arm's own numbers (see program.py), which is why re-lining the
// model up with the arm in Setup never changes what a program does.

import { $, escape, SHORT_LABELS } from '/static/util.js';

const KIND_NAMES = { move: 'move', wait: 'wait', gripper: 'gripper', wait_for: 'wait for' };
const SAFE_NAME = /^[A-Za-z0-9 _-]{1,64}$/;

export class ProgramPanel {
  constructor(app) {
    this.app = app;
    this.steps = [];
    this.selected = null;
    this.dirty = false;
    this.lastSpeed = 40;
    app.on('programs', (reply) => {
      this.fillSaved(reply.names);
      if (reply.text?.startsWith('saved')) this.setDirty(false);
    });
    app.on('program', (reply) => {
      this.open(reply.program);
      this.openedAs = reply.program.name;
    });
    app.on('program.exists', (reply) => {
      if (confirm(`There's already a saved program called "${reply.name}". Replace it?`)) {
        this.app.send({ do: 'program.save', program: this.toJson(), replace: true });
      }
    });
    this.wire();
    this.render();
  }

  async load() {
    const { names } = await (await fetch('/api/programs')).json();
    this.fillSaved(names);
  }

  // ---- the program as data ---------------------------------------------------

  toJson() {
    return {
      name: $('program-name').value.trim() || 'untitled',
      loop: $('program-loop').checked,
      steps: this.steps,
    };
  }

  open(program) {
    $('program-name').value = program.name;
    $('program-loop').checked = program.loop;
    this.steps = program.steps;
    this.selected = null;
    this.setDirty(false);
    this.render();
    this.preview();
    this.app.say(`opened "${program.name}"`);
  }

  setDirty(dirty) {
    this.dirty = dirty;
    $('program-dirty').textContent = dirty ? 'unsaved' : '';
  }

  changed() {
    this.setDirty(true);
    this.render();
    this.preview();
  }

  // ---- adding and changing steps ------------------------------------------------

  insert(step) {
    const at = this.selected === null ? this.steps.length : this.selected + 1;
    this.steps.splice(at, 0, step);
    this.selected = at;
    this.changed();
  }

  record() {
    const state = this.app.state;
    if (!state) return;
    this.insert({ kind: 'move', arm: [...state.arm_pose], speed: this.lastSpeed, note: '' });
    this.app.say(`recorded ${state.mode === 'plan' ? 'the virtual arm' : 'the arm'} as step ${this.selected + 1}`);
  }

  select(index) {
    this.selected = index === null || index < 0 || index >= this.steps.length ? null : index;
    // Only the highlight changes: rebuilding the list here would replace the element under the pointer
    // between the two clicks of a double-click, and the double-click would never arrive.
    [...$('steps').children].forEach((item, i) => item.classList.toggle('selected', i === this.selected));
    this.renderEditor();
    this.preview();
    $('play-here').disabled = this.selected === null || !this.app.state?.movable;
  }

  preview() {
    const step = this.steps[this.selected];
    this.app.scene.setPreview(step?.kind === 'move' ? this.app.armToModel(step.arm) : null);
  }

  act(action) {
    const index = this.selected;
    const step = this.steps[index];
    if (!step) return;
    if (action === 'delete') {
      this.steps.splice(index, 1);
      this.selected = this.steps.length ? Math.min(index, this.steps.length - 1) : null;
    } else if (action === 'duplicate') {
      this.steps.splice(index + 1, 0, structuredClone(step));
      this.selected = index + 1;
    } else if (action === 'up' && index > 0) {
      [this.steps[index - 1], this.steps[index]] = [this.steps[index], this.steps[index - 1]];
      this.selected = index - 1;
    } else if (action === 'down' && index < this.steps.length - 1) {
      [this.steps[index + 1], this.steps[index]] = [this.steps[index], this.steps[index + 1]];
      this.selected = index + 1;
    } else if (action === 'set-pose' && this.app.state) {
      step.arm = [...this.app.state.arm_pose];
      this.app.say(`step ${index + 1} now goes where the arm is`);
    } else if (action === 'go') {
      this.app.send({ do: 'target_arm', arm: step.arm });
      return;
    } else {
      return;
    }
    this.changed();
  }

  // ---- drawing ---------------------------------------------------------------------

  summary(step) {
    if (step.kind === 'move') {
      const joints = step.arm.slice(0, 5).map((v) => v.toFixed(0)).join(' · ');
      return `${joints} · ${step.arm[5].toFixed(0)}%  @ ${step.speed}°/s`;
    }
    if (step.kind === 'wait') return `${step.seconds} s`;
    if (step.kind === 'gripper') return `${step.percent >= 99 ? 'open' : step.percent <= 1 ? 'shut' : `${step.percent}%`}`;
    if (step.kind === 'wait_for') {
      const label = step.label ? `"${escape(step.label)}"` : '<em>nothing chosen</em>';
      return `until the camera sees ${label}${step.seconds > 0 ? `, up to ${step.seconds} s` : ''}`;
    }
    return '';
  }

  render() {
    const list = $('steps');
    list.innerHTML = '';
    if (!this.steps.length) {
      list.innerHTML = '<li class="empty">No steps yet. Pose the arm and press <b>record pose</b> (or R).</li>';
    }
    this.steps.forEach((step, index) => {
      const item = document.createElement('li');
      item.dataset.index = index;
      item.classList.toggle('selected', index === this.selected);
      const note = step.note ? `<span class="step-note">${escape(step.note)}</span>` : '';
      item.innerHTML = `
        <span class="num">${index + 1}</span>
        <span class="kind ${step.kind}">${KIND_NAMES[step.kind]}</span>
        <span class="detail">${note}${this.summary(step)}</span>`;
      list.appendChild(item);
    });
    this.renderEditor();
    const seconds = this.duration();
    $('program-length').textContent = this.steps.length
      ? `${this.steps.length} steps · about ${seconds < 60 ? `${seconds.toFixed(0)} s` : `${(seconds / 60).toFixed(1)} min`}`
      : '';
  }

  renderEditor() {
    const editor = $('step-editor');
    const step = this.steps[this.selected];
    editor.classList.toggle('hidden', !step);
    if (!step) return;
    const n = this.selected + 1;
    let body = '';
    if (step.kind === 'move') {
      const values = step.arm.map((v, i) => `
        <span class="jv"><b>${SHORT_LABELS[this.app.model.motors[i]]}</b>${i < 5 ? `${v.toFixed(1)}°` : `${v.toFixed(0)}%`}</span>`).join('');
      body = `
        <div class="joint-values">${values}</div>
        <label class="field">speed <input type="number" data-field="speed" min="1" max="180" step="5" value="${step.speed}"> °/s</label>
        <div class="row">
          <button data-act="set-pose" title="replace this pose with where the arm is now">set to current pose</button>
          <button data-act="go" title="move the model there (Plan) or the arm (Drive)">go there</button>
        </div>`;
    } else if (step.kind === 'wait') {
      body = `<label class="field">wait <input type="number" data-field="seconds" min="0" max="3600" step="0.5" value="${step.seconds}"> seconds</label>`;
    } else if (step.kind === 'gripper') {
      body = `
        <label class="field">open <input type="range" data-field="percent" min="0" max="100" step="1" value="${step.percent}">
          <output>${step.percent}%</output></label>
        <div class="row"><button data-set-percent="0">shut</button><button data-set-percent="100">open</button></div>`;
    } else if (step.kind === 'wait_for') {
      const labels = this.app.camera?.labels() || [];
      const options = labels.map((l) => `<option value="${escape(l)}">`).join('');
      body = `
        <label class="field">until the camera sees
          <input data-field="label" list="vision-labels" value="${escape(step.label)}" placeholder="a name the model knows, e.g. cup" spellcheck="false">
          <datalist id="vision-labels">${options}</datalist></label>
        <label class="field">give up after <input type="number" data-field="seconds" min="0" max="3600" step="5" value="${step.seconds}"> s <span class="muted small">(0 = wait for ever)</span></label>
        ${labels.length ? '' : '<p class="muted small">Start a model on the Camera tab to pick from what it knows.</p>'}`;
    }
    editor.innerHTML = `
      <div class="editor-head"><span class="kind ${step.kind}">${KIND_NAMES[step.kind]}</span> step ${n} of ${this.steps.length}</div>
      ${body}
      <label class="field">note <input data-field="note" value="${escape(step.note || '')}" placeholder="what this step is for" spellcheck="false"></label>
      <div class="row small-buttons">
        <button data-act="duplicate">duplicate</button>
        <button data-act="up" title="earlier">↑</button>
        <button data-act="down" title="later">↓</button>
        <button data-act="delete" class="danger">delete</button>
      </div>`;
  }

  duration() {
    let total = 0;
    let here = null;
    for (const step of this.steps) {
      if (step.kind === 'move') {
        if (here) total += Math.max(0.15, Math.max(...step.arm.slice(0, 5).map((v, i) => Math.abs(v - here[i]))) / step.speed);
        here = step.arm;
      } else if (step.kind === 'wait') {
        total += step.seconds;
      } else if (step.kind === 'gripper') {
        total += 0.6;
      }
    }
    return total;
  }

  fillSaved(names) {
    const select = $('saved-programs');
    const current = select.value;
    select.innerHTML = `<option value="">saved programs (${names.length})…</option>`;
    for (const name of names) {
      const option = document.createElement('option');
      option.value = option.textContent = name;
      select.appendChild(option);
    }
    if (names.includes(current)) select.value = current;
  }

  // ---- wiring ------------------------------------------------------------------------

  wire() {
    $('add-move').addEventListener('click', () => this.record());
    $('add-wait').addEventListener('click', () => this.insert({ kind: 'wait', seconds: 1, note: '' }));
    $('add-gripper').addEventListener('click', () => {
      // Whichever way the gripper isn't now: the next thing a gripper step does is usually change it.
      const open = (this.app.state?.arm_pose?.[5] ?? 0) > 50;
      this.insert({ kind: 'gripper', percent: open ? 0 : 100, note: '' });
    });
    $('add-waitfor').addEventListener('click', () => {
      const labels = this.app.camera?.labels() || [];
      this.insert({ kind: 'wait_for', label: labels[0] || '', seconds: 0, note: '' });
    });

    $('steps').addEventListener('click', (event) => {
      const item = event.target.closest('li[data-index]');
      if (item) this.select(parseInt(item.dataset.index, 10));
    });
    $('steps').addEventListener('dblclick', (event) => {
      const item = event.target.closest('li[data-index]');
      if (item && this.steps[item.dataset.index]?.kind === 'move') this.act('go');
    });

    const editor = $('step-editor');
    editor.addEventListener('click', (event) => {
      const button = event.target.closest('button');
      if (!button) return;
      if (button.dataset.act) this.act(button.dataset.act);
      if (button.dataset.setPercent !== undefined) {
        this.steps[this.selected].percent = Number(button.dataset.setPercent);
        this.changed();
      }
    });
    editor.addEventListener('input', (event) => {
      const field = event.target.dataset.field;
      const step = this.steps[this.selected];
      if (!field || !step) return;
      if (field === 'note' || field === 'label') {
        step[field] = event.target.value.slice(0, field === 'note' ? 200 : 64);
        this.setDirty(true);
        // Redraw the list only: redrawing the editor would steal the text box from under the cursor.
        const item = $('steps').children[this.selected];
        if (item) item.querySelector('.detail').innerHTML =
          `${step.note ? `<span class="step-note">${escape(step.note)}</span>` : ''}${this.summary(step)}`;
        return;
      }
      const value = parseFloat(event.target.value);
      if (!Number.isFinite(value)) return;
      if (field === 'speed') {
        step.speed = Math.max(1, Math.min(180, value));
        this.lastSpeed = step.speed;
      } else if (field === 'seconds') {
        step.seconds = Math.max(0, Math.min(3600, value));
      } else if (field === 'percent') {
        step.percent = Math.max(0, Math.min(100, value));
        const output = event.target.parentElement.querySelector('output');
        if (output) output.textContent = `${step.percent}%`;
      }
      this.setDirty(true);
      const item = $('steps').children[this.selected];
      if (item) item.querySelector('.detail').innerHTML = this.summary(step);
      $('program-length').textContent = `${this.steps.length} steps · about ${this.duration().toFixed(0)} s`;
    });

    $('program-name').addEventListener('input', () => this.setDirty(true));
    $('program-loop').addEventListener('change', () => this.setDirty(true));

    $('play').addEventListener('click', () => this.play(0));
    $('play-here').addEventListener('click', () => this.play(this.selected ?? 0));
    $('stop-program').addEventListener('click', () => this.app.send({ do: 'program.stop' }));

    $('save-program').addEventListener('click', () => {
      const program = this.toJson();
      if (!SAFE_NAME.test(program.name)) {
        this.app.say('a program name may only use letters, numbers, spaces, - and _');
        return;
      }
      // Saving the program that was opened under this name replaces it, as "save" should. Any other
      // clash is asked about first (see the 'program.exists' reply).
      this.app.send({ do: 'program.save', program, replace: program.name === this.openedAs });
      this.openedAs = program.name;
    });
    $('open-program').addEventListener('click', () => {
      const name = $('saved-programs').value;
      if (!name) return this.app.say('choose a saved program first');
      if (this.dirty && !confirm('Open another program and lose the changes to this one?')) return;
      this.app.send({ do: 'program.load', name });
    });
    $('new-program').addEventListener('click', () => {
      if (this.dirty && !confirm('Start a new program and lose the changes to this one?')) return;
      // A name no saved program has, so a new program can never be saved over an old one by accident.
      const taken = new Set([...$('saved-programs').options].map((o) => o.value));
      let name = 'untitled';
      for (let n = 2; taken.has(name); n++) name = `untitled ${n}`;
      this.open({ name, loop: false, steps: [] });
      this.openedAs = null;
    });
    $('delete-program').addEventListener('click', () => {
      const name = $('saved-programs').value;
      if (!name) return this.app.say('choose a saved program to delete');
      if (confirm(`Delete the saved program "${name}"? This can't be undone.`)) {
        this.app.send({ do: 'program.delete', name });
      }
    });
  }

  play(start) {
    if (!this.steps.length) return this.app.say('nothing to play yet');
    this.app.send({ do: 'program.run', program: this.toJson(), start });
  }

  onKey(event) {
    if (event.key === 'r' || event.key === 'R') {
      this.record();
      return true;
    }
    if (this.app.activeTab !== 'program') return false;
    if (event.key === 'Delete' || event.key === 'Backspace') {
      this.act('delete');
      return true;
    }
    if (event.key === 'ArrowDown' || event.key === 'ArrowUp') {
      const step = event.key === 'ArrowDown' ? 1 : -1;
      const next = this.selected === null ? 0 : this.selected + step;
      if (next >= 0 && next < this.steps.length) this.select(next);
      return true;
    }
    return false;
  }

  onState(state) {
    const running = state.program.running;
    const can = state.mode !== 'watch';
    $('play').disabled = !can || running;
    $('play-here').disabled = !can || running || this.selected === null;
    $('stop-program').disabled = !running;
    $('play').title = can ? 'Play from the start' : 'Switch to Plan to rehearse, or Drive to run it on the arm';

    const note = running
      ? `${state.mode === 'plan' ? 'rehearsing' : 'running'} step ${state.program.index + 1} of ${state.program.count}`
        + (state.program.waiting_for ? ` — waiting to see "${state.program.waiting_for}"` : '')
        + (state.program.laps ? ` · lap ${state.program.laps + 1}` : '')
      : (state.program.message || '');
    $('program-note').textContent = note;
    [...$('steps').children].forEach((item, i) => item.classList.toggle('running', running && i === state.program.index));
  }
}
