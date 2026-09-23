// Lining the model up with the arm.
//
// Each joint shows the arm's own reading beside the model's, live, so lining up is a matter of moving a
// joint by hand and watching whether the two agree: flip the ones that turn the wrong way, offset the ones
// that sit at an angle, save.

import { $, degrees, JOINT_LABELS, settled, touch } from '/static/util.js';

// What's in a number box, or `fallback` if it isn't a number. Not `parseFloat(text) || fallback`: that
// turns a typed 0 into the fallback, and 0 is a perfectly good angle.
const numberOr = (text, fallback) => {
  const value = parseFloat(text);
  return Number.isFinite(value) ? value : fallback;
};

export class SetupPanel {
  constructor(app) {
    this.app = app;
    this.build();
    this.wire();
  }

  build() {
    const body = document.querySelector('#mapping-table tbody');
    body.innerHTML = '';
    for (const name of this.app.model.motors.slice(0, 5)) {
      const row = document.createElement('tr');
      row.innerHTML = `
        <td>${JOINT_LABELS[name]}</td>
        <td class="num" id="arm-${name}">—</td>
        <td class="num" id="model-${name}">—</td>
        <td><button class="flip" id="flip-${name}" title="this joint turns the wrong way">flip</button></td>
        <td><input type="number" id="offset-${name}" step="1" class="inline" title="degrees to shift the model's zero">°</td>`;
      body.appendChild(row);
    }
  }

  fill(mapping) {
    for (const [name, values] of Object.entries(mapping.joints)) {
      const offset = $(`offset-${name}`);
      if (offset && document.activeElement !== offset && settled(offset)) offset.value = values.offset_deg.toFixed(1);
      const flip = $(`flip-${name}`);
      if (flip && settled(flip)) flip.classList.toggle('flipped', values.sign < 0);
    }
    for (const [id, value] of [['grip-closed', mapping.gripper.closed_deg], ['grip-open', mapping.gripper.open_deg]]) {
      if (document.activeElement !== $(id) && settled($(id))) $(id).value = value.toFixed(0);
    }
  }

  collect() {
    const joints = {};
    for (const name of this.app.model.motors.slice(0, 5)) {
      joints[name] = {
        sign: $(`flip-${name}`).classList.contains('flipped') ? -1 : 1,
        offset_deg: numberOr($(`offset-${name}`).value, 0),
      };
    }
    return {
      joints,
      gripper: {
        closed_deg: numberOr($('grip-closed').value, 0),
        open_deg: numberOr($('grip-open').value, 90),
      },
    };
  }

  // Changes take effect as soon as they are made, so the model can be watched moving into line; "save"
  // is what keeps them. Every change is sent whole, and the server checks it all before using any of it.
  push(save = false) {
    this.app.send({ do: 'mapping', save, ...this.collect() });
  }

  wire() {
    for (const name of this.app.model.motors.slice(0, 5)) {
      $(`flip-${name}`).addEventListener('click', (event) => {
        touch(event.target);
        event.target.classList.toggle('flipped');
        this.push();
      });
      $(`offset-${name}`).addEventListener('change', (event) => {
        touch(event.target);
        this.push();
      });
    }
    for (const id of ['grip-closed', 'grip-open']) {
      $(id).addEventListener('change', (event) => {
        touch(event.target);
        this.push();
      });
    }
    $('save-mapping').addEventListener('click', () => this.push(true));
    $('zero-here').addEventListener('click', () => {
      if (confirm('Take the real arm’s current pose as the model’s zero pose? This sets every offset.')) {
        this.app.send({ do: 'mapping.zero' });
      }
    });
  }

  onState(state) {
    this.fill(state.mapping);  // skips whichever box has the cursor, so typing is never overwritten
    const reporting = Boolean(state.real);
    for (const [index, name] of this.app.model.motors.slice(0, 5).entries()) {
      // The arm's own number, straight from the arm; and the model's angle for the same joint.
      $(`arm-${name}`).textContent = state.real_degrees ? `${state.real_degrees[index].toFixed(1)}°` : '—';
      $(`model-${name}`).textContent = reporting ? `${degrees(state.real[index]).toFixed(1)}°` : '—';
    }
    $('zero-here').disabled = !reporting;

    const status = $('setup-status');
    let text;
    let tone;
    if (!reporting) {
      [text, tone] = ['The arm isn’t reporting, so there is nothing to line the model up against yet. '
        + 'Power the follower and wait for the pill at the top left.', 'idle'];
    } else if (state.mapping_dirty) {
      [text, tone] = ['Changed but not saved: the model is using these settings now, and goes back to the '
        + 'saved ones next time unless you press save.', 'warn'];
    } else if (!state.mapping_saved) {
      [text, tone] = ['Not lined up yet: the model is using defaults, so some joints may turn the wrong way. '
        + 'Follow the steps below — it takes a couple of minutes, once.', 'warn'];
    } else {
      [text, tone] = ['Lined up and saved. Move a joint by hand to check the model still agrees.', 'good'];
    }
    status.textContent = text;
    status.className = `card status ${tone}`;
  }
}
