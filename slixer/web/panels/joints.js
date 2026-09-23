// A slider per motor, in the arm's own numbers: degrees from the middle of each joint's calibrated travel,
// and the gripper in percent -- the numbers programs store and the leader prints. In Plan and Drive they
// pose the model; in Watch they only show where the arm is.

import { $, JOINT_LABELS, throttle } from '/static/util.js';

export class JointsPanel {
  constructor(app) {
    this.app = app;
    this.holding = new Set();   // sliders the user has hold of: the server must not yank them back
    this.send = throttle((arm) => app.send({ do: 'target_arm', arm }), 20);
    this.build();
  }

  build() {
    const holder = $('sliders');
    holder.innerHTML = '';
    for (const [index, name] of this.app.model.motors.entries()) {
      const row = document.createElement('div');
      row.className = 'slider-row';
      row.innerHTML = `
        <label for="slider-${name}">${JOINT_LABELS[name] || name}</label>
        <input type="range" id="slider-${name}" min="-100" max="100" step="0.5" value="0">
        <output id="value-${name}">0</output>`;
      holder.appendChild(row);

      const slider = row.querySelector('input');
      slider.addEventListener('pointerdown', () => this.holding.add(name));
      const release = () => {
        this.holding.delete(name);
        slider.blur();  // give the keyboard back: R and Esc must work straight after touching a slider
      };
      slider.addEventListener('pointerup', release);
      slider.addEventListener('pointercancel', release);
      slider.addEventListener('blur', () => this.holding.delete(name));
      // A focused range input answers Home, End and the arrows by moving -- End sends the joint to its
      // limit. In Drive that's the real arm, so these sliders only ever move by hand.
      slider.addEventListener('keydown', (event) => {
        if (['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', 'Home', 'End', 'PageUp', 'PageDown'].includes(event.key)) {
          event.preventDefault();
        }
      });
      slider.addEventListener('input', () => {
        const state = this.app.state;
        if (!state?.movable) return;
        const arm = [...state.commanded_arm];
        arm[index] = parseFloat(slider.value);
        $(`value-${name}`).textContent = this.format(index, arm[index]);
        this.send(arm);
      });
    }

    $('zero-pose').addEventListener('click', () => {
      this.app.send({ do: 'target', pose: this.app.model.motors.map(() => 0) });
    });
    $('match-arm').addEventListener('click', () => {
      const real = this.app.state?.real;
      if (!real) {
        this.app.say('no real arm reporting to match');
        return;
      }
      this.app.send({ do: 'target', pose: real });
    });
  }

  format(index, value) {
    return index < 5 ? `${value.toFixed(1)}°` : `${value.toFixed(0)}%`;
  }

  onState(state) {
    const movable = state.movable;
    $('joints-hint').textContent = {
      watch: 'Watching: these follow the real arm. Switch to Plan to pose the model, or Drive to move the arm.',
      plan: 'Planning: these pose the virtual arm. Nothing is sent to the real one.',
      drive: 'Driving: these move the real arm, at up to 90° a second.',
    }[state.mode];
    $('zero-pose').disabled = !movable;
    $('match-arm').disabled = !movable || !state.real;

    for (const [index, name] of this.app.model.motors.entries()) {
      const slider = $(`slider-${name}`);
      slider.disabled = !movable;
      if (this.holding.has(name)) continue;
      const [low, high] = state.limits_arm[index];
      slider.min = low.toFixed(1);
      slider.max = high.toFixed(1);
      slider.step = index < 5 ? '0.5' : '1';
      slider.value = state.commanded_arm[index].toFixed(1);
      $(`value-${name}`).textContent = this.format(index, state.arm_pose[index]);
    }
  }
}
