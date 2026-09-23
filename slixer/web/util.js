// Small things every panel uses.

export const $ = (id) => document.getElementById(id);
export const degrees = (radians) => (radians * 180) / Math.PI;
export const radians = (deg) => (deg * Math.PI) / 180;

// Anything that came from outside -- a file name, a label someone typed -- goes through this before it
// goes into HTML. A part imported from "<img onerror=...>.stl" must show up as a name, not run.
export function escape(text) {
  return String(text ?? '').replace(/[&<>"']/g, (c) => (
    { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]
  ));
}

export const JOINT_LABELS = {
  shoulder_pan: 'shoulder pan',
  shoulder_lift: 'shoulder lift',
  elbow_flex: 'elbow',
  wrist_flex: 'wrist flex',
  wrist_roll: 'wrist roll',
  gripper: 'gripper',
};

export const SHORT_LABELS = {
  shoulder_pan: 'pan', shoulder_lift: 'lift', elbow_flex: 'elbow',
  wrist_flex: 'flex', wrist_roll: 'roll', gripper: 'grip',
};

// Remembered in this browser only: panel sizes, the last tab. Never anything that matters if lost.
export const remember = {
  get(key, fallback) {
    try {
      const value = localStorage.getItem(`slixer.${key}`);
      return value === null ? fallback : JSON.parse(value);
    } catch {
      return fallback;
    }
  },
  set(key, value) {
    try { localStorage.setItem(`slixer.${key}`, JSON.stringify(value)); } catch { /* private window */ }
  },
};

// Runs `fn` at most once per `ms` while calls keep coming -- and always runs it once more with the last
// call's arguments after they stop. Dropping the last call would leave a flicked slider or a dragged
// gripper short of where it was let go, and the arm with it.
export function throttle(fn, ms) {
  let last = 0;
  let timer = null;
  let pending = null;
  const throttled = (...args) => {
    const wait = ms - (performance.now() - last);
    if (wait <= 0) {
      clearTimeout(timer);
      timer = null;
      last = performance.now();
      fn(...args);
      return;
    }
    pending = args;
    if (!timer) {
      timer = setTimeout(() => {
        timer = null;
        last = performance.now();
        fn(...pending);
      }, wait);
    }
  };
  // Drops the call still waiting to go, if there is one. STOP uses it: a move the throttle was holding
  // back must not follow the STOP to the server and set the arm off again.
  throttled.cancel = () => {
    clearTimeout(timer);
    timer = null;
    pending = null;
  };
  return throttled;
}

// A control the user has just changed shouldn't be set back by the next update from the server, which can
// arrive before the server has acted on the change: the tick box would flick back, then forward again.
const touchedAt = new WeakMap();
export function touch(element) {
  touchedAt.set(element, performance.now());
}
export function settled(element, ms = 1000) {
  const at = touchedAt.get(element);
  return at === undefined || performance.now() - at > ms;
}
