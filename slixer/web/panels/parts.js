// Imported parts: the bench, a fixture, the box you pick out of. Drop an STL in, put it where the real one
// sits, and program against something you can see. Double-click a part's surface (in Plan or Drive) and
// the gripper reaches for it, stopping a set distance short along the surface.

import { $, degrees, escape, radians, remember, throttle } from '/static/util.js';

export class PartsPanel {
  constructor(app) {
    this.app = app;
    this.items = [];
    this.selected = null;
    app.on('scene', (reply) => this.apply(reply.items));
    const scene = app.scene;
    scene.onPartPicked = (id) => { if (id !== this.selected) this.select(id); };
    scene.onPartMoved = throttle((id, position) => {
      app.send({ do: 'scene.update', id, changes: { position } });
    }, 60);
    scene.onPartTarget = (point, normal) => this.reach(point, normal);
    $('approach').value = remember.get('approach', 20);
    this.wire();
  }

  async load() {
    const { items } = await (await fetch('/api/scene')).json();
    this.apply(items);
  }

  apply(items) {
    this.items = items;
    this.app.scene.setParts(items);
    if (!items.some((i) => i.id === this.selected)) this.selected = null;
    this.app.scene.select(this.selected, items);
    this.render();
  }

  select(id) {
    this.selected = id;
    this.app.scene.select(id, this.items);
    this.render();
  }

  current() {
    return this.items.find((i) => i.id === this.selected) || null;
  }

  change(changes) {
    if (this.selected) this.app.send({ do: 'scene.update', id: this.selected, changes });
  }

  // Double-clicked a part: aim the gripper tip at the point, backed off along the surface.
  reach(point, normal) {
    const state = this.app.state;
    if (!state?.movable) {
      this.app.say('switch to Plan (or Drive) to send the gripper to a part');
      return;
    }
    const back = Math.max(0, parseFloat($('approach').value) || 0) / 1000;
    const length = Math.hypot(...normal) || 1;
    const aim = point.map((v, i) => v + (normal[i] / length) * back);
    this.app.scene.showTarget(point, normal);
    this.app.send({ do: 'ik', xyz: aim });
    this.app.say(`reaching for the part, ${Math.round(back * 1000)} mm short of its surface`);
    clearTimeout(this.targetTimer);
    this.targetTimer = setTimeout(() => this.app.scene.showTarget(null), 4000);
  }

  render() {
    const list = $('parts');
    list.innerHTML = '';
    for (const item of this.items) {
      const row = document.createElement('li');
      row.dataset.id = item.id;
      row.className = item.id === this.selected ? 'chosen' : '';
      row.innerHTML = `
        <span class="swatch" style="background:${escape(item.colour)}"></span>
        <span class="part-name">${escape(item.name)}</span>
        <button data-act="visible" title="${item.visible ? 'hide' : 'show'}">${item.visible ? '◉' : '○'}</button>`;
      list.appendChild(row);
    }
    if (!this.items.length) list.innerHTML = '<li class="muted empty">nothing imported yet</li>';

    const item = this.current();
    $('part-editor').classList.toggle('hidden', !item);
    if (!item) return;
    const set = (id, value) => {
      const box = $(id);
      if (document.activeElement !== box) box.value = value;
    };
    set('part-name', item.name);
    set('part-x', (item.position[0] * 1000).toFixed(0));
    set('part-y', (item.position[1] * 1000).toFixed(0));
    set('part-z', (item.position[2] * 1000).toFixed(0));
    set('part-roll', degrees(item.rotation[0]).toFixed(0));
    set('part-pitch', degrees(item.rotation[1]).toFixed(0));
    set('part-yaw', degrees(item.rotation[2]).toFixed(0));
    set('part-scale', item.scale);
    $('part-note').textContent = item.note;
  }

  async importFiles(files) {
    for (const file of files) {
      if (!file.name.toLowerCase().endsWith('.stl')) {
        this.app.say(`${file.name} isn't an STL`);
        continue;
      }
      this.app.say(`importing ${file.name}…`);
      const body = new FormData();
      body.append('file', file);
      const reply = await fetch('/api/scene/upload', { method: 'POST', body });
      if (!reply.ok) {
        const { detail } = await reply.json().catch(() => ({ detail: reply.statusText }));
        this.app.say(`couldn't import ${file.name}: ${detail}`);
        continue;
      }
      const { item, items } = await reply.json();
      this.selected = item.id;
      this.apply(items);
      this.app.say(`${item.name}: ${item.note}`);
    }
  }

  wire() {
    $('stl-file').addEventListener('change', (event) => {
      this.importFiles([...event.target.files]);
      event.target.value = '';   // so choosing the same file twice still fires
    });
    const drop = $('drop');
    for (const name of ['dragenter', 'dragover']) {
      drop.addEventListener(name, (event) => { event.preventDefault(); drop.classList.add('over'); });
    }
    for (const name of ['dragleave', 'drop']) drop.addEventListener(name, () => drop.classList.remove('over'));
    drop.addEventListener('drop', (event) => {
      event.preventDefault();
      this.importFiles([...event.dataTransfer.files]);
    });

    $('parts').addEventListener('click', (event) => {
      const row = event.target.closest('li[data-id]');
      if (!row) return;
      if (event.target.dataset.act === 'visible') {
        const item = this.items.find((i) => i.id === row.dataset.id);
        this.app.send({ do: 'scene.update', id: row.dataset.id, changes: { visible: !item.visible } });
        return;
      }
      this.select(row.dataset.id);
    });

    const position = () => ['part-x', 'part-y', 'part-z'].map((id) => (parseFloat($(id).value) || 0) / 1000);
    const rotation = () => ['part-roll', 'part-pitch', 'part-yaw'].map((id) => radians(parseFloat($(id).value) || 0));
    for (const id of ['part-x', 'part-y', 'part-z']) $(id).addEventListener('change', () => this.change({ position: position() }));
    for (const id of ['part-roll', 'part-pitch', 'part-yaw']) $(id).addEventListener('change', () => this.change({ rotation: rotation() }));
    $('part-scale').addEventListener('change', () => this.change({ scale: parseFloat($('part-scale').value) || 1 }));
    $('part-name').addEventListener('change', () => this.change({ name: $('part-name').value }));
    $('part-to-floor').addEventListener('click', () => {
      // Its lowest point on the floor, whichever way it has been turned: the mesh's own bounding box
      // taken in the world, so a part tipped on its side still lands flat.
      const mesh = this.app.scene.partMeshes.get(this.selected);
      const item = this.current();
      if (!mesh || !item) return;
      mesh.updateMatrixWorld(true);
      const lowest = this.lowestZ(mesh);
      this.change({ position: [item.position[0], item.position[1], item.position[2] - lowest] });
    });
    $('part-lie-flat').addEventListener('click', () => this.lieFlat());
    $('part-remove').addEventListener('click', () => {
      const item = this.current();
      if (item && confirm(`Remove "${item.name}" from the scene?`)) this.app.send({ do: 'scene.remove', id: item.id });
    });
    $('approach').addEventListener('change', () => remember.set('approach', parseFloat($('approach').value) || 0));
  }

  // Plates, trays and fixtures often arrive standing on an edge: CAD packages disagree about which way is
  // up, and plenty export with Y up where this model has Z. Turning the part so its thinnest dimension is
  // vertical is nearly always what was meant, then it is set down on the floor.
  lieFlat() {
    const mesh = this.app.scene.partMeshes.get(this.selected);
    const item = this.current();
    if (!mesh || !item) return;
    mesh.geometry.computeBoundingBox();
    const { min, max } = mesh.geometry.boundingBox;
    const extents = [max.x - min.x, max.y - min.y, max.z - min.z];
    const thinnest = extents.indexOf(Math.min(...extents));
    // Roll turns Y into Z; pitching by -90 degrees turns X into Z. Already Z: keep its turn about Z.
    const rotation = thinnest === 2 ? [0, 0, item.rotation[2]] : thinnest === 1 ? [Math.PI / 2, 0, 0] : [0, -Math.PI / 2, 0];
    mesh.rotation.set(...rotation);
    mesh.updateMatrixWorld(true);
    const lowest = this.lowestZ(mesh);
    this.change({ rotation, position: [item.position[0], item.position[1], item.position[2] - lowest] });
    this.app.say(`${item.name}: laid flat, ${Math.round(Math.min(...extents) * item.scale * 1000)} mm tall`);
  }

  // The lowest point of a part, in the arm's own frame (Z up).
  lowestZ(mesh) {
    const position = mesh.geometry.attributes.position;
    const world = this.app.scene.world;
    const toModel = world.matrixWorld.clone().invert().multiply(mesh.matrixWorld);
    let lowest = Infinity;
    const e = toModel.elements;
    for (let i = 0; i < position.count; i++) {
      const x = position.getX(i), y = position.getY(i), z = position.getZ(i);
      const zModel = e[2] * x + e[6] * y + e[10] * z + e[14];
      if (zModel < lowest) lowest = zModel;
    }
    return lowest;
  }
}
