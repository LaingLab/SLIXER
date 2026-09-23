// The 3D view: the arm, two see-through copies of it, the parts you have imported, and the ways of
// grabbing things with the mouse.
//
// The arm's shape comes down as JSON from the same URDF the kinematics uses, so there is nothing to keep
// in step by hand. Each link is a group holding one baked mesh, parented exactly as the joints say, so
// posing the arm is writing each joint's matrix. The same tree is built three times with different
// materials:
//
//   arm      solid: the pose the model is in right now
//   ghost    blue, see-through: where the real arm is, when that differs from the model
//   preview  amber, see-through: a program step you are looking at, without going there

import * as THREE from 'three';
import { OrbitControls } from '/static/vendor/OrbitControls.js';
import { STLLoader } from '/static/vendor/STLLoader.js';

const HANDLE = 0xff9d3f;
const TARGET = 0x46d18a;

const MATERIALS = {
  arm: () => new THREE.MeshStandardMaterial({ color: 0xb9c2cc, roughness: 0.55, metalness: 0.25 }),
  ghost: () => new THREE.MeshStandardMaterial({
    color: 0x4fd1ff, transparent: true, opacity: 0.25, roughness: 0.6, depthWrite: false,
  }),
  preview: () => new THREE.MeshStandardMaterial({
    color: 0xffb347, transparent: true, opacity: 0.32, roughness: 0.6, depthWrite: false,
  }),
};

export class ArmScene {
  constructor(holder) {
    this.holder = holder;
    this.trees = {};
    this.showGhost = true;
    this.partMeshes = new Map();  // id -> THREE.Mesh
    this.selected = null;         // the part being edited

    // Things the page wants to hear about.
    this.onDrag = null;        // (xyz) while the gripper handle is dragged
    this.onPartPicked = null;  // (id) when a part is clicked
    this.onPartMoved = null;   // (id, xyz) while a part is dragged
    this.onPartTarget = null;  // (point, normal) when a part's surface is double-clicked

    this._build();
  }

  _build() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x11151a);

    const width = this.holder.clientWidth || 800;
    const height = this.holder.clientHeight || 600;
    this.camera = new THREE.PerspectiveCamera(45, width / height, 0.01, 50);
    this.camera.position.set(0.55, 0.45, 0.55);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(width, height);
    this.holder.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.target.set(0, 0.12, 0);

    this.scene.add(new THREE.HemisphereLight(0xdfe9f5, 0x20262e, 2.0));
    const key = new THREE.DirectionalLight(0xffffff, 2.2);
    key.position.set(0.6, 1.0, 0.4);
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0x93b5d8, 0.8);
    fill.position.set(-0.5, 0.3, -0.6);
    this.scene.add(fill);
    this.scene.add(new THREE.GridHelper(1.2, 24, 0x2c3947, 0x1b232c));

    // URDF is Z-up; three.js is Y-up. One rotation on each root keeps every other number identical to
    // what the maths on the server produces, so positions can be compared directly.
    this.world = new THREE.Group();
    this.world.rotation.x = -Math.PI / 2;
    this.scene.add(this.world);

    this.parts = new THREE.Group();
    this.world.add(this.parts);

    this.handle = new THREE.Mesh(
      new THREE.SphereGeometry(0.012, 24, 16),
      new THREE.MeshStandardMaterial({ color: HANDLE, emissive: 0x5a2c00, roughness: 0.35 }),
    );
    this.scene.add(this.handle);

    this.target = new THREE.Group();
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(0.012, 0.0025, 8, 32),
      new THREE.MeshBasicMaterial({ color: TARGET }),
    );
    const dot = new THREE.Mesh(new THREE.SphereGeometry(0.004, 12, 8), new THREE.MeshBasicMaterial({ color: TARGET }));
    this.target.add(ring, dot);
    this.target.visible = false;
    this.world.add(this.target);

    this.raycaster = new THREE.Raycaster();
    this.dragPlane = new THREE.Plane();
    this.pointer = new THREE.Vector2();
    this._wirePointer();

    window.addEventListener('resize', () => this.resize());
    this.renderer.setAnimationLoop(() => {
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    });
  }

  resize() {
    const width = this.holder.clientWidth;
    const height = this.holder.clientHeight;
    if (!width || !height) return;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  // ---- building the arm from the server's description ----------------------

  async load(model) {
    this.model = model;
    for (const name of Object.keys(MATERIALS)) this.trees[name] = this._tree(name);
    this.trees.ghost.root.visible = false;
    this.trees.preview.root.visible = false;

    const loader = new STLLoader();
    const withMeshes = model.meshes ? model.links.filter((l) => model.meshes.includes(l)) : model.links;
    await Promise.all(withMeshes.map(async (link) => {
      let geometry;
      try {
        geometry = await loader.loadAsync(`/static/models/${link}.stl`);
      } catch {
        return; // a frame with no shape of its own, such as the gripper tip
      }
      geometry.computeVertexNormals();
      for (const tree of Object.values(this.trees)) {
        tree.links.get(link).add(new THREE.Mesh(geometry, tree.material));
      }
    }));

    this.setPose(model.motors.map(() => 0));
    this.frame();
  }

  _tree(name) {
    const root = new THREE.Group();
    const links = new Map();
    const joints = new Map();
    for (const link of this.model.links) {
      const group = new THREE.Group();
      group.matrixAutoUpdate = false;
      links.set(link, group);
    }
    root.add(links.get(this.model.root));
    for (const joint of this.model.joints) {
      const origin = new THREE.Matrix4().set(...joint.origin); // the server sends row-major
      const child = links.get(joint.child);
      links.get(joint.parent).add(child);
      child.matrix.copy(origin);
      joints.set(joint.name, { group: child, origin, axis: new THREE.Vector3(...joint.axis), actuated: joint.actuated });
    }
    this.world.add(root);
    return { root, links, joints, material: MATERIALS[name]() };
  }

  _pose(tree, angles) {
    const values = new Map(this.model.motors.map((name, index) => [name, angles[index] ?? 0]));
    const rotation = new THREE.Matrix4();
    for (const [name, joint] of tree.joints) {
      if (!joint.actuated) continue;
      rotation.makeRotationAxis(joint.axis, values.get(name) ?? 0);
      joint.group.matrix.copy(joint.origin).multiply(rotation);
    }
    tree.root.updateMatrixWorld(true);
  }

  setPose(angles) {
    this._pose(this.trees.arm, angles);
    const tip = this.trees.arm.links.get(this.model.tip);
    if (tip) this.handle.position.setFromMatrixPosition(tip.matrixWorld);
  }

  setGhost(angles) {
    const tree = this.trees.ghost;
    tree.root.visible = Boolean(angles) && this.showGhost;
    if (tree.root.visible) this._pose(tree, angles);
  }

  setPreview(angles) {
    const tree = this.trees.preview;
    tree.root.visible = Boolean(angles);
    if (angles) this._pose(tree, angles);
  }

  setHandleVisible(visible) {
    this.handle.visible = visible;
  }

  // A point in the arm's own frame (metres, Z up) -- what the server's IK expects.
  toModel(worldPoint) {
    return this.world.worldToLocal(worldPoint.clone()).toArray();
  }

  showTarget(point, normal) {
    this.target.visible = Boolean(point);
    if (!point) return;
    this.target.position.set(...point);
    const up = new THREE.Vector3(0, 0, 1);
    this.target.quaternion.setFromUnitVectors(up, new THREE.Vector3(...(normal || [0, 0, 1])).normalize());
  }

  frame() {
    const box = new THREE.Box3().setFromObject(this.trees.arm.root);
    for (const mesh of this.partMeshes.values()) if (mesh.visible) box.expandByObject(mesh);
    if (box.isEmpty()) return;
    const centre = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3()).length();
    this.controls.target.copy(centre);
    this.camera.position.copy(centre).add(new THREE.Vector3(1, 0.85, 1).setLength(size * 1.1));
    this.controls.update();
  }

  // ---- imported parts ------------------------------------------------------

  async setParts(items) {
    const wanted = new Set(items.map((i) => i.id));
    for (const [id, mesh] of this.partMeshes) {
      if (wanted.has(id)) continue;
      this.parts.remove(mesh);
      mesh.geometry.dispose();
      mesh.material.dispose();
      this.partMeshes.delete(id);
    }
    const loader = new STLLoader();
    for (const item of items) {
      let mesh = this.partMeshes.get(item.id);
      if (!mesh) {
        let geometry;
        try {
          geometry = await loader.loadAsync(`/api/scene/mesh/${item.id}.stl`);
        } catch {
          continue; // deleted between listing it and drawing it
        }
        geometry.computeVertexNormals();
        mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ roughness: 0.65, metalness: 0.1 }));
        mesh.userData.partId = item.id;
        this.parts.add(mesh);
        this.partMeshes.set(item.id, mesh);
      }
      this.placePart(item);
    }
  }

  placePart(item) {
    const mesh = this.partMeshes.get(item.id);
    if (!mesh) return;
    mesh.position.set(...item.position);
    mesh.rotation.set(...item.rotation);
    mesh.scale.setScalar(item.scale);
    mesh.visible = item.visible;
    mesh.material.color.set(item.colour);
    // The part being edited is lit up, so there is never a doubt about which one the numbers apply to.
    const chosen = item.id === this.selected;
    mesh.material.emissive.set(chosen ? item.colour : 0x000000);
    mesh.material.emissiveIntensity = chosen ? 0.35 : 0;
  }

  select(id, items) {
    this.selected = id;
    for (const item of items) this.placePart(item);
  }

  // ---- the mouse ---------------------------------------------------------------
  //
  // One pointer, several jobs. Pressing on the orange handle drags the gripper; pressing on a part picks
  // it up and drags it; double-clicking a part's surface asks for the gripper to go there; anything else
  // turns the camera.

  _wirePointer() {
    const element = this.renderer.domElement;
    let mode = null;
    let heldPart = null;
    let grabOffset = new THREE.Vector3();
    let pressedAt = null;

    const aim = (event) => {
      const rect = element.getBoundingClientRect();
      this.pointer.set(
        ((event.clientX - rect.left) / rect.width) * 2 - 1,
        -((event.clientY - rect.top) / rect.height) * 2 + 1,
      );
      this.raycaster.setFromCamera(this.pointer, this.camera);
    };
    const visibleParts = () => this.parts.children.filter((m) => m.visible);
    // Drag within the plane facing the camera through the thing being moved: it then follows the pointer
    // the way you expect, whichever way the scene happens to be turned.
    const planeThrough = (point) => this.dragPlane.setFromNormalAndCoplanarPoint(
      this.camera.getWorldDirection(new THREE.Vector3()).negate(), point,
    );

    element.addEventListener('pointerdown', (event) => {
      aim(event);
      pressedAt = { x: event.clientX, y: event.clientY };
      const onHandle = this.handle.visible && this.raycaster.intersectObject(this.handle).length > 0;
      const onPart = onHandle ? [] : this.raycaster.intersectObjects(visibleParts(), false);
      if (onHandle) {
        mode = 'tip';
        planeThrough(this.handle.position);
      } else if (onPart.length) {
        const mesh = onPart[0].object;
        mode = 'part';
        heldPart = mesh.userData.partId;
        planeThrough(onPart[0].point);
        grabOffset = mesh.position.clone().sub(this.world.worldToLocal(onPart[0].point.clone()));
        if (this.onPartPicked) this.onPartPicked(heldPart);
      } else {
        return; // empty space: OrbitControls has it
      }
      this.controls.enabled = false;
      element.setPointerCapture(event.pointerId);
      element.style.cursor = 'grabbing';
    });

    element.addEventListener('pointermove', (event) => {
      aim(event);
      if (!mode) {
        const over = (this.handle.visible && this.raycaster.intersectObject(this.handle).length)
          || this.raycaster.intersectObjects(visibleParts(), false).length;
        element.style.cursor = over ? 'grab' : '';
        return;
      }
      // A part only starts moving once the pointer really moves, so a click (or the first half of a
      // double-click) never nudges it.
      if (mode === 'part' && pressedAt && Math.hypot(event.clientX - pressedAt.x, event.clientY - pressedAt.y) < 4) {
        return;
      }
      pressedAt = null;
      const hit = new THREE.Vector3();
      if (!this.raycaster.ray.intersectPlane(this.dragPlane, hit)) return;
      if (mode === 'tip') {
        if (this.onDrag) this.onDrag(this.toModel(hit));
      } else if (heldPart) {
        const where = this.world.worldToLocal(hit).add(grabOffset);
        const mesh = this.partMeshes.get(heldPart);
        if (mesh) mesh.position.copy(where);
        if (this.onPartMoved) this.onPartMoved(heldPart, where.toArray());
      }
    });

    const release = (event) => {
      if (!mode) return;
      mode = null;
      heldPart = null;
      this.controls.enabled = true;
      element.style.cursor = '';
      if (element.hasPointerCapture?.(event.pointerId)) element.releasePointerCapture(event.pointerId);
    };
    element.addEventListener('pointerup', release);
    element.addEventListener('pointercancel', release);

    element.addEventListener('dblclick', (event) => {
      aim(event);
      const hits = this.raycaster.intersectObjects(visibleParts(), false);
      if (!hits.length || !this.onPartTarget) return;
      const hit = hits[0];
      // The face's normal, turned into the arm's frame, says which way is "away from the surface".
      const normal = hit.face.normal.clone().transformDirection(hit.object.matrixWorld);
      const inModel = normal.clone().transformDirection(this.world.matrixWorld.clone().invert());
      this.onPartTarget(this.toModel(hit.point), inModel.toArray());
    });
  }
}
