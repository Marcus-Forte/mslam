import "./styles.css";

import {
  AxesHelper,
  Box3,
  BufferAttribute,
  BufferGeometry,
  Color,
  Float32BufferAttribute,
  GridHelper,
  Group,
  Line,
  LineBasicMaterial,
  PerspectiveCamera,
  Points,
  PointsMaterial,
  Raycaster,
  Scene,
  SRGBColorSpace,
  Vector2,
  Vector3,
  WebGLRenderer,
} from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";

function requireElement<T extends Element>(
  selector: string,
  parent: ParentNode = document,
): T {
  const element = parent.querySelector<T>(selector);
  if (!element) {
    throw new Error(`Required element not found: ${selector}`);
  }

  return element;
}

const app = requireElement<HTMLDivElement>("#app");

app.innerHTML = `
  <div class="shell">
    <section class="panel">
      <p class="eyebrow">MSLAM Viewer</p>
      <h1>PLY Loader</h1>
      <p class="lede">
        Load a local <span>.ply</span> point cloud and inspect it directly in the browser.
      </p>

      <label class="dropzone" for="ply-input" id="dropzone">
        <input id="ply-input" type="file" accept=".ply" />
        <strong>Choose a PLY file</strong>
        <span>or drag one here</span>
      </label>

      <div class="controls-panel">
        <label class="control" for="point-size">
          <span>Point size</span>
          <div class="control-row">
            <input id="point-size" type="range" min="0.01" max="0.2" step="0.005" value="0.12" />
            <output id="point-size-value">0.12</output>
          </div>
        </label>

        <div class="tool-row">
          <button id="ruler-toggle" class="tool-button" type="button" aria-pressed="false">
            Ruler off
          </button>
          <button id="ruler-clear" class="tool-button secondary" type="button" disabled>
            Clear
          </button>
        </div>
      </div>

      <dl class="stats">
        <div>
          <dt>File</dt>
          <dd id="file-name">None loaded</dd>
        </div>
        <div>
          <dt>Points</dt>
          <dd id="point-count">0</dd>
        </div>
        <div>
          <dt>Bounds</dt>
          <dd id="bounds">-</dd>
        </div>
        <div>
          <dt>Measure</dt>
          <dd id="ruler-distance">-</dd>
        </div>
      </dl>

      <p class="hint">
        Z is up. Drag to orbit. Scroll to zoom. Right-drag to pan. Use W A S D to move and Q E for vertical motion.
      </p>
    </section>

    <section class="viewer-frame">
      <canvas id="viewer-canvas"></canvas>
      <div id="ruler-labels" class="ruler-labels"></div>
    </section>
  </div>
`;

const canvas = requireElement<HTMLCanvasElement>("#viewer-canvas");
const input = requireElement<HTMLInputElement>("#ply-input");
const dropzone = requireElement<HTMLLabelElement>("#dropzone");
const fileNameLabel = requireElement<HTMLElement>("#file-name");
const pointCountLabel = requireElement<HTMLElement>("#point-count");
const boundsLabel = requireElement<HTMLElement>("#bounds");
const pointSizeInput = requireElement<HTMLInputElement>("#point-size");
const pointSizeValue = requireElement<HTMLOutputElement>("#point-size-value");
const rulerToggle = requireElement<HTMLButtonElement>("#ruler-toggle");
const rulerClear = requireElement<HTMLButtonElement>("#ruler-clear");
const rulerDistanceLabel = requireElement<HTMLElement>("#ruler-distance");
const rulerLabels = requireElement<HTMLDivElement>("#ruler-labels");

const backgroundColor = new Color("#f6f1e8");

const scene = new Scene();
scene.background = backgroundColor;

const camera = new PerspectiveCamera(55, 1, 0.01, 5000);
camera.up.set(0, 0, 1);
camera.position.set(1.25, -1.25, 0.95);

const renderer = new WebGLRenderer({ antialias: true, canvas });
renderer.outputColorSpace = SRGBColorSpace;
renderer.setPixelRatio(window.devicePixelRatio);

const controls = new OrbitControls(camera, canvas);
controls.enableDamping = false;
controls.target.set(0, 0, 0);

const grid = new GridHelper(10, 20, 0x9f8b6d, 0xd6c8b4);
grid.rotation.x = Math.PI * 0.5;
grid.position.z = -0.001;
scene.add(grid);

const axes = new AxesHelper(1.5);
scene.add(axes);

const pointCloudGroup = new Group();
scene.add(pointCloudGroup);

const rulerGroup = new Group();
scene.add(rulerGroup);

const loader = new PLYLoader();
const raycaster = new Raycaster();
const pointer = new Vector2();

type RulerMeasurement = {
  start: Vector3;
  end: Vector3;
  distance: number;
  midpoint: Vector3;
  label: HTMLDivElement;
};

let currentPointCloud: Points<BufferGeometry, PointsMaterial> | null = null;
let currentPointSize = Number.parseFloat(pointSizeInput.value);
let rulerEnabled = false;
let pointerDownPosition: { x: number; y: number } | null = null;
let pendingRulerPoint: Vector3 | null = null;
const rulerMeasurements: RulerMeasurement[] = [];
const pressedKeys = new Set<string>();
let lastFrameTime = performance.now();

raycaster.params.Points.threshold = Math.max(currentPointSize * 1.5, 0.02);

function setStatus(name: string, pointCount: number, bounds: Box3 | null): void {
  fileNameLabel.textContent = name;
  pointCountLabel.textContent = new Intl.NumberFormat().format(pointCount);

  if (!bounds || bounds.isEmpty()) {
    boundsLabel.textContent = "-";
    return;
  }

  const size = new Vector3();
  bounds.getSize(size);
  boundsLabel.textContent = `${size.x.toFixed(2)} x ${size.y.toFixed(2)} x ${size.z.toFixed(2)}`;
}

function setRulerDistance(distance: number | null): void {
  if (distance === null) {
    rulerDistanceLabel.textContent = "-";
    return;
  }

  const countSuffix = rulerMeasurements.length > 1 ? ` (${rulerMeasurements.length})` : "";
  rulerDistanceLabel.textContent = `${distance.toFixed(4)}${countSuffix}`;
}

function disposeGroupObjects(group: Group): void {
  for (const child of group.children) {
    const geometry = (child as { geometry?: BufferGeometry }).geometry;
    const material = (child as { material?: PointsMaterial | LineBasicMaterial }).material;
    geometry?.dispose();

    if (Array.isArray(material)) {
      for (const entry of material) {
        entry.dispose();
      }
    } else {
      material?.dispose();
    }
  }

  group.clear();
}

function colorizeByHeight(geometry: BufferGeometry): void {
  const positionAttribute = geometry.getAttribute("position");
  if (!(positionAttribute instanceof BufferAttribute)) {
    return;
  }

  const colors = new Float32Array(positionAttribute.count * 3);
  let minZ = Number.POSITIVE_INFINITY;
  let maxZ = Number.NEGATIVE_INFINITY;

  for (let index = 0; index < positionAttribute.count; index += 1) {
    const z = positionAttribute.getZ(index);
    minZ = Math.min(minZ, z);
    maxZ = Math.max(maxZ, z);
  }

  const span = Math.max(maxZ - minZ, 1e-6);

  for (let index = 0; index < positionAttribute.count; index += 1) {
    const normalized = (positionAttribute.getZ(index) - minZ) / span;
    colors[index * 3] = normalized;
    colors[index * 3 + 1] = 0.35 + 0.45 * (1 - Math.abs(normalized - 0.5) * 2);
    colors[index * 3 + 2] = 1 - normalized;
  }

  geometry.setAttribute("color", new Float32BufferAttribute(colors, 3));
}

function focusCameraOnGeometry(geometry: BufferGeometry): void {
  geometry.computeBoundingBox();
  geometry.computeBoundingSphere();

  const bounds = geometry.boundingBox;
  const sphere = geometry.boundingSphere;
  if (!bounds || !sphere) {
    return;
  }

  const center = sphere.center.clone();
  const radius = Math.max(sphere.radius, 0.5);
  const framingDistance = Math.max(
    radius / Math.tan((camera.fov * Math.PI) / 360) * 1.15,
    1,
  );
  const viewDirection = new Vector3(1, -1, 0.75).normalize();

  controls.target.copy(center);
  camera.position.copy(center).addScaledVector(viewDirection, framingDistance);
  camera.near = Math.max(radius / 100, 0.01);
  camera.far = Math.max(radius * 50, 100);
  camera.updateProjectionMatrix();
  controls.update();

  setStatus(fileNameLabel.textContent ?? "Loaded file", geometry.getAttribute("position").count, bounds);
}

function disposeCurrentPointCloud(): void {
  disposeGroupObjects(pointCloudGroup);
  currentPointCloud = null;
}

function updatePointSize(pointSize: number): void {
  currentPointSize = pointSize;
  pointSizeValue.textContent = pointSize.toFixed(3);
  raycaster.params.Points.threshold = Math.max(pointSize * 1.5, 0.02);

  if (currentPointCloud) {
    currentPointCloud.material.size = pointSize;
    currentPointCloud.material.needsUpdate = true;
  }

  renderRulers();
}

function setRulerEnabled(enabled: boolean): void {
  rulerEnabled = enabled;
  rulerToggle.textContent = enabled ? "Ruler on" : "Ruler off";
  rulerToggle.setAttribute("aria-pressed", String(enabled));
  canvas.classList.toggle("ruler-active", enabled);
}

function buildRulerLabel(distance: number): HTMLDivElement {
  const label = document.createElement("div");
  label.className = "ruler-label";
  label.textContent = distance.toFixed(4);
  return label;
}

function updateRulerLabels(): void {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;

  for (const measurement of rulerMeasurements) {
    const projected = measurement.midpoint.clone().project(camera);
    const isVisible = projected.z >= -1 && projected.z <= 1;

    if (!isVisible) {
      measurement.label.hidden = true;
      continue;
    }

    measurement.label.hidden = false;
    const x = (projected.x * 0.5 + 0.5) * width;
    const y = (-projected.y * 0.5 + 0.5) * height;
    measurement.label.style.transform = `translate(-50%, -50%) translate(${x}px, ${y - 14}px)`;
  }
}

function renderRulers(): void {
  disposeGroupObjects(rulerGroup);
  rulerLabels.replaceChildren(...rulerMeasurements.map((measurement) => measurement.label));

  for (const measurement of rulerMeasurements) {
    const markerGeometry = new BufferGeometry().setFromPoints([
      measurement.start,
      measurement.end,
    ]);
    const markerMaterial = new PointsMaterial({
      color: "#b84122",
      size: currentPointSize * 2.2,
      sizeAttenuation: true,
    });
    rulerGroup.add(new Points(markerGeometry, markerMaterial));

    const lineGeometry = new BufferGeometry().setFromPoints([
      measurement.start,
      measurement.end,
    ]);
    const lineMaterial = new LineBasicMaterial({ color: "#1b2f38" });
    rulerGroup.add(new Line(lineGeometry, lineMaterial));
  }

  if (pendingRulerPoint) {
    const pendingGeometry = new BufferGeometry().setFromPoints([pendingRulerPoint]);
    const pendingMaterial = new PointsMaterial({
      color: "#b84122",
      size: currentPointSize * 2.2,
      sizeAttenuation: true,
    });
    rulerGroup.add(new Points(pendingGeometry, pendingMaterial));
  }

  rulerClear.disabled = rulerMeasurements.length === 0 && pendingRulerPoint === null;
  if (rulerMeasurements.length === 0) {
    setRulerDistance(null);
    updateRulerLabels();
    return;
  }

  setRulerDistance(rulerMeasurements[rulerMeasurements.length - 1].distance);
  updateRulerLabels();
}

function clearRulers(): void {
  pendingRulerPoint = null;
  rulerMeasurements.length = 0;
  renderRulers();
}

function getPickedPoint(event: MouseEvent): Vector3 | null {
  if (!currentPointCloud) {
    return null;
  }

  const rect = canvas.getBoundingClientRect();
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

  raycaster.setFromCamera(pointer, camera);
  const intersections = raycaster.intersectObject(currentPointCloud);
  const closest = intersections[0];
  if (!closest || closest.index === undefined) {
    return null;
  }

  const positions = currentPointCloud.geometry.getAttribute("position");
  if (!(positions instanceof BufferAttribute)) {
    return null;
  }

  return new Vector3(
    positions.getX(closest.index),
    positions.getY(closest.index),
    positions.getZ(closest.index),
  ).applyMatrix4(currentPointCloud.matrixWorld);
}

function shouldHandleKeyboardNavigation(): boolean {
  const activeElement = document.activeElement;
  if (!activeElement) {
    return true;
  }

  return !["INPUT", "BUTTON", "SELECT", "TEXTAREA"].includes(activeElement.tagName);
}

function updateKeyboardNavigation(deltaSeconds: number): void {
  if (!shouldHandleKeyboardNavigation()) {
    return;
  }

  let forwardAxis = 0;
  let lateralAxis = 0;
  let verticalAxis = 0;

  if (pressedKeys.has("w")) {
    forwardAxis += 1;
  }
  if (pressedKeys.has("s")) {
    forwardAxis -= 1;
  }
  if (pressedKeys.has("d")) {
    lateralAxis += 1;
  }
  if (pressedKeys.has("a")) {
    lateralAxis -= 1;
  }
  if (pressedKeys.has("q")) {
    verticalAxis += 1;
  }
  if (pressedKeys.has("e")) {
    verticalAxis -= 1;
  }

  if (forwardAxis === 0 && lateralAxis === 0 && verticalAxis === 0) {
    return;
  }

  const up = new Vector3(0, 0, 1);
  const forward = controls.target.clone().sub(camera.position).setZ(0);
  if (forward.lengthSq() < 1e-8) {
    forward.set(1, 0, 0);
  } else {
    forward.normalize();
  }

  const right = forward.clone().cross(up).normalize();
  const movement = new Vector3();
  movement.addScaledVector(forward, forwardAxis);
  movement.addScaledVector(right, lateralAxis);
  movement.addScaledVector(up, verticalAxis);

  if (movement.lengthSq() < 1e-8) {
    return;
  }

  movement.normalize();
  const orbitDistance = camera.position.distanceTo(controls.target);
  const movementSpeed = Math.max(orbitDistance * 0.9, 0.25);
  movement.multiplyScalar(movementSpeed * deltaSeconds);

  camera.position.add(movement);
  controls.target.add(movement);
}

function pickPoint(event: PointerEvent): void {
  const point = getPickedPoint(event);
  if (!point) {
    return;
  }

  if (!pendingRulerPoint) {
    pendingRulerPoint = point;
    renderRulers();
    return;
  }

  const distance = pendingRulerPoint.distanceTo(point);
  rulerMeasurements.push({
    start: pendingRulerPoint,
    end: point,
    distance,
    midpoint: pendingRulerPoint.clone().add(point).multiplyScalar(0.5),
    label: buildRulerLabel(distance),
  });
  pendingRulerPoint = null;
  renderRulers();
}

function loadPlyFile(file: File): void {
  file.arrayBuffer().then((buffer) => {
    const geometry = loader.parse(buffer);
    geometry.computeVertexNormals();
    colorizeByHeight(geometry);

    const material = new PointsMaterial({
      size: currentPointSize,
      vertexColors: true,
      sizeAttenuation: true,
    });

    const points = new Points(geometry, material);
    disposeCurrentPointCloud();
    clearRulers();
    pointCloudGroup.add(points);
    currentPointCloud = points;

    fileNameLabel.textContent = file.name;
    focusCameraOnGeometry(geometry);
  }).catch((error: unknown) => {
    console.error(error);
    window.alert("Failed to load PLY file. Check that the file is valid ASCII or binary PLY.");
  });
}

function handleFileSelection(fileList: FileList | null): void {
  const file = fileList?.[0];
  if (!file) {
    return;
  }

  loadPlyFile(file);
}

input.addEventListener("change", () => {
  handleFileSelection(input.files);
});

pointSizeInput.addEventListener("input", () => {
  updatePointSize(Number.parseFloat(pointSizeInput.value));
});

rulerToggle.addEventListener("click", () => {
  setRulerEnabled(!rulerEnabled);
});

rulerClear.addEventListener("click", () => {
  clearRulers();
});

dropzone.addEventListener("dragover", (event) => {
  event.preventDefault();
  dropzone.dataset.dragging = "true";
});

dropzone.addEventListener("dragleave", () => {
  delete dropzone.dataset.dragging;
});

dropzone.addEventListener("drop", (event) => {
  event.preventDefault();
  delete dropzone.dataset.dragging;
  handleFileSelection(event.dataTransfer?.files ?? null);
});

canvas.addEventListener("pointerdown", (event) => {
  if (!rulerEnabled) {
    return;
  }

  pointerDownPosition = { x: event.clientX, y: event.clientY };
});

canvas.addEventListener("pointerup", (event) => {
  if (!rulerEnabled || !pointerDownPosition) {
    return;
  }

  const pointerTravel = Math.hypot(
    event.clientX - pointerDownPosition.x,
    event.clientY - pointerDownPosition.y,
  );
  pointerDownPosition = null;
  if (pointerTravel > 4) {
    return;
  }

  pickPoint(event);
});

canvas.addEventListener("pointerleave", () => {
  pointerDownPosition = null;
});

window.addEventListener("keydown", (event) => {
  if (event.repeat) {
    return;
  }

  const key = event.key.toLowerCase();
  if (!["w", "a", "s", "d", "q", "e"].includes(key)) {
    return;
  }

  pressedKeys.add(key);
  if (shouldHandleKeyboardNavigation()) {
    event.preventDefault();
  }
});

window.addEventListener("keyup", (event) => {
  pressedKeys.delete(event.key.toLowerCase());
});

window.addEventListener("blur", () => {
  pressedKeys.clear();
});

function resize(): void {
  const frame = canvas.parentElement;
  if (!frame) {
    return;
  }

  const { clientWidth, clientHeight } = frame;
  camera.aspect = clientWidth / Math.max(clientHeight, 1);
  camera.updateProjectionMatrix();
  renderer.setSize(clientWidth, clientHeight, false);
  updateRulerLabels();
}

window.addEventListener("resize", resize);
resize();
setStatus("None loaded", 0, null);
setRulerDistance(null);
updatePointSize(currentPointSize);
setRulerEnabled(false);

function render(): void {
  const now = performance.now();
  const deltaSeconds = Math.min((now - lastFrameTime) / 1000, 0.1);
  lastFrameTime = now;

  updateKeyboardNavigation(deltaSeconds);
  controls.update();
  updateRulerLabels();
  renderer.render(scene, camera);
  window.requestAnimationFrame(render);
}

render();