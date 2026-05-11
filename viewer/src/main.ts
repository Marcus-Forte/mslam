import "./styles.css";

import {
  AxesHelper,
  BufferAttribute,
  BufferGeometry,
  Color,
  DepthTexture,
  Float32BufferAttribute,
  GridHelper,
  Group,
  Line,
  LineBasicMaterial,
  Mesh,
  NearestFilter,
  OrthographicCamera,
  PerspectiveCamera,
  PlaneGeometry,
  Points,
  PointsMaterial,
  Raycaster,
  Scene,
  ShaderMaterial,
  SRGBColorSpace,
  UnsignedIntType,
  Vector2,
  Vector3,
  WebGLRenderer,
  WebGLRenderTarget,
} from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";
import { edlFragmentShader, edlVertexShader } from "./edlShader";

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
        Load one or more <span>.ply</span> point clouds and inspect them in the browser.
      </p>

      <label class="dropzone" for="ply-input" id="dropzone">
        <input id="ply-input" type="file" accept=".ply" multiple />
        <strong>Choose PLY files</strong>
        <span>or drag them here</span>
      </label>

      <div class="cloud-list-section">
        <p class="section-label">Loaded clouds</p>
        <div class="cloud-list" id="cloud-list">
          <p class="cloud-empty" id="cloud-empty">No clouds loaded</p>
        </div>
      </div>

      <div class="controls-panel">
        <div class="tool-row">
          <button id="edl-toggle" class="tool-button" type="button" aria-pressed="true">
            EDL on
          </button>
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
          <dt>Clouds</dt>
          <dd id="cloud-count">0</dd>
        </div>
        <div>
          <dt>Total points</dt>
          <dd id="total-points">0</dd>
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
const cloudList = requireElement<HTMLDivElement>("#cloud-list");
const cloudEmptyLabel = requireElement<HTMLElement>("#cloud-empty");
const cloudCountLabel = requireElement<HTMLElement>("#cloud-count");
const totalPointsLabel = requireElement<HTMLElement>("#total-points");
const edlToggle = requireElement<HTMLButtonElement>("#edl-toggle");
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

// ── EDL post-processing ──

const edlRenderTarget = new WebGLRenderTarget(1, 1, {
  minFilter: NearestFilter,
  magFilter: NearestFilter,
});
edlRenderTarget.depthTexture = new DepthTexture(1, 1, UnsignedIntType);

const edlMaterial = new ShaderMaterial({
  uniforms: {
    tColor: { value: edlRenderTarget.texture },
    tDepth: { value: edlRenderTarget.depthTexture },
    uScreenSize: { value: new Vector2(1, 1) },
    uStrength: { value: 1.0 },
    uRadius: { value: 1.5 },
    uNear: { value: camera.near },
    uFar: { value: camera.far },
  },
  vertexShader: edlVertexShader,
  fragmentShader: edlFragmentShader,
  depthWrite: false,
  depthTest: false,
});

const edlScene = new Scene();
const edlCamera = new OrthographicCamera(-1, 1, 1, -1, 0, 1);
const edlQuad = new Mesh(new PlaneGeometry(2, 2), edlMaterial);
edlScene.add(edlQuad);

let edlEnabled = true;

type RulerMeasurement = {
  start: Vector3;
  end: Vector3;
  distance: number;
  midpoint: Vector3;
  label: HTMLDivElement;
};

type ColorMode = "height" | "flat";

type CloudEntry = {
  id: number;
  name: string;
  mesh: Points<BufferGeometry, PointsMaterial>;
  geometry: BufferGeometry;
  colorMode: ColorMode;
  flatColor: string;
  pointSize: number;
  opacity: number;
  visible: boolean;
  pointCount: number;
  element: HTMLElement;
};

const PRESET_COLORS = [
  "#e57b45", "#2470a8", "#4caf50", "#9c27b0",
  "#ff9800", "#00bcd4", "#f44336", "#795548",
];
const RULER_MARKER_SIZE = 0.025;
const DEFAULT_POINT_SIZE = 0.04;

let nextCloudId = 0;
const clouds: CloudEntry[] = [];
let rulerEnabled = false;
let pointerDownPosition: { x: number; y: number } | null = null;
let pendingRulerPoint: Vector3 | null = null;
const rulerMeasurements: RulerMeasurement[] = [];
const pressedKeys = new Set<string>();
let lastFrameTime = performance.now();

raycaster.params.Points.threshold = 0.03;

function updateStats(): void {
  cloudCountLabel.textContent = String(clouds.length);
  const total = clouds.reduce((sum, c) => sum + c.pointCount, 0);
  totalPointsLabel.textContent = new Intl.NumberFormat().format(total);
  cloudEmptyLabel.hidden = clouds.length > 0;
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

function applyFlatColor(geometry: BufferGeometry, hex: string): void {
  const color = new Color(hex);
  const count = geometry.getAttribute("position").count;
  const colors = new Float32Array(count * 3);
  for (let i = 0; i < count; i += 1) {
    colors[i * 3] = color.r;
    colors[i * 3 + 1] = color.g;
    colors[i * 3 + 2] = color.b;
  }
  geometry.setAttribute("color", new Float32BufferAttribute(colors, 3));
}

function applyCloudColor(entry: CloudEntry): void {
  if (entry.colorMode === "height") {
    colorizeByHeight(entry.geometry);
  } else {
    applyFlatColor(entry.geometry, entry.flatColor);
  }
  const colorAttr = entry.geometry.getAttribute("color");
  if (colorAttr) {
    (colorAttr as BufferAttribute).needsUpdate = true;
  }
}

function updateRaycasterThreshold(): void {
  const visibleSizes = clouds.filter((c) => c.visible).map((c) => c.pointSize);
  const maxSize = visibleSizes.length > 0 ? Math.max(...visibleSizes) : DEFAULT_POINT_SIZE;
  raycaster.params.Points.threshold = Math.max(maxSize * 1.5, 0.02);
}

function focusOnCloud(entry: CloudEntry): void {
  entry.geometry.computeBoundingSphere();
  const sphere = entry.geometry.boundingSphere;
  if (!sphere) {
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
}

function createCloudElement(entry: CloudEntry): HTMLElement {
  const el = document.createElement("div");
  el.className = "cloud-entry";
  el.innerHTML = `
    <div class="cloud-header">
      <label class="cloud-visibility">
        <input type="checkbox" ${entry.visible ? "checked" : ""} />
      </label>
      <span class="cloud-name" title="${entry.name}">${entry.name}</span>
      <span class="cloud-points">${new Intl.NumberFormat().format(entry.pointCount)} pts</span>
      <button class="cloud-btn cloud-focus" type="button" title="Focus camera">\u2295</button>
      <button class="cloud-btn cloud-remove" type="button" title="Remove">\u00d7</button>
    </div>
    <div class="cloud-options">
      <div class="cloud-option-row">
        <span class="cloud-option-label">Color</span>
        <select class="cloud-color-mode">
          <option value="height"${entry.colorMode === "height" ? " selected" : ""}>Height gradient</option>
          <option value="flat"${entry.colorMode === "flat" ? " selected" : ""}>Flat color</option>
        </select>
        <input type="color" class="cloud-color-picker" value="${entry.flatColor}" />
      </div>
      <div class="cloud-option-row">
        <span class="cloud-option-label">Size</span>
        <input type="range" class="cloud-size-slider" min="0.005" max="0.2" step="0.005" value="${entry.pointSize}" />
        <output class="cloud-size-value">${entry.pointSize.toFixed(3)}</output>
      </div>
      <div class="cloud-option-row">
        <span class="cloud-option-label">Alpha</span>
        <input type="range" class="cloud-opacity-slider" min="0.05" max="1" step="0.05" value="${entry.opacity}" />
        <output class="cloud-opacity-value">${Math.round(entry.opacity * 100)}%</output>
      </div>
    </div>
  `;

  const checkbox = el.querySelector<HTMLInputElement>('input[type="checkbox"]')!;
  checkbox.addEventListener("change", () => {
    entry.visible = checkbox.checked;
    entry.mesh.visible = checkbox.checked;
    updateRaycasterThreshold();
  });

  const colorModeSelect = el.querySelector<HTMLSelectElement>(".cloud-color-mode")!;
  const colorPicker = el.querySelector<HTMLInputElement>(".cloud-color-picker")!;
  colorPicker.style.display = entry.colorMode === "flat" ? "" : "none";

  colorModeSelect.addEventListener("change", () => {
    entry.colorMode = colorModeSelect.value as ColorMode;
    colorPicker.style.display = entry.colorMode === "flat" ? "" : "none";
    applyCloudColor(entry);
  });

  colorPicker.addEventListener("input", () => {
    entry.flatColor = colorPicker.value;
    if (entry.colorMode === "flat") {
      applyCloudColor(entry);
    }
  });

  const sizeSlider = el.querySelector<HTMLInputElement>(".cloud-size-slider")!;
  const sizeOutput = el.querySelector<HTMLOutputElement>(".cloud-size-value")!;
  sizeSlider.addEventListener("input", () => {
    const size = Number.parseFloat(sizeSlider.value);
    entry.pointSize = size;
    entry.mesh.material.size = size;
    entry.mesh.material.needsUpdate = true;
    sizeOutput.textContent = size.toFixed(3);
    updateRaycasterThreshold();
  });

  const opacitySlider = el.querySelector<HTMLInputElement>(".cloud-opacity-slider")!;
  const opacityOutput = el.querySelector<HTMLOutputElement>(".cloud-opacity-value")!;
  opacitySlider.addEventListener("input", () => {
    const alpha = Number.parseFloat(opacitySlider.value);
    entry.opacity = alpha;
    entry.mesh.material.opacity = alpha;
    entry.mesh.material.alphaHash = alpha < 1;
    entry.mesh.material.needsUpdate = true;
    opacityOutput.textContent = `${Math.round(alpha * 100)}%`;
  });

  el.querySelector(".cloud-focus")!.addEventListener("click", () => {
    focusOnCloud(entry);
  });

  el.querySelector(".cloud-remove")!.addEventListener("click", () => {
    removeCloud(entry.id);
  });

  return el;
}

function addCloud(file: File): void {
  file.arrayBuffer().then((buffer) => {
    const geometry = loader.parse(buffer);
    colorizeByHeight(geometry);

    const id = nextCloudId;
    nextCloudId += 1;

    const pointSize = DEFAULT_POINT_SIZE;
    const opacity = 1;
    const material = new PointsMaterial({
      size: pointSize,
      vertexColors: true,
      sizeAttenuation: true,
    });
    material.onBeforeCompile = (shader) => {
      shader.fragmentShader = shader.fragmentShader.replace(
        "void main() {",
        "void main() {\n  vec2 c = 2.0 * gl_PointCoord - 1.0;\n  if (dot(c, c) > 1.0) discard;",
      );
    };

    const mesh = new Points(geometry, material);
    pointCloudGroup.add(mesh);

    const entry: CloudEntry = {
      id,
      name: file.name,
      mesh,
      geometry,
      colorMode: "height",
      flatColor: PRESET_COLORS[id % PRESET_COLORS.length],
      pointSize,
      opacity,
      visible: true,
      pointCount: geometry.getAttribute("position").count,
      element: document.createElement("div"),
    };

    entry.element = createCloudElement(entry);
    cloudList.appendChild(entry.element);
    clouds.push(entry);
    updateStats();
    updateRaycasterThreshold();
  }).catch((error: unknown) => {
    console.error(error);
    window.alert(`Failed to load ${file.name}. Check that the file is valid PLY.`);
  });
}

function removeCloud(id: number): void {
  const index = clouds.findIndex((c) => c.id === id);
  if (index === -1) {
    return;
  }

  const entry = clouds[index];
  pointCloudGroup.remove(entry.mesh);
  entry.geometry.dispose();
  entry.mesh.material.dispose();
  entry.element.remove();
  clouds.splice(index, 1);
  updateStats();
  updateRaycasterThreshold();
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
      size: RULER_MARKER_SIZE,
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
      size: RULER_MARKER_SIZE,
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
  const rect = canvas.getBoundingClientRect();
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

  raycaster.setFromCamera(pointer, camera);

  let best: { point: Vector3; distance: number } | null = null;
  for (const cloud of clouds) {
    if (!cloud.visible) {
      continue;
    }

    const hits = raycaster.intersectObject(cloud.mesh);
    if (hits.length === 0 || hits[0].index === undefined) {
      continue;
    }

    const positions = cloud.geometry.getAttribute("position");
    if (!(positions instanceof BufferAttribute)) {
      continue;
    }

    const point = new Vector3(
      positions.getX(hits[0].index),
      positions.getY(hits[0].index),
      positions.getZ(hits[0].index),
    ).applyMatrix4(cloud.mesh.matrixWorld);

    if (!best || hits[0].distance < best.distance) {
      best = { point, distance: hits[0].distance };
    }
  }

  return best?.point ?? null;
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

function handleFileSelection(fileList: FileList | null): void {
  if (!fileList || fileList.length === 0) {
    return;
  }

  for (const file of fileList) {
    addCloud(file);
  }
}

input.addEventListener("change", () => {
  handleFileSelection(input.files);
});

edlToggle.addEventListener("click", () => {
  edlEnabled = !edlEnabled;
  edlToggle.textContent = edlEnabled ? "EDL on" : "EDL off";
  edlToggle.setAttribute("aria-pressed", String(edlEnabled));
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
  const dpr = renderer.getPixelRatio();
  const w = Math.floor(clientWidth * dpr);
  const h = Math.floor(clientHeight * dpr);
  camera.aspect = clientWidth / Math.max(clientHeight, 1);
  camera.updateProjectionMatrix();
  renderer.setSize(clientWidth, clientHeight, false);
  edlRenderTarget.setSize(w, h);
  edlMaterial.uniforms.uScreenSize.value.set(w, h);
  updateRulerLabels();
}

window.addEventListener("resize", resize);
resize();
updateStats();
setRulerDistance(null);
setRulerEnabled(false);

function render(): void {
  const now = performance.now();
  const deltaSeconds = Math.min((now - lastFrameTime) / 1000, 0.1);
  lastFrameTime = now;

  updateKeyboardNavigation(deltaSeconds);
  controls.update();
  updateRulerLabels();

  if (edlEnabled) {
    edlMaterial.uniforms.uNear.value = camera.near;
    edlMaterial.uniforms.uFar.value = camera.far;
    renderer.setRenderTarget(edlRenderTarget);
    renderer.render(scene, camera);
    renderer.setRenderTarget(null);
    renderer.render(edlScene, edlCamera);
  } else {
    renderer.render(scene, camera);
  }

  window.requestAnimationFrame(render);
}

render();