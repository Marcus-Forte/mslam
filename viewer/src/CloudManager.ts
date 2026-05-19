import {
  BufferAttribute,
  BufferGeometry,
  Color,
  Float32BufferAttribute,
  Group,
  PerspectiveCamera,
  Points,
  PointsMaterial,
  Raycaster,
  Vector2,
  Vector3,
} from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";

export type ColorMode = "intensity" | "flat";

export type CloudEntry = {
  id: number;
  name: string;
  mesh: Points<BufferGeometry, PointsMaterial>;
  geometry: BufferGeometry;
  colorMode: ColorMode;
  flatColor: string;
  pointSize: number;
  visible: boolean;
  pointCount: number;
  element: HTMLElement;
};

const PRESET_COLORS = [
  "#e57b45", "#2470a8", "#4caf50", "#9c27b0",
  "#ff9800", "#00bcd4", "#f44336", "#795548",
];
const DEFAULT_POINT_SIZE = 0.04;

/**
 * Rainbow colormap matching _getRainbowColor from msensor (see client/colormap.py).
 * Maps a normalized [0,1] value to an RGB triplet in [0,1].
 */
function rainbowColor(value: number): [number, number, number] {
  const v = Math.max(0, Math.min(1, value));
  const h = v * 5.0 + 1.0;
  const i = Math.floor(h);
  let f = h - i;
  if ((i & 1) === 0) f = 1.0 - f;
  const n = 1.0 - f;

  if (i <= 1) return [n, 0, 1];
  if (i === 2) return [0, n, 1];
  if (i === 3) return [0, 1, n];
  if (i === 4) return [n, 1, 0];
  return [1, n, 0];
}

function colorizeByIntensity(geometry: BufferGeometry): void {
  const intensityAttr = geometry.getAttribute("intensity");
  if (!intensityAttr || intensityAttr.count === 0) {
    return;
  }

  const count = intensityAttr.count;
  const colors = new Float32Array(count * 3);
  let minI = Number.POSITIVE_INFINITY;
  let maxI = Number.NEGATIVE_INFINITY;

  for (let i = 0; i < count; i += 1) {
    const v = intensityAttr.getX(i);
    minI = Math.min(minI, v);
    maxI = Math.max(maxI, v);
  }

  const span = Math.max(maxI - minI, 1e-6);

  for (let i = 0; i < count; i += 1) {
    const n = (intensityAttr.getX(i) - minI) / span;
    const [r, g, b] = rainbowColor(n);
    colors[i * 3] = r;
    colors[i * 3 + 1] = g;
    colors[i * 3 + 2] = b;
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
  if (entry.colorMode === "intensity") {
    colorizeByIntensity(entry.geometry);
  } else {
    applyFlatColor(entry.geometry, entry.flatColor);
  }
  const attr = entry.geometry.getAttribute("color");
  if (attr) {
    (attr as BufferAttribute).needsUpdate = true;
  }
}

export class CloudManager {
  private readonly group = new Group();
  private readonly loader: PLYLoader;
  private readonly raycaster = new Raycaster();
  private readonly pointer = new Vector2();
  private readonly clouds: CloudEntry[] = [];
  private readonly listEl: HTMLDivElement;
  private readonly emptyEl: HTMLElement;
  private readonly countEl: HTMLElement;
  private readonly totalEl: HTMLElement;
  private nextId = 0;

  constructor(
    private readonly camera: PerspectiveCamera,
    private readonly controls: OrbitControls,
    listEl: HTMLDivElement,
    emptyEl: HTMLElement,
    countEl: HTMLElement,
    totalEl: HTMLElement,
  ) {
    this.listEl = listEl;
    this.emptyEl = emptyEl;
    this.countEl = countEl;
    this.totalEl = totalEl;
    this.raycaster.params.Points.threshold = 0.03;

    this.loader = new PLYLoader();
    this.loader.setCustomPropertyNameMapping({
      intensity: ['intensity'],
    });
  }

  get sceneGroup(): Group {
    return this.group;
  }

  add(file: File): void {
    file.arrayBuffer().then((buffer) => {
      const geometry = this.loader.parse(buffer);
      colorizeByIntensity(geometry);

      const id = this.nextId++;
      const pointSize = DEFAULT_POINT_SIZE;
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
      this.group.add(mesh);

      const entry: CloudEntry = {
        id,
        name: file.name,
        mesh,
        geometry,
        colorMode: "intensity",
        flatColor: PRESET_COLORS[id % PRESET_COLORS.length],
        pointSize,
        visible: true,
        pointCount: geometry.getAttribute("position").count,
        element: document.createElement("div"),
      };

      entry.element = this.createCloudElement(entry);
      this.listEl.appendChild(entry.element);
      this.clouds.push(entry);
      this.updateStats();
    }).catch((error: unknown) => {
      console.error(error);
      window.alert(`Failed to load ${file.name}. Check that the file is valid PLY.`);
    });
  }

  remove(id: number): void {
    const index = this.clouds.findIndex((c) => c.id === id);
    if (index === -1) {
      return;
    }

    const entry = this.clouds[index];
    this.group.remove(entry.mesh);
    entry.geometry.dispose();
    entry.mesh.material.dispose();
    entry.element.remove();
    this.clouds.splice(index, 1);
    this.updateStats();
  }

  pickPoint(event: MouseEvent, canvas: HTMLCanvasElement): Vector3 | null {
    const rect = canvas.getBoundingClientRect();
    this.pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    this.pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    this.raycaster.setFromCamera(this.pointer, this.camera);

    let best: { point: Vector3; distance: number } | null = null;
    for (const cloud of this.clouds) {
      if (!cloud.visible) {
        continue;
      }

      const hits = this.raycaster.intersectObject(cloud.mesh);
      if (hits.length === 0 || hits[0].index === undefined) {
        continue;
      }

      const positions = cloud.geometry.getAttribute("position");
      if (!positions) {
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

  private updateStats(): void {
    this.countEl.textContent = String(this.clouds.length);
    const total = this.clouds.reduce((sum, c) => sum + c.pointCount, 0);
    this.totalEl.textContent = new Intl.NumberFormat().format(total);
    this.emptyEl.hidden = this.clouds.length > 0;
    this.updateRaycasterThreshold();
  }

  private updateRaycasterThreshold(): void {
    const sizes = this.clouds.filter((c) => c.visible).map((c) => c.pointSize);
    const max = sizes.length > 0 ? Math.max(...sizes) : DEFAULT_POINT_SIZE;
    this.raycaster.params.Points.threshold = Math.max(max * 1.5, 0.02);
  }

  private focusOnCloud(entry: CloudEntry): void {
    entry.geometry.computeBoundingSphere();
    const sphere = entry.geometry.boundingSphere;
    if (!sphere) {
      return;
    }

    const center = sphere.center.clone();
    const radius = Math.max(sphere.radius, 0.5);
    const framingDistance = Math.max(
      radius / Math.tan((this.camera.fov * Math.PI) / 360) * 1.15,
      1,
    );
    const viewDir = new Vector3(1, -1, 0.75).normalize();

    this.controls.target.copy(center);
    this.camera.position.copy(center).addScaledVector(viewDir, framingDistance);
    this.camera.near = Math.max(radius / 100, 0.01);
    this.camera.far = Math.max(radius * 50, 100);
    this.camera.updateProjectionMatrix();
    this.controls.update();
  }

  private createCloudElement(entry: CloudEntry): HTMLElement {
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
            <option value="intensity"${entry.colorMode === "intensity" ? " selected" : ""}>Intensity</option>
            <option value="flat"${entry.colorMode === "flat" ? " selected" : ""}>Flat color</option>
          </select>
          <input type="color" class="cloud-color-picker" value="${entry.flatColor}" />
        </div>
        <div class="cloud-option-row">
          <span class="cloud-option-label">Size</span>
          <input type="range" class="cloud-size-slider" min="0.005" max="0.2" step="0.005" value="${entry.pointSize}" />
          <output class="cloud-size-value">${entry.pointSize.toFixed(3)}</output>
        </div>
      </div>
    `;

    const checkbox = el.querySelector<HTMLInputElement>('input[type="checkbox"]')!;
    checkbox.addEventListener("change", () => {
      entry.visible = checkbox.checked;
      entry.mesh.visible = checkbox.checked;
      this.updateRaycasterThreshold();
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
      this.updateRaycasterThreshold();
    });

    el.querySelector(".cloud-focus")!.addEventListener("click", () => {
      this.focusOnCloud(entry);
    });

    el.querySelector(".cloud-remove")!.addEventListener("click", () => {
      this.remove(entry.id);
    });

    return el;
  }
}
