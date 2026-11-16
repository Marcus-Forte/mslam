import {
  BufferAttribute,
  BufferGeometry,
  Group,
  PerspectiveCamera,
  Points,
  PointsMaterial,
  Raycaster,
  Sphere,
  Vector2,
  Vector3,
} from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { PlyLoaderClient } from "./PlyLoaderClient";
import { PointPicker } from "./pick";

export type ColorMode = "intensity" | "flat";

export type CloudEntry = {
  id: number;
  name: string;
  mesh: Points<BufferGeometry, PointsMaterial>;
  geometry: BufferGeometry;
  colorMode: ColorMode;
  /** True when the baked intensity colours exist (the file had an intensity property). */
  hasIntensity: boolean;
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

function applyCloudColor(entry: CloudEntry): void {
  const material = entry.mesh.material;
  const useVertexColors = entry.colorMode === "intensity" && entry.hasIntensity;
  if (useVertexColors) {
    material.color.setRGB(1, 1, 1);
  } else {
    material.color.set(entry.flatColor);
  }
  // Toggling vertexColors changes the program; a colour change is only a uniform.
  if (material.vertexColors !== useVertexColors) {
    material.vertexColors = useVertexColors;
    material.needsUpdate = true;
  }
}

function boundingSphereOf(geometry: BufferGeometry): Sphere | null {
  if (geometry.boundingSphere === null) geometry.computeBoundingSphere();
  return geometry.boundingSphere;
}

export class CloudManager {
  private readonly group = new Group();
  private readonly loader = new PlyLoaderClient();
  private readonly raycaster = new Raycaster();
  private readonly pointer = new Vector2();
  private readonly picker = new PointPicker();
  private readonly focusDir = new Vector3(1, -1, 0.75).normalize();
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
    private readonly requestRender: () => void,
  ) {
    this.listEl = listEl;
    this.emptyEl = emptyEl;
    this.countEl = countEl;
    this.totalEl = totalEl;
    this.raycaster.params.Points.threshold = 0.03;
  }

  get sceneGroup(): Group {
    return this.group;
  }

  add(file: File): void {
    this.loader.load(file).then(({ positions, colors }) => {
      const geometry = new BufferGeometry();
      geometry.setAttribute("position", new BufferAttribute(positions, 3));
      if (colors) {
        geometry.setAttribute("color", new BufferAttribute(colors, 3, true));
      }

      const id = this.nextId++;
      const pointSize = DEFAULT_POINT_SIZE;
      const hasIntensity = colors !== null;
      const material = new PointsMaterial({
        size: pointSize,
        vertexColors: hasIntensity,
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
        colorMode: hasIntensity ? "intensity" : "flat",
        hasIntensity,
        flatColor: PRESET_COLORS[id % PRESET_COLORS.length],
        pointSize,
        visible: true,
        pointCount: positions.length / 3,
        element: document.createElement("div"),
      };
      applyCloudColor(entry);

      entry.element = this.createCloudElement(entry);
      this.listEl.appendChild(entry.element);
      this.clouds.push(entry);
      this.updateStats();
      this.requestRender();
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
    this.requestRender();
  }

  /**
   * Returns the point nearest the camera among those within the raycaster's
   * point threshold of the pick ray (same rule as Raycaster on Points), or null.
   */
  pickPoint(event: MouseEvent, canvas: HTMLCanvasElement): Vector3 | null {
    const rect = canvas.getBoundingClientRect();
    this.pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    this.pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    this.raycaster.setFromCamera(this.pointer, this.camera);
    return this.picker.pick(this.raycaster.ray, this.raycaster.params.Points.threshold, this.clouds);
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
    const sphere = boundingSphereOf(entry.geometry);
    if (!sphere) {
      return;
    }

    const radius = Math.max(sphere.radius, 0.5);
    const framingDistance = Math.max(
      radius / Math.tan((this.camera.fov * Math.PI) / 360) * 1.15,
      1,
    );

    this.controls.target.copy(sphere.center);
    this.camera.position.copy(sphere.center).addScaledVector(this.focusDir, framingDistance);
    this.camera.near = Math.max(radius / 100, 0.01);
    this.camera.far = Math.max(radius * 50, 100);
    this.camera.updateProjectionMatrix();
    this.controls.update();
    this.requestRender();
  }

  private createCloudElement(entry: CloudEntry): HTMLElement {
    const el = document.createElement("div");
    el.className = "cloud-entry";
    el.innerHTML = `
      <div class="cloud-header">
        <label class="cloud-visibility">
          <input type="checkbox" ${entry.visible ? "checked" : ""} />
        </label>
        <span class="cloud-name"></span>
        <span class="cloud-points">${new Intl.NumberFormat().format(entry.pointCount)} pts</span>
        <button class="cloud-btn cloud-focus" type="button" title="Focus camera">\u2295</button>
        <button class="cloud-btn cloud-remove" type="button" title="Remove">\u00d7</button>
      </div>
      <div class="cloud-options">
        <div class="cloud-option-row">
          <span class="cloud-option-label">Color</span>
          <select class="cloud-color-mode">
            <option value="intensity"${entry.colorMode === "intensity" ? " selected" : ""}${entry.hasIntensity ? "" : " disabled"}>Intensity</option>
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

    // File names are untrusted: set them as text, never as markup.
    const nameEl = el.querySelector<HTMLElement>(".cloud-name")!;
    nameEl.textContent = entry.name;
    nameEl.title = entry.name;

    const checkbox = el.querySelector<HTMLInputElement>('input[type="checkbox"]')!;
    checkbox.addEventListener("change", () => {
      entry.visible = checkbox.checked;
      entry.mesh.visible = checkbox.checked;
      this.updateRaycasterThreshold();
      this.requestRender();
    });

    const colorModeSelect = el.querySelector<HTMLSelectElement>(".cloud-color-mode")!;
    const colorPicker = el.querySelector<HTMLInputElement>(".cloud-color-picker")!;
    colorPicker.style.display = entry.colorMode === "flat" ? "" : "none";

    colorModeSelect.addEventListener("change", () => {
      entry.colorMode = colorModeSelect.value as ColorMode;
      colorPicker.style.display = entry.colorMode === "flat" ? "" : "none";
      applyCloudColor(entry);
      this.requestRender();
    });

    colorPicker.addEventListener("input", () => {
      entry.flatColor = colorPicker.value;
      if (entry.colorMode === "flat") {
        applyCloudColor(entry);
        this.requestRender();
      }
    });

    const sizeSlider = el.querySelector<HTMLInputElement>(".cloud-size-slider")!;
    const sizeOutput = el.querySelector<HTMLOutputElement>(".cloud-size-value")!;
    sizeSlider.addEventListener("input", () => {
      const size = Number.parseFloat(sizeSlider.value);
      entry.pointSize = size;
      entry.mesh.material.size = size;
      sizeOutput.textContent = size.toFixed(3);
      this.updateRaycasterThreshold();
      this.requestRender();
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
