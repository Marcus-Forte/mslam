import {
  BufferGeometry,
  Group,
  Line,
  LineBasicMaterial,
  PerspectiveCamera,
  Points,
  PointsMaterial,
  Vector3,
} from "three";

type Measurement = {
  id: number;
  start: Vector3;
  end: Vector3;
  distance: number;
  midpoint: Vector3;
  label: HTMLDivElement;
  element: HTMLElement;
  markers: Points;
  line: Line;
};

const MARKER_SIZE = 0.08;
const COLOR_DEFAULT = "#1b2f38";
const COLOR_SELECTED = "#e57b45";
const MARKER_COLOR = "#d63030";

function createMarkerMaterial(): PointsMaterial {
  const mat = new PointsMaterial({ color: MARKER_COLOR, size: MARKER_SIZE, sizeAttenuation: true });
  mat.onBeforeCompile = (shader) => {
    shader.fragmentShader = shader.fragmentShader.replace(
      "void main() {",
      "void main() {\n  vec2 c = 2.0 * gl_PointCoord - 1.0;\n  if (dot(c, c) > 1.0) discard;",
    );
  };
  return mat;
}

export class RulerTool {
  private readonly group = new Group();
  // Shared by every measurement; never disposed per measurement.
  private readonly markerMaterial = createMarkerMaterial();
  private readonly lineMaterial = new LineBasicMaterial({ color: COLOR_DEFAULT, linewidth: 2 });
  private readonly lineMaterialSelected = new LineBasicMaterial({ color: COLOR_SELECTED, linewidth: 2 });
  private readonly projected = new Vector3();
  private pendingMarker: Points | null = null;
  private readonly measurements: Measurement[] = [];
  private pendingPoint: Vector3 | null = null;
  private enabled = false;
  private selectedId: number | null = null;
  private nextId = 0;

  constructor(
    private readonly camera: PerspectiveCamera,
    private readonly canvas: HTMLCanvasElement,
    private readonly labelsContainer: HTMLDivElement,
    private readonly listSection: HTMLElement,
    private readonly listContainer: HTMLElement,
    private readonly toggleButton: HTMLButtonElement,
    private readonly clearButton: HTMLButtonElement,
    private readonly requestRender: () => void,
  ) {
    this.toggleButton.addEventListener("click", () => this.setEnabled(!this.enabled));
    this.clearButton.addEventListener("click", () => this.clearAll());
    this.setEnabled(false);
  }

  get sceneGroup(): Group {
    return this.group;
  }

  get isEnabled(): boolean {
    return this.enabled;
  }

  addPoint(point: Vector3): void {
    if (!this.pendingPoint) {
      this.pendingPoint = point;
      this.pendingMarker = new Points(new BufferGeometry().setFromPoints([point]), this.markerMaterial);
      this.group.add(this.pendingMarker);
      this.requestRender();
      return;
    }

    const start = this.pendingPoint;
    this.clearPending();

    const distance = start.distanceTo(point);
    const id = this.nextId++;
    const markers = new Points(new BufferGeometry().setFromPoints([start, point]), this.markerMaterial);
    const line = new Line(new BufferGeometry().setFromPoints([start, point]), this.lineMaterial);
    this.group.add(markers, line);

    const measurement: Measurement = {
      id,
      start,
      end: point,
      distance,
      midpoint: start.clone().add(point).multiplyScalar(0.5),
      label: this.buildLabel(distance),
      element: this.buildListEntry(id, distance),
      markers,
      line,
    };
    this.measurements.push(measurement);
    this.listContainer.appendChild(measurement.element);
    this.labelsContainer.appendChild(measurement.label);
    this.listSection.hidden = false;
    this.selectedId = id;
    this.updateSelection();
    this.requestRender();
  }

  select(id: number | null): void {
    this.selectedId = id;
    this.updateSelection();
    this.requestRender();
  }

  remove(id: number): void {
    const index = this.measurements.findIndex((m) => m.id === id);
    if (index === -1) return;

    const m = this.measurements[index];
    this.removeScene(m);
    m.label.remove();
    m.element.remove();
    this.measurements.splice(index, 1);

    if (this.selectedId === id) {
      this.selectedId = this.measurements.length > 0
        ? this.measurements[this.measurements.length - 1].id
        : null;
    }
    this.listSection.hidden = this.measurements.length === 0;
    this.updateSelection();
    this.requestRender();
  }

  clearAll(): void {
    for (const m of this.measurements) {
      this.removeScene(m);
      m.label.remove();
      m.element.remove();
    }
    this.measurements.length = 0;
    this.clearPending();
    this.selectedId = null;
    this.listSection.hidden = true;
    this.requestRender();
  }

  /** Repositions the distance labels; call once per rendered frame. */
  updateLabels(): void {
    const width = this.canvas.clientWidth;
    const height = this.canvas.clientHeight;

    for (const m of this.measurements) {
      const projected = this.projected.copy(m.midpoint).project(this.camera);
      const isVisible = projected.z >= -1 && projected.z <= 1;

      if (!isVisible) {
        m.label.hidden = true;
        continue;
      }

      m.label.hidden = false;
      const x = (projected.x * 0.5 + 0.5) * width;
      const y = (-projected.y * 0.5 + 0.5) * height;
      m.label.style.transform = `translate(-50%, -50%) translate(${x}px, ${y - 14}px)`;
    }
  }

  private setEnabled(on: boolean): void {
    this.enabled = on;
    this.toggleButton.textContent = on ? "Ruler on" : "Ruler off";
    this.toggleButton.setAttribute("aria-pressed", String(on));
    this.canvas.classList.toggle("ruler-active", on);
  }

  private buildLabel(distance: number): HTMLDivElement {
    const el = document.createElement("div");
    el.className = "ruler-label";
    el.textContent = distance.toFixed(4);
    return el;
  }

  private buildListEntry(id: number, distance: number): HTMLElement {
    const el = document.createElement("div");
    el.className = "ruler-entry";
    el.innerHTML = `
      <span class="ruler-entry-dist">${distance.toFixed(4)} m</span>
      <button class="cloud-btn ruler-entry-remove" type="button" title="Remove">\u00d7</button>
    `;
    el.addEventListener("click", (e) => {
      if ((e.target as HTMLElement).closest(".ruler-entry-remove")) return;
      this.select(id);
    });
    el.querySelector(".ruler-entry-remove")!.addEventListener("click", () => {
      this.remove(id);
    });
    return el;
  }

  private updateSelection(): void {
    for (const m of this.measurements) {
      const selected = m.id === this.selectedId;
      m.element.classList.toggle("selected", selected);
      m.label.classList.toggle("selected", selected);
      m.line.material = selected ? this.lineMaterialSelected : this.lineMaterial;
    }
  }

  private clearPending(): void {
    if (this.pendingMarker) {
      this.group.remove(this.pendingMarker);
      this.pendingMarker.geometry.dispose();
      this.pendingMarker = null;
    }
    this.pendingPoint = null;
  }

  /** Geometries are per measurement; the materials are shared and stay alive. */
  private removeScene(m: Measurement): void {
    this.group.remove(m.markers, m.line);
    m.markers.geometry.dispose();
    m.line.geometry.dispose();
  }
}
