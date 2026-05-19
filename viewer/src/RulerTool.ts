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
      this.render();
      return;
    }

    const distance = this.pendingPoint.distanceTo(point);
    const id = this.nextId++;
    const measurement: Measurement = {
      id,
      start: this.pendingPoint,
      end: point,
      distance,
      midpoint: this.pendingPoint.clone().add(point).multiplyScalar(0.5),
      label: this.buildLabel(distance),
      element: this.buildListEntry(id, distance),
    };
    this.measurements.push(measurement);
    this.listContainer.appendChild(measurement.element);
    this.pendingPoint = null;
    this.selectedId = id;
    this.render();
  }

  select(id: number | null): void {
    this.selectedId = id;
    this.updateSelection();
    this.render();
  }

  remove(id: number): void {
    const index = this.measurements.findIndex((m) => m.id === id);
    if (index === -1) return;

    const m = this.measurements[index];
    m.label.remove();
    m.element.remove();
    this.measurements.splice(index, 1);

    if (this.selectedId === id) {
      this.selectedId = this.measurements.length > 0
        ? this.measurements[this.measurements.length - 1].id
        : null;
    }
    this.render();
  }

  clearAll(): void {
    for (const m of this.measurements) {
      m.label.remove();
      m.element.remove();
    }
    this.measurements.length = 0;
    this.pendingPoint = null;
    this.selectedId = null;
    this.render();
  }

  updateLabels(): void {
    const width = this.canvas.clientWidth;
    const height = this.canvas.clientHeight;

    for (const m of this.measurements) {
      const projected = m.midpoint.clone().project(this.camera);
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
      m.element.classList.toggle("selected", m.id === this.selectedId);
      m.label.classList.toggle("selected", m.id === this.selectedId);
    }
  }

  private disposeGroup(): void {
    for (const child of this.group.children) {
      const geo = (child as { geometry?: BufferGeometry }).geometry;
      const mat = (child as { material?: PointsMaterial | LineBasicMaterial }).material;
      geo?.dispose();
      if (Array.isArray(mat)) {
        for (const m of mat) m.dispose();
      } else {
        mat?.dispose();
      }
    }
    this.group.clear();
  }

  private render(): void {
    this.disposeGroup();
    this.labelsContainer.replaceChildren(...this.measurements.map((m) => m.label));
    this.listSection.hidden = this.measurements.length === 0;

    for (const m of this.measurements) {
      const isSelected = m.id === this.selectedId;
      const lineColor = isSelected ? COLOR_SELECTED : COLOR_DEFAULT;

      const markerGeo = new BufferGeometry().setFromPoints([m.start, m.end]);
      this.group.add(new Points(markerGeo, createMarkerMaterial()));

      const lineGeo = new BufferGeometry().setFromPoints([m.start, m.end]);
      const lineMat = new LineBasicMaterial({ color: lineColor, linewidth: 2 });
      this.group.add(new Line(lineGeo, lineMat));
    }

    if (this.pendingPoint) {
      const geo = new BufferGeometry().setFromPoints([this.pendingPoint]);
      this.group.add(new Points(geo, createMarkerMaterial()));
    }

    this.updateSelection();
    this.updateLabels();
  }
}
