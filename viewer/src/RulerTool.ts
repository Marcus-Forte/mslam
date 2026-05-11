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
  start: Vector3;
  end: Vector3;
  distance: number;
  midpoint: Vector3;
  label: HTMLDivElement;
};

const MARKER_SIZE = 0.025;

export class RulerTool {
  private readonly group = new Group();
  private readonly measurements: Measurement[] = [];
  private pendingPoint: Vector3 | null = null;
  private enabled = false;

  constructor(
    private readonly camera: PerspectiveCamera,
    private readonly canvas: HTMLCanvasElement,
    private readonly labelsContainer: HTMLDivElement,
    private readonly distanceLabel: HTMLElement,
    private readonly toggleButton: HTMLButtonElement,
    private readonly clearButton: HTMLButtonElement,
  ) {
    this.toggleButton.addEventListener("click", () => this.setEnabled(!this.enabled));
    this.clearButton.addEventListener("click", () => this.clear());
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
    this.measurements.push({
      start: this.pendingPoint,
      end: point,
      distance,
      midpoint: this.pendingPoint.clone().add(point).multiplyScalar(0.5),
      label: this.buildLabel(distance),
    });
    this.pendingPoint = null;
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

  private clear(): void {
    this.pendingPoint = null;
    this.measurements.length = 0;
    this.render();
  }

  private buildLabel(distance: number): HTMLDivElement {
    const el = document.createElement("div");
    el.className = "ruler-label";
    el.textContent = distance.toFixed(4);
    return el;
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

    for (const m of this.measurements) {
      const markerGeo = new BufferGeometry().setFromPoints([m.start, m.end]);
      const markerMat = new PointsMaterial({ color: "#b84122", size: MARKER_SIZE, sizeAttenuation: true });
      this.group.add(new Points(markerGeo, markerMat));

      const lineGeo = new BufferGeometry().setFromPoints([m.start, m.end]);
      const lineMat = new LineBasicMaterial({ color: "#1b2f38" });
      this.group.add(new Line(lineGeo, lineMat));
    }

    if (this.pendingPoint) {
      const geo = new BufferGeometry().setFromPoints([this.pendingPoint]);
      const mat = new PointsMaterial({ color: "#b84122", size: MARKER_SIZE, sizeAttenuation: true });
      this.group.add(new Points(geo, mat));
    }

    this.clearButton.disabled = this.measurements.length === 0 && this.pendingPoint === null;

    if (this.measurements.length === 0) {
      this.distanceLabel.textContent = "-";
    } else {
      const last = this.measurements[this.measurements.length - 1].distance;
      const suffix = this.measurements.length > 1 ? ` (${this.measurements.length})` : "";
      this.distanceLabel.textContent = `${last.toFixed(4)}${suffix}`;
    }

    this.updateLabels();
  }
}
