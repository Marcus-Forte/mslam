import "./styles.css";

import {
  AxesHelper,
  Color,
  GridHelper,
  PerspectiveCamera,
  Scene,
  SRGBColorSpace,
  Vector3,
  WebGLRenderer,
} from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { appLayout } from "./layout";
import { CloudManager } from "./CloudManager";
import { RulerTool } from "./RulerTool";
import { EdlPass } from "./EdlPass";

const MAX_PIXEL_RATIO = 2;

function requireElement<T extends Element>(selector: string, parent: ParentNode = document): T {
  const el = parent.querySelector<T>(selector);
  if (!el) throw new Error(`Required element not found: ${selector}`);
  return el;
}

class ViewerApp {
  private readonly scene = new Scene();
  private readonly camera: PerspectiveCamera;
  private readonly renderer: WebGLRenderer;
  private readonly controls: OrbitControls;
  private readonly cloudManager: CloudManager;
  private readonly ruler: RulerTool;
  private readonly edl: EdlPass;
  private readonly canvas: HTMLCanvasElement;
  private readonly pressedKeys = new Set<string>();
  private frameHandle: number | null = null;
  /** Timestamp of the previous frame while keys keep the loop running; null after idle. */
  private lastNavTime: number | null = null;
  private readonly navUp = new Vector3(0, 0, 1);
  private readonly navForward = new Vector3();
  private readonly navRight = new Vector3();
  private readonly navMove = new Vector3();
  private pointerDownPos: { x: number; y: number } | null = null;

  constructor() {
    const app = requireElement<HTMLDivElement>("#app");
    app.innerHTML = appLayout;

    this.canvas = requireElement<HTMLCanvasElement>("#viewer-canvas");

    this.scene.background = new Color("#f6f1e8");

    this.camera = new PerspectiveCamera(55, 1, 0.01, 5000);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(1.25, -1.25, 0.95);

    // No MSAA: with EDL on (the default) the scene goes into a non-multisampled
    // render target and only a full-screen quad reaches the canvas, so canvas MSAA
    // would be paid for nothing; points are round discs cut by `discard`, which
    // MSAA does not smooth either.
    this.renderer = new WebGLRenderer({
      antialias: false,
      powerPreference: "high-performance",
      canvas: this.canvas,
    });
    this.renderer.outputColorSpace = SRGBColorSpace;
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, MAX_PIXEL_RATIO));

    this.controls = new OrbitControls(this.camera, this.canvas);
    this.controls.enableDamping = false;
    this.controls.target.set(0, 0, 0);
    this.controls.addEventListener("change", this.requestRender);

    const grid = new GridHelper(10, 20, 0x9f8b6d, 0xd6c8b4);
    grid.rotation.x = Math.PI * 0.5;
    grid.position.z = -0.001;
    this.scene.add(grid);
    this.scene.add(new AxesHelper(1.5));

    this.cloudManager = new CloudManager(
      this.camera,
      this.controls,
      requireElement("#cloud-list"),
      requireElement("#cloud-empty"),
      requireElement("#cloud-count"),
      requireElement("#total-points"),
      this.requestRender,
    );
    this.scene.add(this.cloudManager.sceneGroup);

    this.ruler = new RulerTool(
      this.camera,
      this.canvas,
      requireElement("#ruler-labels"),
      requireElement("#ruler-list-section"),
      requireElement("#ruler-list"),
      requireElement("#ruler-toggle"),
      requireElement("#ruler-clear"),
      this.requestRender,
    );
    this.scene.add(this.ruler.sceneGroup);

    this.edl = new EdlPass(this.camera);

    this.bindEdlToggle();
    this.bindFileInput();
    this.bindDropzone();
    this.bindPointerEvents();
    this.bindKeyboard();

    // Observe the frame rather than the window so layout changes (panel wrapping,
    // devtools docking) are caught too. Fires once on observe().
    const frame = this.canvas.parentElement;
    if (frame) {
      new ResizeObserver(() => {
        this.resize();
        this.requestRender();
      }).observe(frame);
    }
    this.resize();
  }

  start(): void {
    this.requestRender();
  }

  /** Schedules at most one frame; safe to call from anywhere, any number of times. */
  private readonly requestRender = (): void => {
    if (this.frameHandle !== null) return;
    this.frameHandle = window.requestAnimationFrame((now) => this.renderFrame(now));
  };

  private bindEdlToggle(): void {
    const btn = requireElement<HTMLButtonElement>("#edl-toggle");
    btn.addEventListener("click", () => {
      this.edl.enabled = !this.edl.enabled;
      btn.textContent = this.edl.enabled ? "EDL on" : "EDL off";
      btn.setAttribute("aria-pressed", String(this.edl.enabled));
      this.requestRender();
    });
  }

  private bindFileInput(): void {
    const input = requireElement<HTMLInputElement>("#ply-input");
    input.addEventListener("change", () => this.loadFiles(input.files));
  }

  private bindDropzone(): void {
    const dropzone = requireElement<HTMLLabelElement>("#dropzone");
    dropzone.addEventListener("dragover", (e) => {
      e.preventDefault();
      dropzone.dataset.dragging = "true";
    });
    dropzone.addEventListener("dragleave", () => {
      delete dropzone.dataset.dragging;
    });
    dropzone.addEventListener("drop", (e) => {
      e.preventDefault();
      delete dropzone.dataset.dragging;
      this.loadFiles(e.dataTransfer?.files ?? null);
    });
  }

  private bindPointerEvents(): void {
    this.canvas.addEventListener("pointerdown", (e) => {
      if (this.ruler.isEnabled) {
        this.pointerDownPos = { x: e.clientX, y: e.clientY };
      }
    });

    this.canvas.addEventListener("pointerup", (e) => {
      if (!this.ruler.isEnabled || !this.pointerDownPos) return;
      const travel = Math.hypot(e.clientX - this.pointerDownPos.x, e.clientY - this.pointerDownPos.y);
      this.pointerDownPos = null;
      if (travel > 4) return;

      const point = this.cloudManager.pickPoint(e, this.canvas);
      if (point) this.ruler.addPoint(point);
    });

    this.canvas.addEventListener("pointerleave", () => {
      this.pointerDownPos = null;
    });
  }

  private bindKeyboard(): void {
    window.addEventListener("keydown", (e) => {
      if (e.repeat) return;
      const key = e.key.toLowerCase();

      if (key === "r" && this.shouldHandleKeys()) {
        e.preventDefault();
        this.camera.position.set(1.25, -1.25, 0.95);
        this.controls.target.set(0, 0, 0);
        this.controls.update();
        this.requestRender();
        return;
      }

      if (["w", "a", "s", "d", "q", "e"].includes(key)) {
        this.pressedKeys.add(key);
        if (this.shouldHandleKeys()) e.preventDefault();
        this.requestRender();
      }
    });

    window.addEventListener("keyup", (e) => this.pressedKeys.delete(e.key.toLowerCase()));
    window.addEventListener("blur", () => this.pressedKeys.clear());
  }

  private loadFiles(files: FileList | null): void {
    if (!files) return;
    for (const file of files) {
      this.cloudManager.add(file);
    }
  }

  private shouldHandleKeys(): boolean {
    const el = document.activeElement;
    return !el || !["INPUT", "BUTTON", "SELECT", "TEXTAREA"].includes(el.tagName);
  }

  private updateKeyboardNav(dt: number): void {
    if (!this.shouldHandleKeys()) return;

    let fwd = 0, lat = 0, vert = 0;
    if (this.pressedKeys.has("w")) fwd += 1;
    if (this.pressedKeys.has("s")) fwd -= 1;
    if (this.pressedKeys.has("d")) lat += 1;
    if (this.pressedKeys.has("a")) lat -= 1;
    if (this.pressedKeys.has("q")) vert += 1;
    if (this.pressedKeys.has("e")) vert -= 1;
    if (fwd === 0 && lat === 0 && vert === 0) return;

    const up = this.navUp;
    const forward = this.navForward.copy(this.controls.target).sub(this.camera.position).setZ(0);
    if (forward.lengthSq() < 1e-8) forward.set(1, 0, 0);
    else forward.normalize();

    const right = this.navRight.copy(forward).cross(up).normalize();
    const move = this.navMove.set(0, 0, 0);
    move.addScaledVector(forward, fwd);
    move.addScaledVector(right, lat);
    move.addScaledVector(up, vert);
    if (move.lengthSq() < 1e-8) return;

    move.normalize();
    const speed = Math.max(this.camera.position.distanceTo(this.controls.target) * 0.9, 0.25);
    move.multiplyScalar(speed * dt);
    this.camera.position.add(move);
    this.controls.target.add(move);
  }

  private resize(): void {
    const frame = this.canvas.parentElement;
    if (!frame) return;

    const { clientWidth, clientHeight } = frame;
    // Re-read the ratio so moving the window to another display is picked up.
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, MAX_PIXEL_RATIO));
    const dpr = this.renderer.getPixelRatio();
    this.camera.aspect = clientWidth / Math.max(clientHeight, 1);
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(clientWidth, clientHeight, false);
    this.edl.resize(Math.floor(clientWidth * dpr), Math.floor(clientHeight * dpr));
  }

  private renderFrame(now: number): void {
    this.frameHandle = null;

    // dt only means something while keys keep the loop running; after idle the
    // previous timestamp is stale, so the first frame assumes one 60 Hz step.
    const dt = this.lastNavTime === null ? 1 / 60 : Math.min((now - this.lastNavTime) / 1000, 0.1);
    this.lastNavTime = now;

    this.updateKeyboardNav(dt);
    this.controls.update();
    this.ruler.updateLabels();
    this.edl.render(this.renderer, this.scene);

    if (this.pressedKeys.size > 0 && this.shouldHandleKeys()) {
      this.requestRender();
    } else {
      this.lastNavTime = null;
    }
  }
}

const viewer = new ViewerApp();
viewer.start();
