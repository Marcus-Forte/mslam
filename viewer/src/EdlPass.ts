import {
  DepthTexture,
  Mesh,
  NearestFilter,
  OrthographicCamera,
  PerspectiveCamera,
  PlaneGeometry,
  Scene,
  ShaderMaterial,
  UnsignedIntType,
  Vector2,
  WebGLRenderer,
  WebGLRenderTarget,
} from "three";
import { edlFragmentShader, edlVertexShader } from "./edlShader";

export class EdlPass {
  private readonly renderTarget: WebGLRenderTarget;
  private readonly material: ShaderMaterial;
  private readonly scene = new Scene();
  private readonly orthoCamera = new OrthographicCamera(-1, 1, 1, -1, 0, 1);
  enabled = true;

  constructor(private readonly camera: PerspectiveCamera) {
    this.renderTarget = new WebGLRenderTarget(1, 1, {
      minFilter: NearestFilter,
      magFilter: NearestFilter,
    });
    this.renderTarget.depthTexture = new DepthTexture(1, 1, UnsignedIntType);

    this.material = new ShaderMaterial({
      uniforms: {
        tColor: { value: this.renderTarget.texture },
        tDepth: { value: this.renderTarget.depthTexture },
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

    this.scene.add(new Mesh(new PlaneGeometry(2, 2), this.material));
  }

  resize(width: number, height: number): void {
    this.renderTarget.setSize(width, height);
    this.material.uniforms.uScreenSize.value.set(width, height);
  }

  render(renderer: WebGLRenderer, mainScene: Scene): void {
    if (!this.enabled) {
      renderer.render(mainScene, this.camera);
      return;
    }

    this.material.uniforms.uNear.value = this.camera.near;
    this.material.uniforms.uFar.value = this.camera.far;
    renderer.setRenderTarget(this.renderTarget);
    renderer.render(mainScene, this.camera);
    renderer.setRenderTarget(null);
    renderer.render(this.scene, this.orthoCamera);
  }
}
