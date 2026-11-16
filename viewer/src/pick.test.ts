import {
  BufferAttribute,
  BufferGeometry,
  PerspectiveCamera,
  Points,
  PointsMaterial,
  Raycaster,
  Vector2,
  Vector3,
} from "three";
import { describe, expect, it } from "vitest";
import { type Pickable, PointPicker } from "./pick";

/** Deterministic pseudo-random numbers in [0, 1) (mulberry32). */
function rng(seed: number): () => number {
  let a = seed;
  return () => {
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function makeCloud(positions: Float32Array, setup?: (points: Points) => void): Points {
  const geometry = new BufferGeometry();
  geometry.setAttribute("position", new BufferAttribute(positions, 3));
  const points = new Points(geometry, new PointsMaterial());
  setup?.(points);
  points.updateMatrixWorld(true);
  return points;
}

function randomPositions(count: number, seed: number, extent: number): Float32Array {
  const next = rng(seed);
  const out = new Float32Array(count * 3);
  for (let i = 0; i < out.length; i += 1) out[i] = (next() - 0.5) * extent;
  return out;
}

const asPickable = (p: Points, visible = true): Pickable => ({ visible, geometry: p.geometry, mesh: p });

/** The previous implementation: Raycaster per cloud, nearest first hit across clouds, returning the cloud point. */
function raycasterPick(raycaster: Raycaster, clouds: Points[]): Vector3 | null {
  let best: { point: Vector3; distance: number } | null = null;
  for (const cloud of clouds) {
    const hits = raycaster.intersectObject(cloud);
    if (hits.length === 0 || hits[0].index === undefined) continue;
    const position = cloud.geometry.getAttribute("position");
    const point = new Vector3(
      position.getX(hits[0].index),
      position.getY(hits[0].index),
      position.getZ(hits[0].index),
    ).applyMatrix4(cloud.matrixWorld);
    if (!best || hits[0].distance < best.distance) best = { point, distance: hits[0].distance };
  }
  return best?.point ?? null;
}

describe("PointPicker", () => {
  const camera = new PerspectiveCamera(60, 1.5, 0.1, 100);
  camera.position.set(0, -8, 3);
  camera.up.set(0, 0, 1);
  camera.lookAt(0, 0, 0);
  camera.updateMatrixWorld(true);

  it("returns the same points as Raycaster on a synthetic cloud", () => {
    // Dense enough that many rays hit something, sparse enough that some miss.
    const cloud = makeCloud(randomPositions(400, 1, 6), (p) => {
      p.position.set(0.5, 0.25, -0.5);
      p.rotation.set(0.3, -0.2, 0.9);
    });
    const picker = new PointPicker();
    const raycaster = new Raycaster();
    raycaster.params.Points.threshold = 0.15;
    const pointer = new Vector2();
    const next = rng(7);

    let hits = 0;
    let misses = 0;
    for (let k = 0; k < 400; k += 1) {
      pointer.set(next() * 2 - 1, next() * 2 - 1);
      raycaster.setFromCamera(pointer, camera);
      const expected = raycasterPick(raycaster, [cloud]);
      const actual = picker.pick(raycaster.ray, raycaster.params.Points.threshold, [asPickable(cloud)]);

      if (expected === null) {
        expect(actual).toBeNull();
        misses += 1;
      } else {
        expect(actual).not.toBeNull();
        expect(actual!.distanceTo(expected)).toBeLessThan(1e-5);
        hits += 1;
      }
    }
    expect(hits).toBeGreaterThan(20);
    expect(misses).toBeGreaterThan(20);
  });

  it("returns the same points as Raycaster across several transformed clouds", () => {
    const clouds = [
      makeCloud(randomPositions(300, 11, 5), (p) => p.position.set(-1, 0, 0)),
      makeCloud(randomPositions(300, 12, 5), (p) => {
        p.position.set(1, 1, 0.5);
        p.scale.set(1.5, 1.5, 1.5);
        p.rotation.set(0, 0.7, 0.2);
      }),
    ];
    const picker = new PointPicker();
    const raycaster = new Raycaster();
    raycaster.params.Points.threshold = 0.2;
    const pointer = new Vector2();
    const next = rng(21);

    let hits = 0;
    for (let k = 0; k < 400; k += 1) {
      pointer.set(next() * 2 - 1, next() * 2 - 1);
      raycaster.setFromCamera(pointer, camera);
      const expected = raycasterPick(raycaster, clouds);
      const actual = picker.pick(raycaster.ray, 0.2, clouds.map((c) => asPickable(c)));
      if (expected === null) {
        expect(actual).toBeNull();
      } else {
        expect(actual).not.toBeNull();
        expect(actual!.distanceTo(expected)).toBeLessThan(1e-5);
        hits += 1;
      }
    }
    expect(hits).toBeGreaterThan(20);
  });

  it("picks the nearest cloud, honours visibility and non-uniform world transforms", () => {
    const near = makeCloud(new Float32Array([0, 0, 0]), (p) => p.position.set(0, -2, 0));
    const far = makeCloud(new Float32Array([0, 0, 0]), (p) => p.position.set(0, 2, 0));
    const scaled = makeCloud(new Float32Array([1, 0, 0]), (p) => {
      p.position.set(5, 0, 0);
      p.scale.set(2, 2, 2);
    });
    const ray = new Raycaster(new Vector3(0, -10, 0), new Vector3(0, 1, 0));
    const picker = new PointPicker();
    const threshold = 0.1;

    const both = picker.pick(ray.ray, threshold, [asPickable(far), asPickable(near)]);
    expect(both!.toArray()).toEqual([0, -2, 0]);

    const hidden = picker.pick(ray.ray, threshold, [asPickable(far), asPickable(near, false)]);
    expect(hidden!.toArray()).toEqual([0, 2, 0]);

    expect(picker.pick(ray.ray, threshold, [asPickable(near, false)])).toBeNull();

    // Local point (1,0,0) scaled by 2 and moved by 5 lands at world (7,0,0).
    const side = new Raycaster(new Vector3(7, -10, 0), new Vector3(0, 1, 0));
    expect(picker.pick(side.ray, threshold, [asPickable(scaled)])!.toArray()).toEqual([7, 0, 0]);
    expect(picker.pick(ray.ray, threshold, [asPickable(scaled)])).toBeNull();
  });

  it("ignores points behind the ray origin and beyond the threshold", () => {
    const cloud = makeCloud(new Float32Array([0, -20, 0, 0.5, 0, 0]));
    const ray = new Raycaster(new Vector3(0, -10, 0), new Vector3(0, 1, 0));
    const picker = new PointPicker();
    // (0,-20,0) is behind the origin, (0.5,0,0) is 0.5 off the ray.
    expect(picker.pick(ray.ray, 0.1, [asPickable(cloud)])).toBeNull();
    expect(picker.pick(ray.ray, 0.6, [asPickable(cloud)])!.toArray()).toEqual([0.5, 0, 0]);
  });

  it("returns a fresh vector, not internal state", () => {
    const cloud = makeCloud(new Float32Array([0, 0, 0]));
    const ray = new Raycaster(new Vector3(0, -10, 0), new Vector3(0, 1, 0));
    const picker = new PointPicker();
    const a = picker.pick(ray.ray, 0.1, [asPickable(cloud)])!;
    a.set(9, 9, 9);
    expect(picker.pick(ray.ray, 0.1, [asPickable(cloud)])!.toArray()).toEqual([0, 0, 0]);
  });
});
