import { type BufferGeometry, Matrix4, Ray, Sphere, Vector3 } from "three";

/** What picking needs from a cloud; CloudEntry satisfies it. */
export type Pickable = {
  visible: boolean;
  geometry: BufferGeometry;
  mesh: { matrixWorld: Matrix4; scale: Vector3 };
};

/**
 * Finds the point nearest the ray origin among those within `threshold` of the
 * ray, with the same rule as Raycaster on Points but in one allocation-free
 * pass over each cloud's position array. Holds scratch objects, so keep one
 * instance and reuse it.
 */
export class PointPicker {
  private readonly sphere = new Sphere();
  private readonly inverse = new Matrix4();
  private readonly localRay = new Ray();
  private readonly closest = new Vector3();
  private readonly best = new Vector3();

  /** The returned vector is a fresh copy in world space, or null when nothing is hit. */
  pick(worldRay: Ray, threshold: number, clouds: Iterable<Pickable>): Vector3 | null {
    let found = false;
    let bestDistance = Number.POSITIVE_INFINITY;

    for (const cloud of clouds) {
      if (!cloud.visible) continue;

      const geometry = cloud.geometry;
      if (geometry.boundingSphere === null) geometry.computeBoundingSphere();
      const sphere = geometry.boundingSphere;
      const position = geometry.getAttribute("position");
      if (!sphere || !position) continue;

      const matrixWorld = cloud.mesh.matrixWorld;
      this.sphere.copy(sphere).applyMatrix4(matrixWorld);
      this.sphere.radius += threshold;
      if (!worldRay.intersectsSphere(this.sphere)) continue;

      // Ray into the cloud's local frame once.
      this.inverse.copy(matrixWorld).invert();
      const ray = this.localRay.copy(worldRay).applyMatrix4(this.inverse);
      const scale = cloud.mesh.scale;
      const localThreshold = threshold / ((scale.x + scale.y + scale.z) / 3);
      const thresholdSq = localThreshold * localThreshold;

      const ox = ray.origin.x, oy = ray.origin.y, oz = ray.origin.z;
      const dx = ray.direction.x, dy = ray.direction.y, dz = ray.direction.z;
      const array = position.array;
      const n = position.count * 3;

      let bestT = Number.POSITIVE_INFINITY;
      let bestIndex = -1;
      for (let i = 0; i < n; i += 3) {
        const vx = array[i] - ox;
        const vy = array[i + 1] - oy;
        const vz = array[i + 2] - oz;
        // Closest point on the ray (clamped at its origin), as Ray.closestPointToPoint.
        let t = vx * dx + vy * dy + vz * dz;
        if (t < 0) t = 0;
        const ex = vx - t * dx;
        const ey = vy - t * dy;
        const ez = vz - t * dz;
        if (ex * ex + ey * ey + ez * ez < thresholdSq && t < bestT) {
          bestT = t;
          bestIndex = i;
        }
      }
      if (bestIndex < 0) continue;

      // Compare across clouds by world-space distance to the closest point on the ray.
      this.closest.copy(ray.direction).multiplyScalar(bestT).add(ray.origin).applyMatrix4(matrixWorld);
      const distance = worldRay.origin.distanceTo(this.closest);
      if (distance < bestDistance) {
        bestDistance = distance;
        found = true;
        this.best.set(array[bestIndex], array[bestIndex + 1], array[bestIndex + 2]).applyMatrix4(matrixWorld);
      }
    }

    return found ? this.best.clone() : null;
  }
}
