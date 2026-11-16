/**
 * Rainbow colormap matching _getRainbowColor from msensor (see client/colormap.py).
 * Maps a normalized [0,1] value to an RGB triplet in [0,1].
 */
function rainbowInto(value: number, out: Uint8Array, offset: number): void {
  const v = Math.max(0, Math.min(1, value));
  const h = v * 5.0 + 1.0;
  const i = Math.floor(h);
  let f = h - i;
  if ((i & 1) === 0) f = 1.0 - f;
  const n = 1.0 - f;

  let r: number, g: number, b: number;
  if (i <= 1) { r = n; g = 0; b = 1; }
  else if (i === 2) { r = 0; g = n; b = 1; }
  else if (i === 3) { r = 0; g = 1; b = n; }
  else if (i === 4) { r = n; g = 1; b = 0; }
  else { r = 1; g = n; b = 0; }

  // Quantized to 8 bits: at most 0.5/255 off the float colormap.
  out[offset] = Math.round(r * 255);
  out[offset + 1] = Math.round(g * 255);
  out[offset + 2] = Math.round(b * 255);
}

/**
 * Bakes intensities into RGB bytes (3 per point, to be used as a normalized
 * Uint8 attribute). Intensities are normalized by their finite min/max;
 * NaN maps to the low end of the scale, infinities to the nearest end.
 */
export function bakeRainbow(intensity: Float32Array): Uint8Array {
  const count = intensity.length;
  const colors = new Uint8Array(count * 3);

  let minI = Number.POSITIVE_INFINITY;
  let maxI = Number.NEGATIVE_INFINITY;
  for (let i = 0; i < count; i += 1) {
    const v = intensity[i];
    if (!Number.isFinite(v)) continue;
    if (v < minI) minI = v;
    if (v > maxI) maxI = v;
  }
  if (!(minI <= maxI)) {
    minI = 0;
    maxI = 0;
  }
  const span = Math.max(maxI - minI, 1e-6);

  for (let i = 0; i < count; i += 1) {
    const n = (intensity[i] - minI) / span;
    rainbowInto(Number.isNaN(n) ? 0 : n, colors, i * 3);
  }
  return colors;
}
