import { describe, expect, it } from "vitest";
import { bakeRainbow } from "./colormap";

const rgb = (colors: Uint8Array, i: number) => Array.from(colors.subarray(i * 3, i * 3 + 3));

const LOW = [255, 0, 255]; // normalized 0
const HIGH = [255, 0, 0]; // normalized 1

describe("bakeRainbow", () => {
  it("returns 3 bytes per point, empty for no points", () => {
    expect(bakeRainbow(new Float32Array(0)).length).toBe(0);
    expect(bakeRainbow(new Float32Array([1, 2, 3, 4])).length).toBe(12);
  });

  it("maps the minimum and maximum to the ends of the scale", () => {
    const colors = bakeRainbow(new Float32Array([10, 20, 30]));
    expect(rgb(colors, 0)).toEqual(LOW);
    expect(rgb(colors, 2)).toEqual(HIGH);
  });

  it("hits the colormap stops at 0.2 steps", () => {
    const colors = bakeRainbow(new Float32Array([0, 0.2, 0.4, 0.6, 0.8, 1]));
    expect(rgb(colors, 0)).toEqual(LOW);
    expect(rgb(colors, 1)).toEqual([0, 0, 255]);
    expect(rgb(colors, 2)).toEqual([0, 255, 255]);
    expect(rgb(colors, 3)).toEqual([0, 255, 0]);
    expect(rgb(colors, 4)).toEqual([255, 255, 0]);
    expect(rgb(colors, 5)).toEqual(HIGH);
  });

  it("normalizes by the data range, not by absolute value", () => {
    expect(bakeRainbow(new Float32Array([1000, 1005, 1010]))).toEqual(
      bakeRainbow(new Float32Array([0, 5, 10])),
    );
  });

  it("maps constant intensity to one colour at the low end", () => {
    const colors = bakeRainbow(new Float32Array([7, 7, 7]));
    for (let i = 0; i < 3; i += 1) expect(rgb(colors, i)).toEqual(LOW);
  });

  it("maps NaN to the low end and ignores it for min/max", () => {
    const colors = bakeRainbow(new Float32Array([Number.NaN, 0, 1]));
    expect(rgb(colors, 0)).toEqual(LOW);
    expect(rgb(colors, 1)).toEqual(LOW);
    expect(rgb(colors, 2)).toEqual(HIGH);
  });

  it("handles all-NaN input", () => {
    const colors = bakeRainbow(new Float32Array([Number.NaN, Number.NaN]));
    expect(Array.from(colors)).toEqual([...LOW, ...LOW]);
  });

  it("clamps infinities to the ends of the scale without distorting the finite values", () => {
    const colors = bakeRainbow(new Float32Array([Number.NEGATIVE_INFINITY, 0, 1, Number.POSITIVE_INFINITY]));
    expect(rgb(colors, 0)).toEqual(LOW);
    expect(rgb(colors, 1)).toEqual(LOW);
    expect(rgb(colors, 2)).toEqual(HIGH);
    expect(rgb(colors, 3)).toEqual(HIGH);
  });
});
