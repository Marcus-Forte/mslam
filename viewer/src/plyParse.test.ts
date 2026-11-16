import { readdirSync, readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { describe, expect, it } from "vitest";
import { parsePly, parsePlyFallback, parsePlyFast } from "./plyParse";

const dataDir = fileURLToPath(new URL("../../test/data/registration/", import.meta.url));

/** Exact element-wise equality that fails with the first differing index (a toEqual diff of a 10^5-element array is very slow). */
function expectSameArray(actual: ArrayLike<number> | null, expected: ArrayLike<number> | null): void {
  if (actual === null || expected === null) {
    expect(actual).toBe(expected);
    return;
  }
  expect(actual.length).toBe(expected.length);
  for (let i = 0; i < actual.length; i += 1) {
    if (!Object.is(actual[i], expected[i])) expect.fail(`index ${i}: ${actual[i]} !== ${expected[i]}`);
  }
}

function toArrayBuffer(bytes: Uint8Array): ArrayBuffer {
  return bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) as ArrayBuffer;
}

type Column = { name: string; type: "float" | "double" | "uchar"; values: number[] };

/** Binary little-endian PLY with one `vertex` element made of the given columns. */
function binaryPly(columns: Column[], extraHeader = "", declaredCount?: number): ArrayBuffer {
  const count = columns[0]?.values.length ?? 0;
  const size = { float: 4, double: 8, uchar: 1 };
  const stride = columns.reduce((s, c) => s + size[c.type], 0);
  const header =
    "ply\nformat binary_little_endian 1.0\n" +
    `element vertex ${declaredCount ?? count}\n` +
    columns.map((c) => `property ${c.type} ${c.name}\n`).join("") +
    extraHeader +
    "end_header\n";
  const head = new TextEncoder().encode(header);
  const bytes = new Uint8Array(head.length + count * stride);
  bytes.set(head);
  const view = new DataView(bytes.buffer);
  for (let i = 0; i < count; i += 1) {
    let offset = head.length + i * stride;
    for (const c of columns) {
      if (c.type === "float") view.setFloat32(offset, c.values[i], true);
      else if (c.type === "double") view.setFloat64(offset, c.values[i], true);
      else view.setUint8(offset, c.values[i]);
      offset += size[c.type];
    }
  }
  return toArrayBuffer(bytes);
}

function asciiPly(rows: number[][], props: string[], extraHeader = "", tail = ""): ArrayBuffer {
  const text =
    "ply\nformat ascii 1.0\n" +
    `element vertex ${rows.length}\n` +
    props.map((p) => `property float ${p}\n`).join("") +
    extraHeader +
    "end_header\n" +
    rows.map((r) => r.join(" ")).join("\n") +
    (rows.length > 0 ? "\n" : "") +
    tail;
  return toArrayBuffer(new TextEncoder().encode(text));
}

const xs = [1.5, -2.25, 3];
const ys = [0.5, 4, -6.125];
const zs = [10, 20.5, -30];
const is = [0.1, 0.5, 0.9].map(Math.fround);
const expectedPositions = [1.5, 0.5, 10, -2.25, 4, 20.5, 3, -6.125, -30];

describe("parsePlyFast on the registration test clouds", () => {
  const files = readdirSync(dataDir).filter((f) => f.endsWith(".ply"));

  it("finds test clouds", () => {
    expect(files.length).toBeGreaterThan(0);
  });

  it.each(files)("%s: matches three's PLYLoader (positions and intensity)", (file) => {
    const buffer = toArrayBuffer(readFileSync(dataDir + file));
    const fast = parsePlyFast(buffer);
    expect(fast, "mslam's own layout must take the fast path").not.toBeNull();
    const reference = parsePlyFallback(buffer);

    expect(fast!.positions.length).toBeGreaterThan(0);
    expectSameArray(fast!.positions, reference.positions);
    expect(fast!.intensity).not.toBeNull();
    expectSameArray(fast!.intensity, reference.intensity);
    expectSameArray(parsePly(buffer).positions, reference.positions);
  });
});

describe("parsePly edge cases", () => {
  it("reads float xyz + intensity and skips other properties by stride", () => {
    const buffer = binaryPly([
      { name: "x", type: "float", values: xs },
      { name: "r", type: "uchar", values: [1, 2, 3] },
      { name: "y", type: "float", values: ys },
      { name: "intensity", type: "float", values: is },
      { name: "z", type: "float", values: zs },
    ]);
    const parsed = parsePlyFast(buffer)!;
    expect(Array.from(parsed.positions)).toEqual(expectedPositions);
    expect(Array.from(parsed.intensity!)).toEqual(is);
  });

  it("reads double xyz on the fast path, rounded to float32", () => {
    const buffer = binaryPly([
      { name: "x", type: "double", values: xs },
      { name: "y", type: "double", values: ys },
      { name: "z", type: "double", values: [10.1, 20.2, -30.3] },
      { name: "intensity", type: "float", values: is },
    ]);
    const parsed = parsePlyFast(buffer);
    expect(parsed).not.toBeNull();
    expect(Array.from(parsed!.positions)).toEqual(
      [1.5, 0.5, 10.1, -2.25, 4, 20.2, 3, -6.125, -30.3].map(Math.fround),
    );
    expect(Array.from(parsed!.intensity!)).toEqual(is);
    expect(parsed!.positions).toEqual(parsePlyFallback(buffer).positions);
  });

  it("returns null intensity when the file has none", () => {
    const buffer = binaryPly([
      { name: "x", type: "float", values: xs },
      { name: "y", type: "float", values: ys },
      { name: "z", type: "float", values: zs },
    ]);
    expect(parsePlyFast(buffer)!.intensity).toBeNull();
    expect(parsePlyFallback(buffer).intensity).toBeNull();
    expect(parsePly(buffer).intensity).toBeNull();
  });

  it("falls back to PLYLoader for ASCII", () => {
    const rows = [
      [1.5, 0.5, 10, 0.25],
      [-2.25, 4, 20.5, 0.5],
    ];
    const buffer = asciiPly(rows, ["x", "y", "z", "intensity"]);
    expect(parsePlyFast(buffer)).toBeNull();
    const parsed = parsePly(buffer);
    expect(Array.from(parsed.positions)).toEqual([1.5, 0.5, 10, -2.25, 4, 20.5]);
    expect(Array.from(parsed.intensity!)).toEqual([0.25, 0.5]);
  });

  it("falls back for a binary file with an extra face element", () => {
    const vertices = binaryPly(
      [
        { name: "x", type: "float", values: xs },
        { name: "y", type: "float", values: ys },
        { name: "z", type: "float", values: zs },
      ],
      "element face 1\nproperty list uchar int vertex_indices\n",
    );
    // Append one triangle (count byte + three int32 indices) to the body.
    const face = new Uint8Array(1 + 3 * 4);
    face[0] = 3;
    new DataView(face.buffer).setInt32(1, 0, true);
    new DataView(face.buffer).setInt32(5, 1, true);
    new DataView(face.buffer).setInt32(9, 2, true);
    const bytes = new Uint8Array(vertices.byteLength + face.length);
    bytes.set(new Uint8Array(vertices));
    bytes.set(face, vertices.byteLength);
    const buffer = toArrayBuffer(bytes);

    expect(parsePlyFast(buffer)).toBeNull();
    expect(Array.from(parsePly(buffer).positions)).toEqual(expectedPositions);
  });

  it("falls back for big-endian files", () => {
    const text =
      "ply\nformat binary_big_endian 1.0\nelement vertex 1\n" +
      "property float x\nproperty float y\nproperty float z\nend_header\n";
    const head = new TextEncoder().encode(text);
    const bytes = new Uint8Array(head.length + 12);
    bytes.set(head);
    const view = new DataView(bytes.buffer);
    view.setFloat32(head.length, 1, false);
    view.setFloat32(head.length + 4, 2, false);
    view.setFloat32(head.length + 8, 3, false);
    const buffer = toArrayBuffer(bytes);
    expect(parsePlyFast(buffer)).toBeNull();
    expect(Array.from(parsePly(buffer).positions)).toEqual([1, 2, 3]);
  });

  it("rejects a truncated binary body", () => {
    const buffer = binaryPly(
      [
        { name: "x", type: "float", values: xs },
        { name: "y", type: "float", values: ys },
        { name: "z", type: "float", values: zs },
      ],
      "",
      4, // header declares one more vertex than the body holds
    );
    expect(() => parsePly(buffer)).toThrow(/shorter than the header declares/);
  });

  it("rejects a binary file with zero vertices", () => {
    const buffer = binaryPly([
      { name: "x", type: "float", values: [] },
      { name: "y", type: "float", values: [] },
      { name: "z", type: "float", values: [] },
    ]);
    expect(() => parsePly(buffer)).toThrow(/no vertices/);
  });

  it("rejects an ASCII file with zero vertices", () => {
    expect(() => parsePly(asciiPly([], ["x", "y", "z"]))).toThrow(/no vertices/);
  });
});
