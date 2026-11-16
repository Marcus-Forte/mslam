import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";

export type ParsedPly = {
  positions: Float32Array;
  /** Per-point intensity, or null when the file has no `intensity` property. */
  intensity: Float32Array | null;
};

type Reader = (view: DataView, offset: number) => number;

type ScalarType = { size: number; read: Reader };

// Little-endian readers, keyed by every spelling PLYLoader accepts.
const rd = {
  i8: { size: 1, read: (v: DataView, o: number) => v.getInt8(o) },
  u8: { size: 1, read: (v: DataView, o: number) => v.getUint8(o) },
  i16: { size: 2, read: (v: DataView, o: number) => v.getInt16(o, true) },
  u16: { size: 2, read: (v: DataView, o: number) => v.getUint16(o, true) },
  i32: { size: 4, read: (v: DataView, o: number) => v.getInt32(o, true) },
  u32: { size: 4, read: (v: DataView, o: number) => v.getUint32(o, true) },
  f32: { size: 4, read: (v: DataView, o: number) => v.getFloat32(o, true) },
  f64: { size: 8, read: (v: DataView, o: number) => v.getFloat64(o, true) },
} satisfies Record<string, ScalarType>;

const SCALAR_TYPES: Record<string, ScalarType> = {
  char: rd.i8, int8: rd.i8,
  uchar: rd.u8, uint8: rd.u8,
  short: rd.i16, int16: rd.i16,
  ushort: rd.u16, uint16: rd.u16,
  int: rd.i32, int32: rd.i32,
  uint: rd.u32, uint32: rd.u32,
  float: rd.f32, float32: rd.f32,
  double: rd.f64, float64: rd.f64,
};

type FastProp = { name: string; type: ScalarType; offset: number };
type FastHeader = { count: number; stride: number; bodyStart: number; props: FastProp[] };

const HEADER_SCAN_BYTES = 1 << 20;
const latin1 = new TextDecoder("latin1");

/**
 * Parses the header of the layout mslam writes (binary little-endian, a single
 * `vertex` element, scalar properties). Returns null for anything else so the
 * caller can fall back to PLYLoader.
 */
function parseFastHeader(bytes: Uint8Array): FastHeader | null {
  const text = latin1.decode(bytes.subarray(0, Math.min(bytes.length, HEADER_SCAN_BYTES)));
  const m = /(?:^|\n)end_header(\r?\n)/.exec(text);
  if (!m) return null;
  const bodyStart = m.index + m[0].length;
  const lines = text.slice(0, m.index + m[0].indexOf("end_header")).split(/\r?\n/);
  if (lines[0] !== "ply") return null;

  let littleEndian = false;
  let count = -1;
  let stride = 0;
  const props: FastProp[] = [];
  const seen = new Set<string>();

  for (let li = 1; li < lines.length; li += 1) {
    const tok = lines[li].trim().split(/\s+/);
    switch (tok[0]) {
      case "format":
        if (tok[1] !== "binary_little_endian") return null;
        littleEndian = true;
        break;
      case "element":
        if (tok[1] !== "vertex" || count !== -1) return null;
        count = Number(tok[2]);
        if (!Number.isInteger(count) || count < 0) return null;
        break;
      case "property": {
        if (count === -1 || tok.length !== 3) return null; // list properties have 5 tokens
        const type = SCALAR_TYPES[tok[1]];
        if (!type || seen.has(tok[2])) return null;
        seen.add(tok[2]);
        props.push({ name: tok[2], type, offset: stride });
        stride += type.size;
        break;
      }
      default:
        break; // comment, obj_info, blank
    }
  }

  if (!littleEndian || count === -1) return null;
  if (!seen.has("x") || !seen.has("y") || !seen.has("z")) return null;
  return { count, stride, bodyStart, props };
}

/** Fast path: fills typed arrays straight from a DataView. Null when not applicable. */
export function parsePlyFast(buffer: ArrayBuffer): ParsedPly | null {
  const header = parseFastHeader(new Uint8Array(buffer));
  if (!header) return null;

  const { count, stride, bodyStart, props } = header;
  if (bodyStart + count * stride > buffer.byteLength) {
    throw new Error("PLY body is shorter than the header declares");
  }

  const find = (name: string): FastProp | undefined => props.find((p) => p.name === name);
  const px = find("x")!, py = find("y")!, pz = find("z")!, pi = find("intensity");

  const view = new DataView(buffer, bodyStart);
  const positions = new Float32Array(count * 3);
  const intensity = pi ? new Float32Array(count) : null;

  const allF32 = [px, py, pz, ...(pi ? [pi] : [])].every((p) => p.type === rd.f32);
  if (allF32) {
    for (let i = 0, base = 0; i < count; i += 1, base += stride) {
      positions[i * 3] = view.getFloat32(base + px.offset, true);
      positions[i * 3 + 1] = view.getFloat32(base + py.offset, true);
      positions[i * 3 + 2] = view.getFloat32(base + pz.offset, true);
    }
    if (pi && intensity) {
      for (let i = 0, base = pi.offset; i < count; i += 1, base += stride) {
        intensity[i] = view.getFloat32(base, true);
      }
    }
  } else {
    const rx = px.type.read, ry = py.type.read, rz = pz.type.read;
    for (let i = 0, base = 0; i < count; i += 1, base += stride) {
      positions[i * 3] = rx(view, base + px.offset);
      positions[i * 3 + 1] = ry(view, base + py.offset);
      positions[i * 3 + 2] = rz(view, base + pz.offset);
    }
    if (pi && intensity) {
      const ri = pi.type.read;
      for (let i = 0, base = pi.offset; i < count; i += 1, base += stride) {
        intensity[i] = ri(view, base);
      }
    }
  }

  return { positions, intensity };
}

/** Generic path: PLYLoader handles ASCII, big-endian, faces, lists, ... */
export function parsePlyFallback(buffer: ArrayBuffer): ParsedPly {
  const loader = new PLYLoader();
  loader.setCustomPropertyNameMapping({ intensity: ["intensity"] });
  const geometry = loader.parse(buffer);

  const position = geometry.getAttribute("position");
  if (!position || position.count === 0) {
    geometry.dispose();
    throw new Error("PLY contains no vertices");
  }
  const intensityAttr = geometry.getAttribute("intensity");
  // PLYLoader fills the custom attribute with NaN (undefined) when the file has
  // no `intensity` property; treat that as "no intensity".
  const intensity = intensityAttr ? (intensityAttr.array as Float32Array) : null;
  const result: ParsedPly = {
    positions: position.array as Float32Array,
    intensity: intensity && intensity.some((v) => !Number.isNaN(v)) ? intensity : null,
  };
  geometry.dispose();
  return result;
}

export function parsePly(buffer: ArrayBuffer): ParsedPly {
  const fast = parsePlyFast(buffer);
  if (fast) {
    if (fast.positions.length === 0) throw new Error("PLY contains no vertices");
    return fast;
  }
  return parsePlyFallback(buffer);
}
