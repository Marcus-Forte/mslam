import { bakeRainbow } from "./colormap";
import { parsePly } from "./plyParse";

export type PlyWorkerRequest = { id: number; file: File };
export type PlyWorkerResponse =
  | { id: number; ok: true; positions: Float32Array; colors: Uint8Array | null }
  | { id: number; ok: false; error: string };

const scope = self as unknown as {
  onmessage: ((e: MessageEvent<PlyWorkerRequest>) => void) | null;
  postMessage(message: PlyWorkerResponse, transfer: Transferable[]): void;
};

scope.onmessage = (e) => {
  const { id, file } = e.data;
  file
    .arrayBuffer()
    .then((buffer) => {
      const { positions, intensity } = parsePly(buffer);
      const colors = intensity ? bakeRainbow(intensity) : null;
      const transfer: Transferable[] = [positions.buffer];
      if (colors) transfer.push(colors.buffer);
      scope.postMessage({ id, ok: true, positions, colors }, transfer);
    })
    .catch((error: unknown) => {
      scope.postMessage(
        { id, ok: false, error: error instanceof Error ? error.message : String(error) },
        [],
      );
    });
};
