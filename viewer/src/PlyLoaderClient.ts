import type { PlyWorkerRequest, PlyWorkerResponse } from "./plyWorker";

export type LoadedCloud = {
  positions: Float32Array;
  /** Baked rainbow colours (3 bytes per point), null when the file has no intensity. */
  colors: Uint8Array | null;
};

type Pending = { resolve: (cloud: LoadedCloud) => void; reject: (error: Error) => void };

/** Parses PLY files in a module Web Worker, created on first use. */
export class PlyLoaderClient {
  private worker: Worker | null = null;
  private readonly pending = new Map<number, Pending>();
  private nextId = 0;

  load(file: File): Promise<LoadedCloud> {
    const worker = this.ensureWorker();
    const id = this.nextId++;
    return new Promise<LoadedCloud>((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      const request: PlyWorkerRequest = { id, file };
      worker.postMessage(request);
    });
  }

  private ensureWorker(): Worker {
    if (this.worker) return this.worker;

    const worker = new Worker(new URL("./plyWorker.ts", import.meta.url), { type: "module" });
    worker.onmessage = (e: MessageEvent<PlyWorkerResponse>) => {
      const msg = e.data;
      const pending = this.pending.get(msg.id);
      if (!pending) return;
      this.pending.delete(msg.id);
      if (msg.ok) pending.resolve({ positions: msg.positions, colors: msg.colors });
      else pending.reject(new Error(msg.error));
    };
    // A worker-level failure (script load, out of memory) loses every request in
    // flight; fail them and start a fresh worker on the next load.
    worker.onerror = (e) => {
      const error = new Error(e.message || "PLY worker failed");
      for (const pending of this.pending.values()) pending.reject(error);
      this.pending.clear();
      worker.terminate();
      this.worker = null;
    };
    this.worker = worker;
    return worker;
  }
}
