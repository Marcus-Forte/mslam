# Viewer

Minimal TypeScript browser app for loading and inspecting local `.ply` files.
It is a static file viewer (the target is about 10 million points), not a live
stream listener.

Features:

- local `.ply` file loading with drag-and-drop
- Z-up scene orientation
- point-size slider
- multiple ruler measurements with on-screen distances
- `W`, `A`, `S`, `D` camera movement

## Commands

Run install, build and tests in the repository's devcontainer
(`.devcontainer/`, image `mdnf1992/cpp-dev`); `node_modules/` is installed for
the container's platform (Linux), so do not run `npm ci` on the host over it.
Inside the devcontainer (or with `docker exec <container> bash -lc 'cd
/workspaces/mslam/viewer && ...'`):

```bash
cd viewer
npm ci
npm run dev -- --host
```

Create a production build (type-checks first) and run the tests:

```bash
npm run build
npm test
```

The tests (`vitest`, node environment, no browser or WebGL) are the
`src/*.test.ts` files: the PLY fast path against three's `PLYLoader` on the
clouds in `test/data/registration/`, its edge cases, the colour baking and
point picking against `Raycaster`.

## How it works

- **PLY parsing** runs in a Web Worker (`plyWorker.ts`, `PlyLoaderClient.ts`),
  so loading a large file does not block the page. `plyParse.ts` has a fast
  path that reads typed arrays straight from a `DataView` for the layout mslam
  writes: binary little-endian, a single `vertex` element, scalar properties
  (`float` or `double` for `x`, `y`, `z`, any scalar type for `intensity`; other
  properties are skipped). Anything else (ASCII, big-endian, faces or other
  elements, list properties) falls back to three's `PLYLoader`, which is slower
  and needs more memory. A body shorter than the header declares, or a file with
  no vertices, is reported as an error.
- **Intensity colours** are baked once in the worker (`colormap.ts`): the
  rainbow colormap of msensor, normalized by the finite min/max of the cloud,
  stored as 8-bit RGB per point. A file without an `intensity` property is shown
  in a flat colour.
- **Rendering is on demand**: a frame is drawn only after something changed
  (camera, resize, file load, option change, held navigation key), not every
  display refresh.
- **Picking** (ruler) is a single allocation-free pass over the position array
  (`pick.ts`) with the same hit rule as three's `Raycaster` on `Points`.
