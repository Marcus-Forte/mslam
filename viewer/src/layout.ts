export const appLayout = `
  <div class="shell">
    <section class="panel">
      <p class="eyebrow">MSLAM Viewer</p>
      <p class="lede">
        Load one or more <span>.ply</span> point clouds and inspect them in the browser.
      </p>

      <label class="dropzone" for="ply-input" id="dropzone">
        <input id="ply-input" type="file" accept=".ply" multiple />
        <strong>Choose PLY files</strong>
        <span>or drag them here</span>
      </label>

      <div class="cloud-list-section">
        <p class="section-label">Loaded clouds</p>
        <div class="cloud-list" id="cloud-list">
          <p class="cloud-empty" id="cloud-empty">No clouds loaded</p>
        </div>
      </div>

      <div class="controls-panel">
        <div class="tool-row">
          <button id="edl-toggle" class="tool-button" type="button" aria-pressed="true">
            EDL on
          </button>
          <button id="ruler-toggle" class="tool-button" type="button" aria-pressed="false">
            Ruler off
          </button>
        </div>
      </div>

      <div class="ruler-list-section" id="ruler-list-section" hidden>
        <div class="ruler-list-header">
          <p class="section-label">Measurements</p>
          <button id="ruler-clear" class="tool-button secondary small" type="button">Clear all</button>
        </div>
        <div class="ruler-list" id="ruler-list"></div>
      </div>

      <dl class="stats">
        <div>
          <dt>Clouds</dt>
          <dd id="cloud-count">0</dd>
        </div>
        <div>
          <dt>Total points</dt>
          <dd id="total-points">0</dd>
        </div>
      </dl>

      <p class="hint">
        Drag to orbit. Scroll to zoom. Right-drag to pan. Use W A S D to move and Q E for vertical motion.
      </p>
    </section>

    <section class="viewer-frame">
      <canvas id="viewer-canvas"></canvas>
      <div id="ruler-labels" class="ruler-labels"></div>
    </section>
  </div>
`;
