class WorkerStream {
  constructor({ state, onStatusNote }) {
    this.state = state;
    this.onStatusNote = typeof onStatusNote === 'function' ? onStatusNote : () => {};
    this.client = null;
    this.ready = false;
    this.status = '';
    this.fillRequested = false;
    this.pendingApply = null;
    this.readyPromise = null;
    this.resolveReady = null;
    this.rejectReady = null;
    this.queueItems = [];
    this.queueStart = 0;
  }

  hasClient() { return !!this.client; }
  isReady() { return !!this.ready; }

  computeBufferConfig() {
    const simDt = Math.max(Number(this.state?.simDt) || (1 / 240), 1e-6);
    const playSpeed = Math.max(Number(this.state?.renderSpeed) || 0, 0);
    if (!(playSpeed > 0)) {
      return { lowWater: 96, highWater: 192, chunkSize: 12 };
    }
    const rate = playSpeed / simDt;
    const lowWater = Math.max(96, Math.min(768, Math.ceil(rate * 0.40)));
    const highWater = Math.max(lowWater + 24, Math.min(1536, Math.ceil(rate * 0.85)));
    const chunkSize = Math.max(8, Math.min(24, Math.ceil(rate * 0.06)));
    return { lowWater, highWater, chunkSize };
  }

  lowWatermark() {
    return this.computeBufferConfig().lowWater;
  }
  queueLength() { return this.queueItems.length - this.queueStart; }
  queuePeek() { return this.queueStart < this.queueItems.length ? this.queueItems[this.queueStart] : null; }
  queueClear() { this.queueItems.length = 0; this.queueStart = 0; this.fillRequested = false; }
  queuePushMany(frames) { if (Array.isArray(frames) && frames.length) this.queueItems.push(...frames); }
  queueShift() {
    if (this.queueStart >= this.queueItems.length) return null;
    const value = this.queueItems[this.queueStart++];
    if (this.queueStart > 64 && this.queueStart * 2 >= this.queueItems.length) {
      this.queueItems = this.queueItems.slice(this.queueStart);
      this.queueStart = 0;
    }
    return value;
  }

  _handleMessage = (event) => {
    const data = event.data || {};
    if (data.type === 'ready') {
      this.ready = true;
      this.state.useWasm = true;
      this.status = 'Worker ready';
      if (this.resolveReady) { this.resolveReady(true); this.resolveReady = this.rejectReady = null; }
      return;
    }
    if (data.type === 'unavailable') {
      this.ready = false;
      this.state.useWasm = false;
      this.status = `Worker unavailable: ${data.message || 'unknown'}`;
      this.onStatusNote(this.status);
      if (this.resolveReady) { this.resolveReady(false); this.resolveReady = this.rejectReady = null; }
      return;
    }
    if (data.type === 'frames') {
      this.fillRequested = false;
      if (this.state.initializing) return;
      if (data.generation != null && data.generation !== this.state.workerGeneration) return;
      this.queuePushMany(data.frames);
      return;
    }
    if (data.type === 'applied') {
      this.fillRequested = false;
      if (this.pendingApply) {
        this.pendingApply.resolve(data);
        this.pendingApply = null;
      }
      return;
    }
    if (data.type === 'apply_error') {
      this.fillRequested = false;
      if (this.pendingApply) {
        this.pendingApply.reject(new Error(data.message || 'worker apply failed'));
        this.pendingApply = null;
      }
      return;
    }
    if (data.type === 'status') this.status = data.message || '';
  };

  start(simDt) {
    this.readyPromise = new Promise((resolve, reject) => { this.resolveReady = resolve; this.rejectReady = reject; });
    if (!window.Worker) {
      this.onStatusNote('Web Worker not supported');
      if (this.resolveReady) { this.resolveReady(false); this.resolveReady = this.rejectReady = null; }
      return null;
    }
    try {
      const worker = new Worker(window.__MCLD_INLINE_WORKER_URL__ || './simulation_worker.js');
      worker.onmessage = this._handleMessage;
      worker.onerror = (error) => {
        console.error('Worker failed.', error);
        this.onStatusNote(`Worker failed: ${error && error.message ? error.message : error}`);
      };
      worker.postMessage({ type: 'boot', simDt, bufferConfig: this.computeBufferConfig() });
      this.client = worker;
      return worker;
    } catch (error) {
      console.error('Worker start failed.', error);
      this.onStatusNote(`Worker start failed: ${error && error.message ? error.message : error}`);
      if (this.resolveReady) { this.resolveReady(false); this.resolveReady = this.rejectReady = null; }
      return null;
    }
  }

  waitUntilReady() {
    return this.readyPromise || Promise.resolve(this.ready);
  }

  reportDrain(consumed) {
    if (!this.client || !this.ready) return;
    const bufferConfig = this.computeBufferConfig();
    const needFill = !this.state.initializing && this.queueLength() < bufferConfig.lowWater && !this.fillRequested;
    if (!(consumed > 0) && !needFill) return;
    this.client.postMessage({
      type: 'drain',
      consumed: Math.max(0, consumed | 0),
      requestFill: needFill,
      queueLength: this.queueLength(),
      bufferConfig,
    });
    if (needFill) this.fillRequested = true;
  }

  requestFillIfNeeded() {
    this.reportDrain(0);
  }

  applyInit(cfg, savedState, l1Vis, l2Vis) {
    return new Promise((resolve, reject) => {
      if (!this.client) {
        reject(new Error('worker unavailable'));
        return;
      }
      this.pendingApply = { resolve, reject };
      this.fillRequested = false;
      this.client.postMessage({ type: 'applyInit', cfg, savedState, l1Vis, l2Vis, bufferConfig: this.computeBufferConfig() });
    });
  }
}

export { WorkerStream };
