
function fmtFixed(value, width, digits = 2) {
  const text = Number.isFinite(value) ? Number(value).toFixed(digits) : '--';
  return text.padStart(width, ' ');
}
function fmtExp(value, width, digits = 2) {
  const text = Number.isFinite(value) ? Number(value).toExponential(digits) : '--';
  return text.padStart(width, ' ');
}
function fmtInt(value, width) {
  const text = Number.isFinite(value) ? String(Math.max(0, Math.floor(value))) : '--';
  return text.padStart(width, ' ');
}
function rodModeLabel(fit) {
  const mode = fit && typeof fit.mode === 'string' ? fit.mode : '';
  if (mode === 'manual') return 'manual';
  if (mode === 'auto') return 'auto';
  return '---';
}
function buildHudText({ state, workerReady, simTime, queueLength, latestDiag, stepWarningText }) {
  const runState = state.playing ? 'run' : 'pause';
  const shellState = state.showPotentialShell ? 'on' : 'off';
  const fpsActual = Number.isFinite(state.actualFps) ? Math.round(state.actualFps) : 0;
  const fpsTarget = Number.isFinite(state.targetFps) && state.targetFps > 0 ? Math.round(state.targetFps) : 0;
  const fpsText = `${String(fpsActual).padStart(3, ' ')}/${fpsTarget > 0 ? String(fpsTarget).padStart(3, ' ') : ' --'}`;
  const line1 = `perf  fps:${fpsText}  queue:${fmtInt(queueLength, 3)}  shell:${shellState}`;
  const line2 = `sim   state:${runState.padEnd(5, ' ')}  t:${fmtFixed(simTime, 7, 2)}  speed:${fmtFixed(state.renderSpeed, 4, 2)}x  spin:${fmtFixed(-state.autoRotateZ, 5, 2)}`;
  const line3 = `solve H:${fmtExp(latestDiag?.H_cur, 11, 3)}  q1:${fmtExp(latestDiag?.q1_err, 10, 2)}  q2:${fmtExp(latestDiag?.q2_err, 10, 2)}  over_tol:${fmtInt(state.overThresholdCount, 4)}`;
  const line4 = `rod   r1:${rodModeLabel(state.rodFit1).padEnd(6, ' ')} loss:${fmtFixed(state.rodFit1?.loss, 8, 4)}  r2:${rodModeLabel(state.rodFit2).padEnd(6, ' ')} loss:${fmtFixed(state.rodFit2?.loss, 8, 4)}`;
  const warning = stepWarningText ? `warn  ${stepWarningText}` : '';
  return warning ? `${line1}
${line2}
${line3}
${line4}
${warning}` : `${line1}
${line2}
${line3}
${line4}`;
}
function updateHudOverlay(statusElement, context) {
  if (!statusElement) return;
  const nextText = buildHudText(context);
  if (statusElement.textContent !== nextText) statusElement.textContent = nextText;
}
export { buildHudText, updateHudOverlay };
