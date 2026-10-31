// Classic script for early WE callbacks; also safe as a main.js side-effect import.
(function (global) {
  if (global.__MCLD_WE_BRIDGE__) return;
  const keys = { general: 'mcld_we_general_props_cache', user: 'mcld_we_user_props_cache' };
  const methods = { general: 'applyGeneralProperties', user: 'applyUserProperties' };
  function read(kind) {
    try {
      const parsed = JSON.parse(global.localStorage.getItem(keys[kind]));
      return parsed && typeof parsed === 'object' && !Array.isArray(parsed) ? parsed : {};
    } catch (_) { return {}; }
  }
  const values = { general: read('general'), user: read('user') };
  let receiver = null;
  function accept(kind, patch) {
    if (!patch || typeof patch !== 'object' || Array.isArray(patch)) return;
    // With a receiver, persist after application succeeds. During bootstrap,
    // retain callbacks immediately so decoding/loading cannot lose them.
    if (receiver) receiver[methods[kind]](patch);
    values[kind] = { ...values[kind], ...patch };
    try { global.localStorage.setItem(keys[kind], JSON.stringify(values[kind])); } catch (_) {}
  }
  const listener = {
    applyGeneralProperties(properties) { accept('general', properties); },
    applyUserProperties(properties) { accept('user', properties); },
  };
  global.__MCLD_WE_BRIDGE__ = {
    listener,
    connect(next) {
      if (!next || typeof next.applyGeneralProperties !== 'function' || typeof next.applyUserProperties !== 'function') {
        throw new Error('Invalid WE property receiver');
      }
      receiver = next;
      if (Object.keys(values.general).length) receiver.applyGeneralProperties({ ...values.general });
      if (Object.keys(values.user).length) receiver.applyUserProperties({ ...values.user });
    },
  };
  global.wallpaperPropertyListener = listener;
})(window);
