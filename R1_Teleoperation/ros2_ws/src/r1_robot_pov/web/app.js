(() => {
  'use strict';

  const $ = (selector) => document.querySelector(selector);
  const ui = {
    app: $('#app'),
    viewport: $('#viewport'),
    canvas: $('#displayCanvas'),
    video: $('#webrtcVideo'),
    mjpeg: $('#mjpegImage'),
    connectionPill: $('#connectionPill'),
    connectionText: $('#connectionText'),
    source: $('#sourceValue'),
    resolution: $('#resolutionValue'),
    fps: $('#fpsValue'),
    latency: $('#latencyValue'),
    network: $('#networkValue'),
    reconnects: $('#reconnectValue'),
    dropped: $('#droppedValue'),
    mode: $('#modeValue'),
    stereo: $('#stereoValue'),
    webxr: $('#webxrValue'),
    modeHint: $('#modeHint'),
    noSignal: $('#noSignal'),
    noSignalTitle: $('#noSignalTitle'),
    noSignalDetail: $('#noSignalDetail'),
    retry: $('#retryButton'),
    profile: $('#profileSelect'),
    layout: $('#layoutSelect'),
    swap: $('#swapButton'),
    xr: $('#xrButton'),
    fullscreen: $('#fullscreenButton'),
    overlay: $('#overlayButton'),
    qr: $('#qrButton'),
    qrDialog: $('#qrDialog'),
    qrClose: $('#qrCloseButton'),
    viewerUrl: $('#viewerUrl'),
    toast: $('#toast'),
    xrCanvas: $('#xrCanvas'),
  };

  const ctx = ui.canvas.getContext('2d', {alpha: false, desynchronized: true});
  const PROFILE_LABELS = {
    high: 'High',
    balanced: 'Balanced',
    'low-latency': 'Low latency',
    'bad-wifi': 'Bad Wi-Fi',
  };
  const BACKOFF_MS = [1000, 2000, 4000, 8000, 16000, 30000];
  const PROFILE_ORDER = ['high', 'balanced', 'low-latency', 'bad-wifi'];
  const ADAPTIVE_COOLDOWN_MS = 30000;
  const DEFAULT_STALE_MS = 2500;
  const STATUS_PERIOD_MS = 1000;
  const METRICS_PERIOD_MS = 5000;

  const state = {
    config: {},
    profile: 'balanced',
    layout: 'auto',
    swapEyes: false,
    overlay: true,
    transport: 'connecting',
    peer: null,
    pendingPeer: null,
    generation: 0,
    connectBusy: false,
    retryAttempt: 0,
    retryTimer: 0,
    retryAt: 0,
    reconnects: 0,
    mjpegReady: false,
    lastMediaFrameAt: 0,
    lastWebRtcFrameAt: 0,
    lastStatusAt: 0,
    statusFailures: 0,
    serverSourceHealthy: null,
    sourceError: '',
    sourceName: '—',
    sourceFps: 0,
    lastSourceSequence: null,
    lastSourceSequenceAt: 0,
    serverReconnects: 0,
    serverDroppedFrames: 0,
    stats: {
      renderedFps: 0,
      bitrateKbps: 0,
      rttMs: null,
      jitterMs: null,
      packetsLost: 0,
      framesDropped: 0,
      mediaLatencyMs: null,
      framesDecoded: 0,
      lastBytes: 0,
      lastBytesAt: 0,
    },
    videoMonitorToken: 0,
    canvasFrames: 0,
    controlsTimer: 0,
    toastTimer: 0,
    xrSession: null,
    xrRenderer: null,
    clientId: makeClientId(),
    quality: {
      badSamples: 0,
      lastChangeAt: 0,
      lastPacketsLost: 0,
      lastFramesDropped: 0,
      reconnectTimes: [],
    },
  };

  function makeClientId() {
    const saved = safeStorageGet('r1-pov-client-id');
    if (saved) return saved;
    const suffix = globalThis.crypto && crypto.randomUUID
      ? crypto.randomUUID()
      : `${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;
    const id = `viewer-${suffix}`;
    safeStorageSet('r1-pov-client-id', id);
    return id;
  }

  function safeStorageGet(key) {
    try {
      return localStorage.getItem(key);
    } catch (_error) {
      return null;
    }
  }

  function safeStorageSet(key, value) {
    try {
      localStorage.setItem(key, value);
    } catch (_error) {
      // Private browsing can disable storage; viewer operation is unaffected.
    }
  }

  function finiteNumber(value) {
    if (value === null || value === undefined || value === '') return null;
    const parsed = Number(value);
    return Number.isFinite(parsed) ? parsed : null;
  }

  async function fetchJson(url, options = {}, timeoutMs = 7000) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), timeoutMs);
    try {
      const response = await fetch(url, {
        cache: 'no-store',
        ...options,
        signal: controller.signal,
      });
      if (!response.ok) {
        const detail = (await response.text()).trim();
        throw new Error(detail || `${response.status} ${response.statusText}`);
      }
      return await response.json();
    } finally {
      clearTimeout(timer);
    }
  }

  async function loadConfig() {
    try {
      state.config = await fetchJson('/api/config', {}, 4000);
    } catch (error) {
      state.config = {
        profile: 'balanced',
        layout: 'stereo',
        transport: 'auto',
        webrtcAvailable: true,
        mjpegAvailable: true,
        staleAfterMs: DEFAULT_STALE_MS,
      };
      showToast(`Конфигурация недоступна: ${friendlyError(error)}`);
    }

    populateProfiles(state.config.profiles);
    const query = new URLSearchParams(location.search);
    const requestedTransport = String(query.get('transport') || '').toLowerCase();
    if (['auto', 'webrtc', 'mjpeg'].includes(requestedTransport)) {
      state.config.transport = requestedTransport;
    }
    const configuredProfile = query.get('profile')
      || safeStorageGet('r1-pov-profile')
      || state.config.profile;
    if ([...ui.profile.options].some((option) => option.value === configuredProfile)) {
      state.profile = configuredProfile;
    }
    ui.profile.value = state.profile;

    const configuredLayout = query.get('layout')
      || query.get('mode')
      || safeStorageGet('r1-pov-layout')
      || (state.config.layout === 'mono' ? 'mono' : 'auto');
    state.layout = ['auto', 'mono', 'sbs', 'tb'].includes(configuredLayout)
      ? configuredLayout
      : 'auto';
    ui.layout.value = state.layout;

    const configuredOverlay = query.get('overlay');
    state.overlay = configuredOverlay === '0' || configuredOverlay === 'false'
      ? false
      : safeStorageGet('r1-pov-overlay') !== 'false';
    updateOverlayButton();

    const querySwap = query.get('swap');
    state.swapEyes = querySwap === '1' || querySwap === 'true'
      ? true
      : safeStorageGet('r1-pov-swap') === 'true';
    updateSwapButton();

    const advertised = state.config.mdnsUrl || state.config.numericUrl || location.href;
    ui.viewerUrl.textContent = advertised;
    const qrImage = ui.qrDialog.querySelector('.qr-frame img');
    if (advertised && advertised !== state.config.numericUrl) {
      qrImage.src = `/qr.png?url=${encodeURIComponent(advertised)}`;
    }
  }

  function populateProfiles(profiles) {
    const names = profiles && typeof profiles === 'object'
      ? Object.keys(profiles)
      : Object.keys(PROFILE_LABELS);
    const usable = names.filter((name) => name && typeof name === 'string');
    if (!usable.length) return;
    ui.profile.replaceChildren(...usable.map((name) => {
      const option = document.createElement('option');
      option.value = name;
      option.textContent = PROFILE_LABELS[name] || name;
      return option;
    }));
  }

  function shouldTryWebRtc() {
    const requested = String(state.config.transport || 'auto').toLowerCase();
    return requested !== 'mjpeg'
      && state.config.webrtcAvailable !== false
      && typeof RTCPeerConnection !== 'undefined';
  }

  function canUseMjpeg() {
    return state.config.mjpegAvailable !== false;
  }

  async function connectPreferred({manual = false} = {}) {
    if (state.connectBusy) return;
    state.connectBusy = true;
    clearRetry();
    if (manual) state.retryAttempt = 0;
    const generation = ++state.generation;
    const hasFallback = state.transport === 'mjpeg' && state.mjpegReady;
    if (!hasFallback) setConnectionState('connecting');

    try {
      if (!shouldTryWebRtc()) throw new Error('WebRTC отключён в профиле сервера');
      await connectWebRtc(generation);
      state.retryAttempt = 0;
    } catch (error) {
      if (generation !== state.generation) return;
      if (canUseMjpeg()) {
        activateMjpeg();
        showToast(`WebRTC недоступен — включён MJPEG: ${friendlyError(error)}`);
      } else {
        setConnectionState('offline');
        state.sourceError = friendlyError(error);
      }
      if (shouldTryWebRtc() || !canUseMjpeg()) scheduleReconnect();
    } finally {
      state.connectBusy = false;
      updateSignalState();
    }
  }

  async function connectWebRtc(generation) {
    closePeer(state.pendingPeer);
    state.pendingPeer = null;
    if (state.peer) {
      closePeer(state.peer);
      state.peer = null;
    }
    ui.video.srcObject = null;

    const peer = new RTCPeerConnection({
      iceServers: [],
      iceCandidatePoolSize: 0,
      bundlePolicy: 'max-bundle',
      rtcpMuxPolicy: 'require',
    });
    state.pendingPeer = peer;
    const remoteStream = new MediaStream();

    peer.addTransceiver('video', {direction: 'recvonly'});
    peer.ontrack = (event) => {
      const stream = event.streams && event.streams[0];
      if (stream) {
        ui.video.srcObject = stream;
      } else {
        remoteStream.addTrack(event.track);
        ui.video.srcObject = remoteStream;
      }
      event.track.onmute = () => {
        if (state.peer === peer) updateSignalState(true);
      };
      event.track.onended = () => {
        if (state.peer === peer) handleWebRtcLoss('Видеотрек остановлен');
      };
    };
    peer.onconnectionstatechange = () => {
      if (state.peer !== peer && state.pendingPeer !== peer) return;
      if (['failed', 'disconnected', 'closed'].includes(peer.connectionState)) {
        if (state.peer === peer) handleWebRtcLoss(`WebRTC: ${peer.connectionState}`);
      }
    };
    peer.oniceconnectionstatechange = () => {
      if (state.peer === peer && ['failed', 'disconnected'].includes(peer.iceConnectionState)) {
        handleWebRtcLoss(`ICE: ${peer.iceConnectionState}`);
      }
    };

    const offer = await peer.createOffer();
    await peer.setLocalDescription(offer);
    await waitForIceGathering(peer, 2500);
    if (generation !== state.generation) throw new Error('Подключение отменено');

    const local = peer.localDescription;
    const answer = await fetchJson('/offer', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({
        sdp: local.sdp,
        type: local.type,
        profile: state.profile,
        layout: serverLayout(state.layout),
      }),
    }, 9000);
    if (!answer || typeof answer.sdp !== 'string') {
      throw new Error('Сервер вернул пустой SDP answer');
    }
    await peer.setRemoteDescription({type: answer.type || 'answer', sdp: answer.sdp});
    await waitForFirstVideoFrame(generation, 9000);
    if (generation !== state.generation) throw new Error('Подключение отменено');

    state.pendingPeer = null;
    state.peer = peer;
    state.transport = 'webrtc';
    state.lastMediaFrameAt = performance.now();
    state.lastWebRtcFrameAt = state.lastMediaFrameAt;
    state.mjpegReady = false;
    ui.mjpeg.onload = null;
    ui.mjpeg.onerror = null;
    ui.mjpeg.removeAttribute('src');
    setConnectionState('live');
    startVideoFrameMonitor();
    ui.modeHint.textContent = 'WebRTC · локальная сеть · без облака';
  }

  function waitForIceGathering(peer, timeoutMs) {
    if (peer.iceGatheringState === 'complete') return Promise.resolve();
    return new Promise((resolve) => {
      let finished = false;
      const finish = () => {
        if (finished) return;
        finished = true;
        clearTimeout(timer);
        peer.removeEventListener('icegatheringstatechange', changed);
        resolve();
      };
      const changed = () => {
        if (peer.iceGatheringState === 'complete') finish();
      };
      const timer = setTimeout(finish, timeoutMs);
      peer.addEventListener('icegatheringstatechange', changed);
    });
  }

  function waitForFirstVideoFrame(generation, timeoutMs) {
    return new Promise((resolve, reject) => {
      let finished = false;
      const finish = (error) => {
        if (finished) return;
        finished = true;
        clearTimeout(timer);
        ui.video.removeEventListener('loadeddata', ready);
        error ? reject(error) : resolve();
      };
      const ready = () => {
        if (generation !== state.generation) return finish(new Error('Подключение отменено'));
        Promise.resolve(ui.video.play()).catch(() => {});
        if (ui.video.videoWidth > 0 && ui.video.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA) {
          finish();
        }
      };
      const timer = setTimeout(
        () => finish(new Error('WebRTC не получил первый видеокадр')),
        timeoutMs,
      );
      ui.video.addEventListener('loadeddata', ready);
      if (ui.video.requestVideoFrameCallback) {
        ui.video.requestVideoFrameCallback(() => finish());
      }
      ready();
    });
  }

  function startVideoFrameMonitor() {
    const token = ++state.videoMonitorToken;
    if (!ui.video.requestVideoFrameCallback) return;
    const frame = (_now, metadata) => {
      if (token !== state.videoMonitorToken || state.transport !== 'webrtc') return;
      const timestamp = performance.now();
      state.lastWebRtcFrameAt = timestamp;
      state.lastMediaFrameAt = timestamp;
      if (metadata && Number.isFinite(metadata.captureTime)) {
        const delay = metadata.expectedDisplayTime - metadata.captureTime;
        if (Number.isFinite(delay) && delay >= 0 && delay < 5000) {
          state.stats.mediaLatencyMs = delay;
        }
      }
      ui.video.requestVideoFrameCallback(frame);
    };
    ui.video.requestVideoFrameCallback(frame);
  }

  function closePeer(peer) {
    if (!peer) return;
    peer.onconnectionstatechange = null;
    peer.oniceconnectionstatechange = null;
    peer.ontrack = null;
    try {
      peer.getTransceivers().forEach((transceiver) => {
        if (transceiver.stop) transceiver.stop();
      });
      peer.close();
    } catch (_error) {
      // The peer may already be closed.
    }
  }

  function handleWebRtcLoss(reason) {
    if (state.transport !== 'webrtc') return;
    state.sourceError = reason;
    state.videoMonitorToken += 1;
    state.stats.renderedFps = 0;
    state.stats.bitrateKbps = 0;
    if (state.peer) closePeer(state.peer);
    state.peer = null;
    if (canUseMjpeg()) activateMjpeg();
    else setConnectionState('offline');
    scheduleReconnect();
    updateSignalState(true);
  }

  function activateMjpeg(force = false) {
    if (!force && state.transport === 'mjpeg' && ui.mjpeg.src) return;
    state.transport = 'mjpeg';
    state.mjpegReady = false;
    ui.mjpeg.onload = () => {
      state.mjpegReady = true;
      state.lastMediaFrameAt = performance.now();
      setConnectionState('live');
      updateSignalState();
    };
    ui.mjpeg.onerror = () => {
      state.mjpegReady = false;
      state.sourceError = 'MJPEG-поток недоступен';
      setConnectionState('offline');
      scheduleReconnect(true);
      updateSignalState(true);
    };
    ui.mjpeg.src = `/stream.mjpg?profile=${encodeURIComponent(state.profile)}&layout=${encodeURIComponent(serverLayout(state.layout))}&_=${Date.now()}`;
    setConnectionState('connecting');
    ui.modeHint.textContent = 'MJPEG fallback · локальная сеть';
  }

  function scheduleReconnect(restartMjpeg = false) {
    if (state.retryTimer || document.hidden) return;
    const index = Math.min(state.retryAttempt, BACKOFF_MS.length - 1);
    const delay = BACKOFF_MS[index];
    state.retryAttempt += 1;
    state.retryAt = Date.now() + delay;
    state.reconnects += 1;
    recordReconnectForAdaptiveQuality();
    state.retryTimer = setTimeout(() => {
      state.retryTimer = 0;
      if (restartMjpeg && state.transport === 'mjpeg') activateMjpeg(true);
      connectPreferred();
    }, delay);
  }

  function clearRetry() {
    if (state.retryTimer) clearTimeout(state.retryTimer);
    state.retryTimer = 0;
    state.retryAt = 0;
  }

  function setConnectionState(connection) {
    ui.connectionPill.classList.remove('is-live', 'is-connecting', 'is-offline');
    ui.connectionPill.classList.add(`is-${connection}`);
    const transport = state.transport === 'webrtc' ? 'WEBRTC' : 'MJPEG';
    if (connection === 'live') ui.connectionText.textContent = `LIVE · ${transport}`;
    else if (connection === 'offline') ui.connectionText.textContent = 'НЕТ СВЯЗИ';
    else ui.connectionText.textContent = 'ПОДКЛЮЧЕНИЕ';
  }

  function updateSignalState(forceLost = false) {
    const now = performance.now();
    const staleMs = finiteNumber(state.config.staleAfterMs) || DEFAULT_STALE_MS;
    const mediaFresh = state.lastMediaFrameAt > 0 && now - state.lastMediaFrameAt < Math.max(staleMs, 1600);
    const serverFresh = state.serverSourceHealthy !== false;
    // Android/Pico may report "offline" on a usable local Wi-Fi without WAN.
    // The local stream and health endpoint are the authoritative signals.
    const healthy = !forceLost && Boolean(activeSource())
      && mediaFresh && serverFresh;
    ui.app.classList.toggle('signal-lost', !healthy);

    if (healthy) {
      setConnectionState('live');
      return;
    }
    if (state.serverSourceHealthy === false) {
      ui.noSignalTitle.textContent = 'Нет сигнала от камеры';
      ui.noSignalDetail.textContent = friendlyVideoError(state.sourceError)
        || 'Viewer подключён, источник видео восстанавливается автоматически.';
      setConnectionState('offline');
      ui.connectionText.textContent = 'НЕТ СИГНАЛА';
      return;
    }
    if (state.transport === 'connecting' || state.connectBusy) {
      ui.noSignalTitle.textContent = 'Подключение к камере';
      ui.noSignalDetail.textContent = 'Проверяем WebRTC в локальной сети…';
      return;
    }
    ui.noSignalTitle.textContent = 'Видеопоток потерян';
    const seconds = Math.max(0, Math.ceil((state.retryAt - Date.now()) / 1000));
    ui.noSignalDetail.textContent = seconds > 0
      ? `Переподключение через ${seconds} с. Последний кадр скрыт для безопасности.`
      : (friendlyVideoError(state.sourceError) || 'Пытаемся восстановить соединение автоматически.');
  }

  function friendlyVideoError(error) {
    const text = String(error || '');
    if (text.includes('3102')) {
      return 'Камера робота пока не отвечает (3102). Повторяем подключение по Ethernet.';
    }
    return text;
  }

  function updateSwapButton() {
    ui.swap.setAttribute('aria-pressed', String(state.swapEyes));
    ui.swap.title = state.swapEyes ? 'Глаза поменяны местами' : 'Поменять левый и правый глаз';
  }

  function effectiveLayout(sourceWidth, sourceHeight) {
    if (state.layout === 'mono' || state.layout === 'sbs' || state.layout === 'tb') return state.layout;
    if (state.config.layout === 'mono') return 'mono';
    if (state.config.layout === 'top-bottom') return 'tb';
    if (state.config.layout === 'stereo') return 'sbs';
    return sourceWidth / Math.max(1, sourceHeight) > 2.0 ? 'sbs' : 'mono';
  }

  function serverLayout(layout) {
    if (layout === 'sbs') return 'stereo';
    if (layout === 'tb') return 'top-bottom';
    if (layout === 'auto') {
      if (state.config.layout === 'stereo') return 'stereo';
      if (state.config.layout === 'top-bottom') return 'top-bottom';
    }
    return 'mono';
  }

  function activeSource() {
    if (state.transport === 'webrtc'
        && ui.video.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA
        && ui.video.videoWidth > 0) {
      return {element: ui.video, width: ui.video.videoWidth, height: ui.video.videoHeight};
    }
    if (state.transport === 'mjpeg' && ui.mjpeg.naturalWidth > 0) {
      if (!state.mjpegReady) {
        state.mjpegReady = true;
        state.lastMediaFrameAt = performance.now();
      }
      return {element: ui.mjpeg, width: ui.mjpeg.naturalWidth, height: ui.mjpeg.naturalHeight};
    }
    return null;
  }

  function resizeCanvas() {
    const rect = ui.canvas.getBoundingClientRect();
    const dpr = Math.min(globalThis.devicePixelRatio || 1, 2);
    let width = Math.max(2, Math.round(rect.width * dpr));
    let height = Math.max(2, Math.round(rect.height * dpr));
    const scale = Math.min(1, 2560 / width, 1440 / height);
    width = Math.round(width * scale);
    height = Math.round(height * scale);
    if (ui.canvas.width !== width || ui.canvas.height !== height) {
      ui.canvas.width = width;
      ui.canvas.height = height;
      ctx.imageSmoothingEnabled = true;
      ctx.imageSmoothingQuality = 'high';
    }
  }

  function drawContained(source, sx, sy, sw, sh, dx, dy, dw, dh) {
    const sourceAspect = sw / sh;
    const targetAspect = dw / dh;
    let width;
    let height;
    let x;
    let y;
    if (sourceAspect > targetAspect) {
      width = dw;
      height = dw / sourceAspect;
      x = dx;
      y = dy + (dh - height) / 2;
    } else {
      height = dh;
      width = dh * sourceAspect;
      x = dx + (dw - width) / 2;
      y = dy;
    }
    ctx.drawImage(source, sx, sy, sw, sh, x, y, width, height);
  }

  function renderCanvasFrame() {
    resizeCanvas();
    const source = activeSource();
    if (source) {
      const width = ui.canvas.width;
      const height = ui.canvas.height;
      ctx.fillStyle = '#000';
      ctx.fillRect(0, 0, width, height);
      const layout = effectiveLayout(source.width, source.height);
      if (layout === 'sbs') {
        const eyeWidth = source.width / 2;
        const firstEye = state.swapEyes ? 1 : 0;
        const secondEye = state.swapEyes ? 0 : 1;
        drawContained(source.element, firstEye * eyeWidth, 0, eyeWidth, source.height,
          0, 0, width / 2, height);
        drawContained(source.element, secondEye * eyeWidth, 0, eyeWidth, source.height,
          width / 2, 0, width / 2, height);
      } else if (layout === 'tb') {
        const eyeHeight = source.height / 2;
        const firstEye = state.swapEyes ? 1 : 0;
        const secondEye = state.swapEyes ? 0 : 1;
        drawContained(source.element, 0, firstEye * eyeHeight, source.width, eyeHeight,
          0, 0, width, height / 2);
        drawContained(source.element, 0, secondEye * eyeHeight, source.width, eyeHeight,
          0, height / 2, width, height / 2);
      } else {
        drawContained(source.element, 0, 0, source.width, source.height, 0, 0, width, height);
      }
      state.canvasFrames += 1;
    }
  }

  function render() {
    renderCanvasFrame();
    requestAnimationFrame(render);
  }

  async function pollStatus() {
    try {
      const status = await fetchJson('/api/status', {}, 2500);
      const now = performance.now();
      state.lastStatusAt = now;
      state.statusFailures = 0;
      const source = status.source && typeof status.source === 'object' ? status.source : {};
      state.sourceName = source.source || state.config.source || status.mode || '—';
      const stale = source.stale === true;
      state.serverSourceHealthy = source.connected === true && !stale && source.has_frame !== false;
      state.sourceError = activeSourceError(source);

      const sequence = finiteNumber(source.sequence);
      if (sequence !== null) {
        if (state.lastSourceSequence !== null && state.lastSourceSequenceAt > 0) {
          const elapsed = (now - state.lastSourceSequenceAt) / 1000;
          if (elapsed > 0) state.sourceFps = Math.max(0, (sequence - state.lastSourceSequence) / elapsed);
        }
        if (sequence !== state.lastSourceSequence) state.lastMediaFrameAt = now;
        state.lastSourceSequence = sequence;
        state.lastSourceSequenceAt = now;
      }

      const receivedAge = finiteNumber(source.received_age_s);
      if (receivedAge !== null && receivedAge * 1000 > (finiteNumber(state.config.staleAfterMs) || DEFAULT_STALE_MS)) {
        state.serverSourceHealthy = false;
      }
      state.serverReconnects = sourceReconnectCount(source);
      state.serverDroppedFrames = finiteNumber(source.dropped_frames) || 0;
      if (state.transport === 'mjpeg') {
        const bitrate = finiteNumber(status.mjpeg_bitrate_kbps);
        if (bitrate !== null) state.stats.bitrateKbps = bitrate;
      }
      updateTelemetry(status);
      updateStereoStatus();
    } catch (error) {
      state.statusFailures += 1;
      if (state.statusFailures >= 3) {
        state.sourceError = `Сервер недоступен: ${friendlyError(error)}`;
        state.serverSourceHealthy = false;
        scheduleReconnect(state.transport === 'mjpeg');
      }
    }
    updateSignalState();
  }

  function activeSourceError(source) {
    if (source.error) return String(source.error);
    const sources = source.sources && typeof source.sources === 'object'
      ? Object.values(source.sources)
      : [];
    const failed = sources.find((item) => item && (item.error || item.last_error));
    return failed ? String(failed.error || failed.last_error) : '';
  }

  function sourceReconnectCount(source) {
    if (!source.sources || typeof source.sources !== 'object') return 0;
    return Object.values(source.sources).reduce((sum, item) => {
      return sum + (finiteNumber(item && item.reconnects) || 0);
    }, 0);
  }

  async function pollPeerStats() {
    const peer = state.peer;
    if (!peer || state.transport !== 'webrtc') return;
    try {
      const report = await peer.getStats();
      let inbound = null;
      let candidate = null;
      report.forEach((entry) => {
        if (entry.type === 'inbound-rtp' && entry.kind === 'video' && !entry.isRemote) inbound = entry;
        if (entry.type === 'candidate-pair' && entry.state === 'succeeded'
            && (entry.nominated || entry.selected)) candidate = entry;
      });
      const now = performance.now();
      let packetLossDelta = 0;
      let frameDropDelta = 0;
      if (inbound) {
        const decoded = finiteNumber(inbound.framesDecoded) || 0;
        if (decoded > state.stats.framesDecoded) state.lastMediaFrameAt = now;
        state.stats.framesDecoded = decoded;
        state.stats.renderedFps = finiteNumber(inbound.framesPerSecond) || state.stats.renderedFps;
        state.stats.jitterMs = finiteNumber(inbound.jitter) === null
          ? state.stats.jitterMs
          : inbound.jitter * 1000;
        const packetsLost = finiteNumber(inbound.packetsLost) || 0;
        const framesDropped = finiteNumber(inbound.framesDropped) || 0;
        packetLossDelta = Math.max(0, packetsLost - state.quality.lastPacketsLost);
        frameDropDelta = Math.max(0, framesDropped - state.quality.lastFramesDropped);
        state.quality.lastPacketsLost = packetsLost;
        state.quality.lastFramesDropped = framesDropped;
        state.stats.packetsLost = packetsLost;
        state.stats.framesDropped = framesDropped;
        const bytes = finiteNumber(inbound.bytesReceived);
        if (bytes !== null && state.stats.lastBytesAt > 0 && now > state.stats.lastBytesAt) {
          state.stats.bitrateKbps = Math.max(
            0,
            (bytes - state.stats.lastBytes) * 8 / (now - state.stats.lastBytesAt),
          );
        }
        if (bytes !== null) {
          state.stats.lastBytes = bytes;
          state.stats.lastBytesAt = now;
        }
      }
      if (candidate && finiteNumber(candidate.currentRoundTripTime) !== null) {
        state.stats.rttMs = candidate.currentRoundTripTime * 1000;
      }
      evaluateAdaptiveQuality(packetLossDelta, frameDropDelta);
    } catch (_error) {
      // Stats are optional and can briefly fail while ICE changes state.
    }
  }

  function evaluateAdaptiveQuality(packetLossDelta, frameDropDelta) {
    if (state.config.adaptive !== true || state.transport !== 'webrtc') return;
    const highRtt = state.stats.rttMs !== null && state.stats.rttMs > 250;
    const highJitter = state.stats.jitterMs !== null && state.stats.jitterMs > 80;
    const losingPackets = packetLossDelta >= 3;
    const droppingFrames = frameDropDelta >= 3;
    if (highRtt || highJitter || losingPackets || droppingFrames) {
      state.quality.badSamples += 1;
    } else {
      state.quality.badSamples = Math.max(0, state.quality.badSamples - 1);
    }
    if (state.quality.badSamples < 3) return;
    const reasons = [];
    if (highRtt) reasons.push(`RTT ${Math.round(state.stats.rttMs)} ms`);
    if (highJitter) reasons.push(`jitter ${Math.round(state.stats.jitterMs)} ms`);
    if (losingPackets) reasons.push(`потеря ${packetLossDelta} пак.`);
    if (droppingFrames) reasons.push(`drop ${frameDropDelta} кадр.`);
    downgradeProfile(reasons.join(', ') || 'нестабильный видеопоток');
  }

  function recordReconnectForAdaptiveQuality() {
    if (state.config.adaptive !== true) return;
    const now = Date.now();
    state.quality.reconnectTimes.push(now);
    state.quality.reconnectTimes = state.quality.reconnectTimes.filter(
      (timestamp) => now - timestamp <= 20000,
    );
    if (state.quality.reconnectTimes.length >= 3) {
      downgradeProfile('повторные переподключения', true);
    }
  }

  function downgradeProfile(reason, reconnecting = false) {
    if (state.config.adaptive !== true) return false;
    const now = Date.now();
    if (now - state.quality.lastChangeAt < ADAPTIVE_COOLDOWN_MS) return false;
    const available = new Set([...ui.profile.options].map((option) => option.value));
    const currentIndex = PROFILE_ORDER.indexOf(state.profile);
    if (currentIndex < 0) return false;
    const next = PROFILE_ORDER.slice(currentIndex + 1).find((name) => available.has(name));
    if (!next) return false;
    const previous = state.profile;
    state.profile = next;
    ui.profile.value = next;
    safeStorageSet('r1-pov-profile', next);
    state.quality.lastChangeAt = now;
    state.quality.badSamples = 0;
    state.quality.reconnectTimes = [];
    showToast(
      `Сеть нестабильна (${reason}). ${PROFILE_LABELS[previous] || previous} → ${PROFILE_LABELS[next] || next}`,
      6500,
    );
    // During an outage the next scheduled attempt will already use the new
    // profile.  Restarting immediately would reset the exponential backoff and
    // create a reconnect storm while the LAN endpoint is unavailable.
    if (!reconnecting) resetForProfile();
    return true;
  }

  function estimatedLatencyMs() {
    return state.stats.mediaLatencyMs !== null
      ? state.stats.mediaLatencyMs
      : (state.stats.rttMs !== null
        ? state.stats.rttMs / 2 + (state.stats.jitterMs || 0)
        : null);
  }

  function updateTelemetry(status = {}) {
    const source = activeSource();
    const fps = state.stats.renderedFps || state.sourceFps;
    const latency = estimatedLatencyMs();
    ui.source.textContent = compactSourceName(state.sourceName);
    ui.resolution.textContent = source
      ? `${source.width}×${source.height}`
      : '—';
    ui.fps.textContent = fps > 0 ? fps.toFixed(fps >= 10 ? 0 : 1) : '—';
    ui.latency.textContent = latency !== null ? `${Math.round(latency)} ms` : '—';
    ui.network.textContent = networkLabel();
    ui.reconnects.textContent = String(state.reconnects + state.serverReconnects);
    ui.dropped.textContent = String(
      Math.round(state.stats.framesDropped + state.serverDroppedFrames),
    );
    const layout = effectiveLayout(
      ui.video.videoWidth || ui.mjpeg.naturalWidth || 2,
      ui.video.videoHeight || ui.mjpeg.naturalHeight || 1,
    );
    const layoutLabel = layout === 'sbs' ? 'SBS' : layout === 'tb' ? 'TOP-BOTTOM' : 'MONO';
    ui.mode.textContent = `${layoutLabel} · ${state.profile}`;
    ui.stereo.textContent = state.config.stereoFallback === true
      ? 'MONO → ОБА ГЛАЗА'
      : (state.config.stereoAvailable === true ? 'STEREO' : '—');
    ui.webxr.textContent = state.xrSession
      ? 'ACTIVE'
      : (navigator.xr ? 'READY' : 'FALLBACK');
    if (status.profile && !ui.profile.matches(':focus') && !state.profile) {
      ui.profile.value = status.profile;
    }
  }

  function updateStereoStatus() {
    const fallback = state.config.stereoFallback === true;
    const label = fallback ? 'MONO → ОБА ГЛАЗА' : 'STEREO';
    if (state.config.stereoStatus) {
      ui.mode.title = state.config.stereoStatus;
    }
    // Keep the compact telemetry readable while exposing the distinction in
    // the mode field and browser accessibility tree.
    ui.mode.setAttribute('aria-label', state.config.stereoStatus || label);
  }

  function compactSourceName(value) {
    const text = String(value || '—');
    if (text === 'ros') return 'ROS CAMERA';
    if (text === 'mock') return 'MOCK';
    if (text === 'usb') return 'USB CAMERA';
    if (text === 'rtsp') return 'RTSP';
    return text.toUpperCase();
  }

  function networkLabel() {
    if (!navigator.onLine) return 'LAN LOCAL';
    const bitrate = state.stats.bitrateKbps;
    const rtt = state.stats.rttMs;
    if (bitrate > 0 && rtt !== null) return `${(bitrate / 1000).toFixed(1)}M · ${Math.round(rtt)}ms`;
    if (bitrate > 0) return `${(bitrate / 1000).toFixed(1)} Mbit/s`;
    const connection = navigator.connection || navigator.mozConnection || navigator.webkitConnection;
    if (connection && finiteNumber(connection.downlink) !== null) return `${connection.downlink} Mbit/s`;
    return 'LAN';
  }

  async function sendMetrics(useBeacon = false) {
    const frameAge = state.lastMediaFrameAt
      ? Math.max(0, performance.now() - state.lastMediaFrameAt)
      : null;
    const payload = {
      clientId: state.clientId,
      transport: state.transport,
      profile: state.profile,
      layout: state.layout,
      renderedFps: state.stats.renderedFps || state.sourceFps || null,
      latencyMs: estimatedLatencyMs(),
      rttMs: state.stats.rttMs,
      jitterMs: state.stats.jitterMs,
      packetsLost: state.stats.packetsLost,
      framesDropped: state.stats.framesDropped,
      frameAgeMs: frameAge,
      reconnects: state.reconnects,
      noSignal: ui.app.classList.contains('signal-lost'),
      online: navigator.onLine,
      visible: !document.hidden,
    };
    const body = JSON.stringify(payload);
    if (useBeacon && navigator.sendBeacon) {
      navigator.sendBeacon('/api/client-metrics', new Blob([body], {type: 'application/json'}));
      return;
    }
    try {
      await fetch('/api/client-metrics', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body,
        keepalive: true,
      });
    } catch (_error) {
      // Telemetry must never interrupt video recovery.
    }
  }

  function resetForProfile() {
    clearRetry();
    state.generation += 1;
    state.retryAttempt = 0;
    state.stats.renderedFps = 0;
    state.stats.bitrateKbps = 0;
    state.stats.lastBytes = 0;
    state.stats.lastBytesAt = 0;
    state.stats.rttMs = null;
    state.stats.jitterMs = null;
    state.quality.lastPacketsLost = 0;
    state.quality.lastFramesDropped = 0;
    state.quality.badSamples = 0;
    state.quality.reconnectTimes = [];
    if (state.pendingPeer) closePeer(state.pendingPeer);
    if (state.peer) closePeer(state.peer);
    state.pendingPeer = null;
    state.peer = null;
    state.videoMonitorToken += 1;
    ui.video.srcObject = null;
    state.transport = 'connecting';
    state.lastMediaFrameAt = 0;
    state.mjpegReady = false;
    ui.mjpeg.onload = null;
    ui.mjpeg.onerror = null;
    ui.mjpeg.removeAttribute('src');
    // Defer so a profile downgrade triggered inside a failed connection can let
    // that connection's finally block release the in-flight guard first.
    setTimeout(() => connectPreferred({manual: true}), 0);
  }

  function friendlyError(error) {
    if (!error) return 'неизвестная ошибка';
    if (error.name === 'AbortError') return 'превышено время ожидания';
    return String(error.message || error).replace(/\s+/g, ' ').slice(0, 180);
  }

  function showToast(message, duration = 4200) {
    clearTimeout(state.toastTimer);
    ui.toast.textContent = message;
    ui.toast.classList.add('visible');
    state.toastTimer = setTimeout(() => ui.toast.classList.remove('visible'), duration);
  }

  async function toggleFullscreen() {
    try {
      if (document.fullscreenElement) await document.exitFullscreen();
      else await document.documentElement.requestFullscreen({navigationUI: 'hide'});
    } catch (error) {
      showToast(`Полноэкранный режим недоступен: ${friendlyError(error)}`);
    }
  }

  async function enterWebXr() {
    if (state.xrSession) {
      await state.xrSession.end();
      return;
    }
    let session = null;
    try {
      if (!navigator.xr) throw new Error('WebXR не поддерживается этим браузером');
      const supported = await navigator.xr.isSessionSupported('immersive-vr');
      if (!supported) throw new Error('Режим immersive-vr недоступен');
      session = await navigator.xr.requestSession('immersive-vr', {
        optionalFeatures: ['local-floor'],
      });
      await startXrRenderer(session);
      showToast('WebXR включён. Нажмите VR ещё раз, чтобы выйти.');
    } catch (error) {
      if (session && state.xrSession !== session) {
        try { await session.end(); } catch (_endError) { /* already ended */ }
      }
      await fallbackToFullscreenSbs(error);
    }
  }

  async function fallbackToFullscreenSbs(error) {
    state.layout = 'sbs';
    ui.layout.value = 'sbs';
    safeStorageSet('r1-pov-layout', 'sbs');
    if (state.transport === 'mjpeg') {
      activateMjpeg(true);
    } else {
      resetForProfile();
    }
    showToast(`WebXR недоступен — включён fullscreen SBS. ${friendlyError(error)}`, 6000);
    if (!document.fullscreenElement) {
      try {
        await document.documentElement.requestFullscreen({navigationUI: 'hide'});
      } catch (fullscreenError) {
        showToast(
          `SBS включён, но fullscreen недоступен: ${friendlyError(fullscreenError)}`,
          6000,
        );
      }
    }
  }

  function updateOverlayButton() {
    ui.overlay.setAttribute('aria-pressed', String(state.overlay));
    ui.overlay.title = state.overlay ? 'Скрыть диагностический overlay' : 'Показать диагностический overlay';
    ui.app.classList.toggle('overlay-hidden', !state.overlay);
  }

  async function startXrRenderer(session) {
    const renderer = createXrRenderer(ui.xrCanvas);
    if (!renderer) throw new Error('WebGL для WebXR недоступен');
    await renderer.gl.makeXRCompatible();
    session.updateRenderState({baseLayer: new XRWebGLLayer(session, renderer.gl)});
    state.xrSession = session;
    state.xrRenderer = renderer;
    ui.xr.classList.add('accent');
    ui.xr.querySelector('span').textContent = 'Выйти VR';

    session.addEventListener('end', () => {
      state.xrSession = null;
      state.xrRenderer = null;
      ui.xr.querySelector('span').textContent = 'VR';
    }, {once: true});

    const draw = (_time, frame) => {
      if (state.xrSession !== session) return;
      const layer = session.renderState.baseLayer;
      const gl = renderer.gl;
      renderCanvasFrame();
      gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
      renderer.upload(ui.canvas);
      const pose = frame.getViewerPose(renderer.referenceSpace);
      // A viewer pose is not required for the head-locked quad. Render all XR views
      // from the current framebuffer dimensions when the runtime omits a pose.
      const views = pose ? pose.views : [{eye: 'left'}, {eye: 'right'}];
      views.forEach((view, index) => {
        const viewport = pose
          ? layer.getViewport(view)
          : {
              x: index * layer.framebufferWidth / 2,
              y: 0,
              width: layer.framebufferWidth / 2,
              height: layer.framebufferHeight,
            };
        const eye = view.eye === 'right' ? 1 : 0;
        renderer.draw(
          viewport,
          eye,
          effectiveLayout(ui.canvas.width, ui.canvas.height),
        );
      });
      session.requestAnimationFrame(draw);
    };
    renderer.referenceSpace = await session.requestReferenceSpace('viewer');
    session.requestAnimationFrame(draw);
  }

  function createXrRenderer(canvas) {
    const gl = canvas.getContext('webgl', {alpha: false, antialias: false, xrCompatible: true});
    if (!gl) return null;
    const vertex = compileShader(gl, gl.VERTEX_SHADER, `
      attribute vec2 position;
      attribute vec2 texcoord;
      varying vec2 uv;
      void main() { uv = texcoord; gl_Position = vec4(position, 0.0, 1.0); }
    `);
    const fragment = compileShader(gl, gl.FRAGMENT_SHADER, `
      precision mediump float;
      varying vec2 uv;
      uniform sampler2D image;
      uniform vec2 uvScale;
      uniform vec2 uvOffset;
      void main() { gl_FragColor = texture2D(image, uv * uvScale + uvOffset); }
    `);
    if (!vertex || !fragment) return null;
    const program = gl.createProgram();
    gl.attachShader(program, vertex);
    gl.attachShader(program, fragment);
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) return null;
    gl.useProgram(program);
    const buffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
      -1, -1, 0, 1, 1, -1, 1, 1, -1, 1, 0, 0,
      -1, 1, 0, 0, 1, -1, 1, 1, 1, 1, 1, 0,
    ]), gl.STATIC_DRAW);
    const position = gl.getAttribLocation(program, 'position');
    const texcoord = gl.getAttribLocation(program, 'texcoord');
    gl.enableVertexAttribArray(position);
    gl.enableVertexAttribArray(texcoord);
    gl.vertexAttribPointer(position, 2, gl.FLOAT, false, 16, 0);
    gl.vertexAttribPointer(texcoord, 2, gl.FLOAT, false, 16, 8);
    const texture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, true);
    const scaleLocation = gl.getUniformLocation(program, 'uvScale');
    const offsetLocation = gl.getUniformLocation(program, 'uvOffset');
    return {
      gl,
      referenceSpace: null,
      upload(source) {
        gl.bindTexture(gl.TEXTURE_2D, texture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, source);
      },
      draw(viewport, eye, layout) {
        gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
        if (layout === 'sbs') {
          gl.uniform2f(scaleLocation, 0.5, 1.0);
          gl.uniform2f(offsetLocation, eye * 0.5, 0.0);
        } else if (layout === 'tb') {
          gl.uniform2f(scaleLocation, 1.0, 0.5);
          gl.uniform2f(offsetLocation, 0.0, eye === 0 ? 0.5 : 0.0);
        } else {
          gl.uniform2f(scaleLocation, 1.0, 1.0);
          gl.uniform2f(offsetLocation, 0.0, 0.0);
        }
        gl.drawArrays(gl.TRIANGLES, 0, 6);
      },
    };
  }

  function compileShader(gl, type, source) {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, source);
    gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) return null;
    return shader;
  }

  function revealControls() {
    ui.app.classList.remove('controls-hidden');
    clearTimeout(state.controlsTimer);
    state.controlsTimer = setTimeout(() => {
      if (!ui.app.classList.contains('signal-lost') && !ui.qrDialog.open) {
        ui.app.classList.add('controls-hidden');
      }
    }, 5000);
  }

  function bindEvents() {
    ui.retry.addEventListener('click', () => {
      state.sourceError = '';
      connectPreferred({manual: true});
    });
    ui.profile.addEventListener('change', () => {
      state.profile = ui.profile.value;
      safeStorageSet('r1-pov-profile', state.profile);
      showToast(`Профиль: ${PROFILE_LABELS[state.profile] || state.profile}`);
      resetForProfile();
    });
    ui.layout.addEventListener('change', () => {
      state.layout = ui.layout.value;
      safeStorageSet('r1-pov-layout', state.layout);
      updateTelemetry();
    });
    ui.swap.addEventListener('click', () => {
      state.swapEyes = !state.swapEyes;
      safeStorageSet('r1-pov-swap', String(state.swapEyes));
      updateSwapButton();
      showToast(state.swapEyes ? 'Левый и правый глаз поменяны' : 'Обычный порядок глаз');
    });
    ui.fullscreen.addEventListener('click', toggleFullscreen);
    ui.overlay.addEventListener('click', () => {
      state.overlay = !state.overlay;
      safeStorageSet('r1-pov-overlay', String(state.overlay));
      updateOverlayButton();
      showToast(state.overlay ? 'Диагностика включена' : 'Immersive view: overlay скрыт');
    });
    ui.xr.addEventListener('click', enterWebXr);
    ui.qr.addEventListener('click', () => {
      if (ui.qrDialog.showModal) ui.qrDialog.showModal();
      else ui.qrDialog.setAttribute('open', '');
    });
    ui.qrClose.addEventListener('click', () => ui.qrDialog.close());
    ui.qrDialog.addEventListener('click', (event) => {
      if (event.target === ui.qrDialog) ui.qrDialog.close();
    });
    document.addEventListener('fullscreenchange', () => {
      ui.fullscreen.querySelector('span').textContent = document.fullscreenElement
        ? 'Выйти'
        : 'На весь экран';
    });
    ['pointermove', 'pointerdown', 'touchstart'].forEach((event) => {
      document.addEventListener(event, revealControls, {passive: true});
    });
    document.addEventListener('keydown', (event) => {
      if (['SELECT', 'INPUT'].includes(document.activeElement && document.activeElement.tagName)) return;
      if (event.key.toLowerCase() === 'f') toggleFullscreen();
      if (event.key.toLowerCase() === 'r') connectPreferred({manual: true});
      if (event.key.toLowerCase() === 's') ui.swap.click();
      if (event.key.toLowerCase() === 'o') ui.overlay.click();
    });
    window.addEventListener('online', () => connectPreferred({manual: true}));
    window.addEventListener('offline', updateSignalState);
    document.addEventListener('visibilitychange', () => {
      if (!document.hidden) {
        revealControls();
        if (!state.lastMediaFrameAt
            || performance.now() - state.lastMediaFrameAt > DEFAULT_STALE_MS) {
          connectPreferred({manual: true});
        }
      }
    });
    window.addEventListener('beforeunload', () => {
      sendMetrics(true);
      clearRetry();
      if (state.peer) closePeer(state.peer);
      if (state.pendingPeer) closePeer(state.pendingPeer);
    });
  }

  async function boot() {
    bindEvents();
    await loadConfig();
    render();
    await pollStatus();
    connectPreferred();
    setInterval(pollStatus, STATUS_PERIOD_MS);
    setInterval(() => {
      pollPeerStats();
      updateTelemetry();
      updateSignalState();
    }, 1000);
    setInterval(sendMetrics, METRICS_PERIOD_MS);
    revealControls();
  }

  boot().catch((error) => {
    state.sourceError = friendlyError(error);
    setConnectionState('offline');
    updateSignalState(true);
  });
})();
