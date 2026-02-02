const canvas = document.getElementById("grid");
const ctx = canvas.getContext("2d");

const xInput = document.getElementById("xInput");
const yInput = document.getElementById("yInput");
const distInput = document.getElementById("distInput");
const angleInput = document.getElementById("angleInput");
const scaleInput = document.getElementById("scaleInput");
const rotateSelect = document.getElementById("rotateSelect");
const thetaOverrideInput = document.getElementById("thetaOverrideInput");
const modeSelect = document.getElementById("modeSelect");

const targetReadout = document.getElementById("targetReadout");
const distReadout = document.getElementById("distReadout");
const angleReadout = document.getElementById("angleReadout");
const thetaReadout = document.getElementById("thetaReadout");
const statusConn = document.getElementById("statusConn");
const statusPose = document.getElementById("statusPose");
const resultEl = document.getElementById("result");

const zoomIn = document.getElementById("zoomIn");
const zoomOut = document.getElementById("zoomOut");
const sendBtn = document.getElementById("sendBtn");
const clearBtn = document.getElementById("clearBtn");
const cancelBtn = document.getElementById("cancelBtn");
const reverseToggle = document.getElementById("reverseToggle");

const GRID_PX = 40;
const MODE_ROBOT = "robot";
const MODE_MAP = "map";

// Target luôn lưu ở robot-frame: x=right(+), y=forward(+), meters
let targetRel = { x: 0, y: 0 };

let scale = parseFloat(scaleInput.value) || 0.5;
let syncing = false;

// Pose từ /api/state: {x,y,theta,thetaDeg,mapId}
let currentPose = null;

// ===== MAP VIEW =====
let viewOrigin = { x: 0, y: 0 };   // map coordinate tại tâm canvas
let viewInitialized = false;

// Map mode:
// false => grid đứng yên, robot chạy trên grid (khuyên dùng)
// true  => camera bám robot (robot luôn giữa)
let followRobot = false;

function round(value, decimals = 2) {
  const factor = Math.pow(10, decimals);
  return Math.round(value * factor) / factor;
}

function metersPerPixel() {
  return scale / GRID_PX;
}

function pixelsPerMeter() {
  return GRID_PX / scale;
}

function parseThetaOverrideDeg() {
  const raw = thetaOverrideInput.value.trim();
  if (!raw) return null;
  const value = parseFloat(raw);
  if (Number.isNaN(value)) return null;
  return value;
}

function getMode() {
  const mode = (modeSelect?.value || MODE_ROBOT).toLowerCase();
  return mode === MODE_MAP ? MODE_MAP : MODE_ROBOT;
}

// Convert robot-frame vector -> map delta using pose.theta
function robotVectorToMapDelta(pose, xRight, yForward) {
  // backend uses forward=y, left=-x
  const forward = yForward;
  const left = -xRight;
  const ct = Math.cos(pose.theta);
  const st = Math.sin(pose.theta);
  const dx = forward * ct - left * st;
  const dy = forward * st + left * ct;
  return { dx, dy };
}

// Convert map delta -> robot-frame vector
function mapDeltaToRobotVector(pose, dx, dy) {
  const ct = Math.cos(pose.theta);
  const st = Math.sin(pose.theta);
  const forward = dx * ct + dy * st;
  const left = -dx * st + dy * ct;
  const xRight = -left;
  const yForward = forward;
  return { xRight, yForward };
}

function mapToCanvas(center, mapX, mapY, pxPerMeter) {
  const cx = center.x + (mapX - viewOrigin.x) * pxPerMeter;
  const cy = center.y - (mapY - viewOrigin.y) * pxPerMeter;
  return { x: cx, y: cy };
}

function computeAutoEndThetaRad() {
  // 0=forward, +right (robot-frame)
  if (targetRel.x === 0 && targetRel.y === 0) return 0;
  if (reverseToggle.checked) return Math.atan2(-targetRel.x, -targetRel.y);
  return Math.atan2(targetRel.x, targetRel.y);
}

function computeEndThetaRad() {
  const override = parseThetaOverrideDeg();
  if (override !== null) return (override * Math.PI) / 180;
  return computeAutoEndThetaRad();
}

function syncPolarInputs() {
  const distance = Math.hypot(targetRel.x, targetRel.y);
  const angle = (Math.atan2(targetRel.x, targetRel.y) * 180) / Math.PI;
  distInput.value = round(distance);
  angleInput.value = round(angle, 1);
}

function syncXYInputsForCurrentMode() {
  const mode = getMode();

  if (mode === MODE_MAP) {
    // X/Y are mapX/mapY absolute
    if (currentPose) {
      const { dx, dy } = robotVectorToMapDelta(currentPose, targetRel.x, targetRel.y);
      xInput.value = round(currentPose.x + dx);
      yInput.value = round(currentPose.y + dy);
    }
  } else {
    // X/Y are robot-frame relative x/y
    xInput.value = round(targetRel.x);
    yInput.value = round(targetRel.y);
  }
}

function updateReadouts() {
  const mode = getMode();

  const distance = Math.hypot(targetRel.x, targetRel.y);
  const angle = (Math.atan2(targetRel.x, targetRel.y) * 180) / Math.PI;
  const endTheta = (computeEndThetaRad() * 180) / Math.PI;

  if (mode === MODE_MAP) {
    if (currentPose) {
      const { dx, dy } = robotVectorToMapDelta(currentPose, targetRel.x, targetRel.y);
      const mapX = currentPose.x + dx;
      const mapY = currentPose.y + dy;
      targetReadout.textContent = `mapX=${round(mapX)}, mapY=${round(mapY)}`;
    } else {
      targetReadout.textContent = `mapX=?, mapY=? (no pose)`;
    }
  } else {
    targetReadout.textContent = `x=${round(targetRel.x)}, y=${round(targetRel.y)}`;
  }

  distReadout.textContent = `${round(distance)} m`;
  angleReadout.textContent = `${round(angle, 1)} deg`;
  thetaReadout.textContent = `${round(endTheta, 1)} deg`;
}

function setTargetRel(nextX, nextY) {
  targetRel = { x: nextX, y: nextY };
  syncXYInputsForCurrentMode();
  syncPolarInputs();
  updateReadouts();
  draw();
}

function syncFromXY() {
  if (syncing) return;
  syncing = true;

  const mode = getMode();
  const xVal = parseFloat(xInput.value) || 0;
  const yVal = parseFloat(yInput.value) || 0;

  if (mode === MODE_MAP) {
    // mapX/mapY -> targetRel
    if (currentPose) {
      const dx = xVal - currentPose.x;
      const dy = yVal - currentPose.y;
      const { xRight, yForward } = mapDeltaToRobotVector(currentPose, dx, dy);
      targetRel = { x: xRight, y: yForward };
      syncPolarInputs();
    }
  } else {
    // robot-frame x/y
    targetRel = { x: xVal, y: yVal };
    syncPolarInputs();
  }

  updateReadouts();
  draw();
  syncing = false;
}

function syncFromPolar() {
  if (syncing) return;
  syncing = true;

  const distance = parseFloat(distInput.value) || 0;
  const angleDeg = parseFloat(angleInput.value) || 0;
  const angleRad = (angleDeg * Math.PI) / 180;

  const xVal = distance * Math.sin(angleRad);
  const yVal = distance * Math.cos(angleRad);
  targetRel = { x: xVal, y: yVal };

  syncXYInputsForCurrentMode();
  updateReadouts();
  draw();

  syncing = false;
}

function updateScale() {
  scale = parseFloat(scaleInput.value) || 0.5;
  if (scale <= 0) {
    scale = 0.5;
    scaleInput.value = scale;
  }
  draw();
}

function drawArrow(fromX, fromY, toX, toY, color) {
  const angle = Math.atan2(toY - fromY, toX - fromX);
  const headLength = 12;
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 2;

  ctx.beginPath();
  ctx.moveTo(fromX, fromY);
  ctx.lineTo(toX, toY);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(toX, toY);
  ctx.lineTo(
    toX - headLength * Math.cos(angle - Math.PI / 7),
    toY - headLength * Math.sin(angle - Math.PI / 7)
  );
  ctx.lineTo(
    toX - headLength * Math.cos(angle + Math.PI / 7),
    toY - headLength * Math.sin(angle + Math.PI / 7)
  );
  ctx.closePath();
  ctx.fill();
}

function drawForklift(x, y, headingRad, fillColor, strokeColor, alpha) {
  const headSize = 26;
  const forkLength = 34;
  const forkWidth = 6;
  const forkGap = 10;
  const tailStart = headSize / 2;

  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(headingRad);
  ctx.globalAlpha = alpha;

  ctx.fillStyle = fillColor;
  ctx.strokeStyle = strokeColor;
  ctx.lineWidth = 1.5;

  ctx.beginPath();
  ctx.rect(-headSize / 2, -headSize / 2, headSize, headSize);
  ctx.fill();
  ctx.stroke();

  ctx.fillRect(-forkGap / 2 - forkWidth, tailStart, forkWidth, forkLength);
  ctx.fillRect(forkGap / 2, tailStart, forkWidth, forkLength);

  ctx.restore();
}

function draw() {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

  const width = rect.width;
  const height = rect.height;
  const center = { x: width / 2, y: height / 2 };

  ctx.clearRect(0, 0, width, height);

  // grid
  for (let x = 0; x <= width; x += GRID_PX) {
    const index = Math.round((x - center.x) / GRID_PX);
    ctx.strokeStyle = Math.abs(index) % 5 === 0 ? "rgba(30, 43, 41, 0.2)" : "rgba(30, 43, 41, 0.08)";
    ctx.lineWidth = Math.abs(index) % 5 === 0 ? 1.4 : 1;
    ctx.beginPath();
    ctx.moveTo(x + 0.5, 0);
    ctx.lineTo(x + 0.5, height);
    ctx.stroke();
  }

  for (let y = 0; y <= height; y += GRID_PX) {
    const index = Math.round((center.y - y) / GRID_PX);
    ctx.strokeStyle = Math.abs(index) % 5 === 0 ? "rgba(30, 43, 41, 0.2)" : "rgba(30, 43, 41, 0.08)";
    ctx.lineWidth = Math.abs(index) % 5 === 0 ? 1.4 : 1;
    ctx.beginPath();
    ctx.moveTo(0, y + 0.5);
    ctx.lineTo(width, y + 0.5);
    ctx.stroke();
  }

  // axis
  ctx.strokeStyle = "rgba(30, 111, 106, 0.7)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(center.x, 0);
  ctx.lineTo(center.x, height);
  ctx.moveTo(0, center.y);
  ctx.lineTo(width, center.y);
  ctx.stroke();

  const pxPerMeter = pixelsPerMeter();
  const mode = getMode();

  // ===== ROBOT POSE (CẬP NHẬT CẢ 2 MODE) =====
  // Robot mode: robot ở giữa, nhưng heading lấy từ currentPose.theta nếu có
  let robotHeading = currentPose ? currentPose.theta : 0;

  // Robot canvas position:
  // - robot mode: luôn center
  // - map mode: theo tọa độ map
  let robotCanvas = { x: center.x, y: center.y };

  if (mode === MODE_MAP && currentPose) {
    if (followRobot) {
      // camera bám robot => robot ở giữa
      robotCanvas = { x: center.x, y: center.y };
    } else {
      // camera đứng yên => robot chạy theo pose
      robotCanvas = mapToCanvas(center, currentPose.x, currentPose.y, pxPerMeter);
    }
  }

  // Target preview in robot frame around robotCanvas
  const targetX = robotCanvas.x + targetRel.x * pxPerMeter;
  const targetY = robotCanvas.y - targetRel.y * pxPerMeter;

  // End heading: end theta (robot frame) + robotHeading (map heading)
  const endHeadingRel = computeEndThetaRad();
  const endHeadingWorld = robotHeading + endHeadingRel;

  ctx.setLineDash([8, 8]);
  drawArrow(robotCanvas.x, robotCanvas.y, targetX, targetY, "rgba(213, 106, 58, 0.7)");
  ctx.setLineDash([]);

  // robot + target
  drawForklift(robotCanvas.x, robotCanvas.y, robotHeading, "#1e6f6a", "#123c39", 1.0);
  drawForklift(targetX, targetY, endHeadingWorld, "#d56a3a", "#8c4022", 0.55);

  // dot
  ctx.fillStyle = "#1e6f6a";
  ctx.beginPath();
  ctx.arc(robotCanvas.x, robotCanvas.y, 4, 0, Math.PI * 2);
  ctx.fill();

  // labels
  ctx.fillStyle = "rgba(30, 43, 41, 0.7)";
  ctx.font = "12px 'Space Grotesk', 'Trebuchet MS', sans-serif";
  ctx.fillText("+Y", center.x + 8, 16);
  ctx.fillText("+X", width - 24, center.y - 8);
  ctx.fillText(mode === MODE_MAP ? "map view" : "robot view", center.x + 10, center.y + 18);
}

async function fetchState() {
  try {
    const response = await fetch("/api/state", { cache: "no-store" });
    const data = await response.json();

    if (data.ok && data.pose) {
      statusConn.textContent = "live";
      const p = data.pose;
      statusPose.textContent = `x=${round(p.x)} y=${round(p.y)} theta=${round(p.thetaDeg, 1)}deg`;
      currentPose = p;

      // init viewOrigin lần đầu theo pose để map mode không bị out-of-view
      if (!viewInitialized && currentPose) {
        viewOrigin.x = currentPose.x;
        viewOrigin.y = currentPose.y;
        viewInitialized = true;
      }

      // nếu followRobot bật, camera bám robot trong map mode
      if (getMode() === MODE_MAP && currentPose && followRobot) {
        viewOrigin.x = currentPose.x;
        viewOrigin.y = currentPose.y;
      }

      syncXYInputsForCurrentMode();
    } else {
      statusConn.textContent = "waiting";
      statusPose.textContent = "n/a";
      currentPose = null;
    }
  } catch (err) {
    statusConn.textContent = "offline";
    statusPose.textContent = "n/a";
    currentPose = null;
  }

  updateReadouts();
  draw();
}

function buildMovePayload() {
  const mode = getMode();
  const payload = {
    mode,
    reverse: reverseToggle.checked,
    rotateDirection: rotateSelect.value,
  };

  const thetaOverride = parseThetaOverrideDeg();
  if (thetaOverride !== null) payload.thetaOverrideDeg = thetaOverride;

  if (mode === MODE_MAP) {
    const mapX = parseFloat(xInput.value);
    const mapY = parseFloat(yInput.value);
    payload.mapX = Number.isFinite(mapX) ? mapX : 0;
    payload.mapY = Number.isFinite(mapY) ? mapY : 0;
  } else {
    payload.x = targetRel.x;
    payload.y = targetRel.y;
  }

  return payload;
}

async function sendMove() {
  const mode = getMode();
  if (mode === MODE_MAP && !currentPose) {
    resultEl.textContent = "No pose yet (cannot send map target)";
    return;
  }

  resultEl.textContent = "Sending...";
  const payload = buildMovePayload();

  try {
    const response = await fetch("/api/move", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const data = await response.json();
    if (data.ok) {
      resultEl.textContent = `Sent (${payload.mode})`;
    } else {
      resultEl.textContent = data.error || "Send failed";
    }
  } catch (err) {
    resultEl.textContent = "Send failed";
  }
}

function clearTarget() {
  setTargetRel(0, 0);
  resultEl.textContent = "Idle";
}

async function cancelOrder() {
  resultEl.textContent = "Canceling...";
  try {
    const response = await fetch("/api/cancel", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: "{}",
    });
    const data = await response.json();
    if (data.ok) resultEl.textContent = "Cancel order sent";
    else resultEl.textContent = data.error || "Cancel failed";
  } catch (err) {
    resultEl.textContent = "Cancel failed";
  }
}

function applyModeUI() {
  const mode = getMode();
  const xLabel = document.querySelector('label[for="xInput"]');
  const yLabel = document.querySelector('label[for="yInput"]');

  if (mode === MODE_MAP) {
    if (xLabel) xLabel.textContent = "Map X (absolute)";
    if (yLabel) yLabel.textContent = "Map Y (absolute)";

    // khi chuyển sang map mode, nếu chưa init origin thì lấy theo pose
    if (currentPose) {
      viewOrigin.x = viewOrigin.x ?? currentPose.x;
      viewOrigin.y = viewOrigin.y ?? currentPose.y;
      viewInitialized = true;
    }
  } else {
    if (xLabel) xLabel.textContent = "X (m, right +)";
    if (yLabel) yLabel.textContent = "Y (m, forward +)";
  }

  syncXYInputsForCurrentMode();
  updateReadouts();
  draw();
}

// Click grid: set targetRel (robot frame)
canvas.addEventListener("click", (event) => {
  const rect = canvas.getBoundingClientRect();
  const dx = event.clientX - rect.left - rect.width / 2;
  const dy = rect.height / 2 - (event.clientY - rect.top);
  const xVal = dx * metersPerPixel();
  const yVal = dy * metersPerPixel();

  setTargetRel(xVal, yVal);

  const autoThetaDeg = (computeAutoEndThetaRad() * 180) / Math.PI;
  thetaOverrideInput.value = round(autoThetaDeg, 1);

  updateReadouts();
  draw();
});

xInput.addEventListener("input", syncFromXY);
yInput.addEventListener("input", syncFromXY);
distInput.addEventListener("input", syncFromPolar);
angleInput.addEventListener("input", syncFromPolar);
scaleInput.addEventListener("input", updateScale);

thetaOverrideInput.addEventListener("input", () => {
  updateReadouts();
  draw();
});

reverseToggle.addEventListener("change", () => {
  updateReadouts();
  draw();
});

modeSelect?.addEventListener("change", () => {
  // khi đổi mode, nếu sang map mode thì set origin theo pose (để robot nằm trong view)
  if (getMode() === MODE_MAP && currentPose) {
    viewOrigin.x = currentPose.x;
    viewOrigin.y = currentPose.y;
    viewInitialized = true;
  }
  applyModeUI();
});

zoomIn.addEventListener("click", () => {
  scale = Math.max(0.1, round(scale * 0.8, 2));
  scaleInput.value = scale;
  draw();
});

zoomOut.addEventListener("click", () => {
  scale = round(scale * 1.25, 2);
  scaleInput.value = scale;
  draw();
});

sendBtn.addEventListener("click", sendMove);
clearBtn.addEventListener("click", clearTarget);
cancelBtn.addEventListener("click", cancelOrder);

window.addEventListener("resize", draw);

// Init
setTargetRel(0, 0);
applyModeUI();
fetchState();
setInterval(fetchState, 1000);
