const JOINTS = [
  { name: "小指", kind: "bend" },
  { name: "无名指", kind: "bend" },
  { name: "中指", kind: "bend" },
  { name: "食指", kind: "bend" },
  { name: "拇指弯曲", kind: "bend" },
  { name: "拇指旋转", kind: "rotation" },
];

const HANDS = [
  { id: "right", label: "右手", channel: "FTDI if01" },
  { id: "left", label: "左手", channel: "FTDI if02" },
];

const model = {
  right: Array(6).fill(0.5),
  left: Array(6).fill(0.5),
};

let service = { selection: "both" };
let armed = false;
let dragging = 0;
let toastTimer;
let settingsTimer;
const pending = new Map();

const handsRoot = document.querySelector("#hands");
const armButton = document.querySelector("#arm-button");
const liveFollow = document.querySelector("#live-follow");
const statusDot = document.querySelector("#status-dot");
const statusText = document.querySelector("#status-text");
const lastUpdate = document.querySelector("#last-update");
const poseList = document.querySelector("#pose-list");
const savePoseButton = document.querySelector("#save-pose");
const poseNameInput = document.querySelector("#pose-name");
const settingsState = document.querySelector("#settings-state");

function validPoseValues(values) {
  return Array.isArray(values) && values.length === 6 &&
    values.every(value => Number.isFinite(value) && value >= 0 && value <= 1);
}

function validPoseDelays(values) {
  return Array.isArray(values) && values.length === 6 &&
    values.every(value => Number.isInteger(value) && value >= 0 && value <= 3000);
}

function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, character => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;",
  })[character]);
}

let poses = [];

function poseBars(pose) {
  return [...pose.right, ...pose.left]
    .map(q => `<i style="height:${Math.max(2, Math.round((1 - q) * 100))}%"></i>`)
    .join("");
}

function delaySummary(delays) {
  const active = delays
    .map((delay, index) => delay ? `${JOINTS[index].name} +${delay} ms` : "")
    .filter(Boolean);
  return active.length ? active.join(" · ") : "全部同步";
}

function renderPoses() {
  if (!poses.length) {
    poseList.innerHTML = '<div class="pose-empty">还没有已保存姿势。调整关节后点击“保存当前姿势”。</div>';
    return;
  }
  poseList.innerHTML = poses.map(pose => `
    <article class="pose-card">
      <div class="pose-info">
        <strong>${escapeHtml(pose.name)}</strong>
        <time datetime="${new Date(pose.created).toISOString()}">${new Date(pose.created).toLocaleString("zh-CN", { hour12: false })}</time>
        <div class="pose-preview" aria-label="双手弯曲程度预览">${poseBars(pose)}</div>
      </div>
      <div class="pose-actions">
        <button class="pose-action replay" data-replay-pose="${pose.id}">执行</button>
        <button class="pose-action" data-rename-pose="${pose.id}">改名</button>
        <button class="pose-action delete" data-delete-pose="${pose.id}" aria-label="删除${escapeHtml(pose.name)}">×</button>
      </div>
      <details class="pose-delay-panel">
        <summary>
          <span>关节启动延时</span>
          <small>${escapeHtml(delaySummary(pose.delays_ms))}</small>
        </summary>
        <div class="delay-presets" role="group" aria-label="延时快捷设置">
          <button type="button" data-delay-preset="${pose.id}:sync">全部同步</button>
          <button type="button" data-delay-preset="${pose.id}:thumb-300">拇指 +300 ms</button>
        </div>
        <div class="pose-delay-grid">
          ${JOINTS.map((joint, index) => `
            <label>
              <span>${joint.name}</span>
              <span class="delay-input">
                <input type="number" min="0" max="3000" step="50"
                       value="${pose.delays_ms[index]}"
                       data-pose-delay="${pose.id}:${index}"
                       aria-label="${joint.name}启动延时">
                <small>ms</small>
              </span>
            </label>`).join("")}
        </div>
        <div class="delay-footer">
          <span>相对于动作开始时间</span>
          <button type="button" data-save-delays="${pose.id}">保存延时</button>
        </div>
      </details>
    </article>`).join("");
}

function poseValues(values) {
  return values.map(value => value.toFixed(3)).join(",");
}

async function postRequest(path, fields) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/x-www-form-urlencoded" },
    body: new URLSearchParams({ ...fields, token: service.token || "" }),
  });
  const result = await response.json().catch(() => ({ error: `HTTP ${response.status}` }));
  if (!response.ok || !result.ok) throw new Error(result.error || `HTTP ${response.status}`);
  return result;
}

async function loadServerPoses() {
  const response = await fetch("/api/poses", { cache: "no-store" });
  const result = await response.json().catch(() => ({ error: `HTTP ${response.status}` }));
  if (!response.ok || !result.ok) throw new Error(result.error || `HTTP ${response.status}`);
  poses = Array.isArray(result.poses) ? result.poses
    .filter(pose => pose && validPoseValues(pose.right) && validPoseValues(pose.left))
    .map(pose => ({
      ...pose,
      delays_ms: validPoseDelays(pose.delays_ms) ? pose.delays_ms : Array(6).fill(0),
    })) : [];

  renderPoses();
}

async function saveCurrentPose() {
  const customName = poseNameInput.value.trim();
  const name = customName || `姿势 ${String(poses.length + 1).padStart(2, "0")}`;
  try {
    const result = await postRequest("/api/poses/save", {
      name,
      right: poseValues(model.right),
      left: poseValues(model.left),
    });
    poses.unshift(result.pose);
    renderPoses();
    poseNameInput.value = "";
    showToast(`${result.pose.name}已保存到控制服务`);
  } catch (error) {
    showToast(`保存失败：${error.message}`, true);
  }
}

async function replayPose(id) {
  const pose = poses.find(item => item.id === id);
  if (!pose) return;
  model.right = [...pose.right];
  model.left = [...pose.left];
  renderAll();
  if (!armed) {
    showToast(`${pose.name}已载入预览；解锁硬件后再次点击即可复现`);
    return;
  }
  try {
    const result = await postRequest("/api/poses/execute", {
      id,
      confirm: "execute-pose",
    });
    showToast(`${pose.name}已执行${result.duration_ms ? ` · 编排 ${result.duration_ms} ms` : ""}`);
  } catch (error) {
    setStatus("error", "姿势复现失败");
    showToast(error.message, true);
  }
}

function delayInputs(id) {
  return [...poseList.querySelectorAll(`[data-pose-delay^="${id}:"]`)];
}

async function savePoseDelays(id, preset) {
  const pose = poses.find(item => item.id === id);
  if (!pose) return;
  const inputs = delayInputs(id);
  if (inputs.length !== 6) return;

  if (preset === "sync") inputs.forEach(input => { input.value = "0"; });
  if (preset === "thumb-300") inputs.forEach((input, index) => {
    input.value = index >= 4 ? "300" : "0";
  });

  const delays = inputs.map(input => Number(input.value));
  if (!validPoseDelays(delays)) {
    showToast("启动延时必须是 0–3000 ms 的整数", true);
    return;
  }
  try {
    const result = await postRequest("/api/poses/delays", {
      id,
      delays_ms: delays.join(","),
    });
    poses = poses.map(item => item.id === id ? result.pose : item);
    renderPoses();
    showToast(`${pose.name}的启动延时已保存`);
  } catch (error) {
    showToast(`延时保存失败：${error.message}`, true);
  }
}

async function deletePose(id) {
  const pose = poses.find(item => item.id === id);
  try {
    await postRequest("/api/poses/delete", { id });
    poses = poses.filter(item => item.id !== id);
    renderPoses();
    if (pose) showToast(`${pose.name}已删除`);
  } catch (error) {
    showToast(`删除失败：${error.message}`, true);
  }
}

async function renamePose(id) {
  const pose = poses.find(item => item.id === id);
  if (!pose) return;
  const requested = window.prompt("输入新的姿势名称（最多 32 个字符）", pose.name);
  if (requested === null) return;
  const name = requested.trim().slice(0, 32);
  if (!name) {
    showToast("姿势名称不能为空", true);
    return;
  }
  try {
    const result = await postRequest("/api/poses/rename", { id, name });
    poses = poses.map(item => item.id === id ? result.pose : item);
    renderPoses();
    showToast(`已改名为“${result.pose.name}”并自动保存`);
  } catch (error) {
    showToast(`改名失败：${error.message}`, true);
  }
}

function enabledHand(hand) {
  return service.selection === "both" || service.selection === hand;
}

function sliderValueFromQ(joint, q) {
  return joint.kind === "bend" ? Math.round((1 - q) * 100) : Math.round(q * 100);
}

function qFromSlider(joint, value) {
  const normalized = Number(value) / 100;
  return joint.kind === "bend" ? 1 - normalized : normalized;
}

function handSvg(hand) {
  return `
    <svg viewBox="0 0 260 290" role="img" aria-label="${hand.label}执行器姿态示意">
      <text x="130" y="24" text-anchor="middle" class="visual-caption">ACTUATOR POSE</text>
      <rect class="wrist" x="94" y="220" width="72" height="48" rx="13" />
      <rect class="palm" x="60" y="140" width="140" height="100" rx="38" />
      ${[0, 1, 2, 3, 4].map(index => `<path class="finger-shadow" data-shadow="${index}"/><path class="finger" data-finger="${index}"/><circle class="joint-dot" data-joint-dot="${index}" r="4"/>`).join("")}
    </svg>`;
}

function handCard(hand) {
  const controls = JOINTS.map((joint, index) => `
    <label class="joint-control" data-contact="${hand.id}:${index}">
      <span class="joint-name">${joint.name}</span>
      <input type="range" min="0" max="100" step="1" data-hand="${hand.id}" data-joint="${index}" aria-label="${hand.label}${joint.name}">
      <output class="joint-value" data-output="${index}">50%</output>
    </label>`).join("");

  return `
    <article class="hand-card" data-card="${hand.id}">
      <div class="hand-heading">
        <h2>${hand.label}</h2>
        <span><b data-contact-mode="${hand.id}">建立接触基线</b> · ${hand.channel} · ID 1</span>
      </div>
      <div class="hand-body">
        <div class="hand-visual">${handSvg(hand)}</div>
        <div class="controls">${controls}</div>
      </div>
      <div class="hand-actions">
        <button class="action-button" data-preset="open" data-hand="${hand.id}">全部张开</button>
        <button class="action-button" data-preset="neutral" data-hand="${hand.id}">中间位置</button>
        <button class="action-button" data-preset="closed" data-hand="${hand.id}">全部闭合</button>
        <button class="action-button primary" data-apply="${hand.id}">应用本手</button>
      </div>
      <div class="grip-panel">
        <div class="grip-heading">
          <strong>接触后抓握力</strong>
          <span>调整后自动永久保存</span>
        </div>
        <label class="grip-control">
          <span>力阈值</span>
          <input type="range" min="50" max="1000" step="25" value="300"
                 data-grip-force="${hand.id}" data-setting="${hand.id}_grip_force_grams"
                 data-unit=" g" aria-label="${hand.label}抓握力阈值">
          <output data-setting-output="${hand.id}_grip_force_grams">300 g</output>
        </label>
        <label class="grip-control">
          <span>电流上限</span>
          <input type="range" min="50" max="300" step="25" value="200"
                 data-grip-current="${hand.id}" data-setting="${hand.id}_grip_current_ma"
                 data-unit=" mA" aria-label="${hand.label}抓握电流上限">
          <output data-setting-output="${hand.id}_grip_current_ma">200 mA</output>
        </label>
        <button class="grip-button" type="button" data-grip="${hand.id}">施加抓握并保持目标</button>
        <p>先用较低力值测试；不要抓人体、易碎品或装有热液体的物体。</p>
      </div>
      <details class="register-panel" data-register-panel="${hand.id}">
        <summary>
          <span>寄存器实时监测</span>
          <span class="register-state" data-register-state>展开后开始读取</span>
        </summary>
        <div class="register-scroll">
          <table class="register-table">
            <thead>
              <tr>
                <th>关节</th>
                <th>位置<br><small>0x060A</small></th>
                <th>力<br><small>0x062E</small></th>
                <th>电流 mA<br><small>0x063A</small></th>
                <th>力阈值 g<br><small>0x05DA</small></th>
                <th>电流上限<br><small>0x03FC</small></th>
                <th>错误<br><small>0x0646</small></th>
                <th>状态<br><small>0x064C</small></th>
                <th>温度 °C<br><small>0x0652</small></th>
              </tr>
            </thead>
            <tbody data-register-body>
              <tr><td colspan="9" class="register-empty">展开面板后实时读取寄存器</td></tr>
            </tbody>
          </table>
        </div>
        <div class="fault-controls">
          <span data-fault-summary>展开后检查故障</span>
          <button type="button" data-clear-hand-fault="${hand.id}" disabled>清除本手故障</button>
        </div>
      </details>
    </article>`;
}

handsRoot.innerHTML = HANDS.map(handCard).join("");

function applySettings(value) {
  document.querySelectorAll("[data-setting]").forEach(input => {
    input.value = value[input.dataset.setting];
    document.querySelector(`[data-setting-output="${input.dataset.setting}"]`).textContent =
      input.value + (input.dataset.unit || "");
  });
}

async function loadSettings() {
  const response = await fetch("/api/settings", { cache: "no-store" });
  const result = await response.json().catch(() => ({ error: `HTTP ${response.status}` }));
  if (!response.ok || !result.ok || !result.settings) {
    settingsState.textContent = "读取失败";
    throw new Error(result.error || `HTTP ${response.status}`);
  }
  applySettings(result.settings);
  settingsState.textContent = "已永久保存";
}

function readSettings() {
  return Object.fromEntries([...document.querySelectorAll("[data-setting]")]
    .map(input => [input.dataset.setting, Number(input.value)]));
}

async function saveSettings() {
  settingsState.textContent = "正在保存…";
  try {
    const result = await postRequest("/api/settings", { settings: JSON.stringify(readSettings()) });
    applySettings(result.settings);
    settingsState.textContent = "已永久保存";
  } catch (error) {
    settingsState.textContent = "保存失败";
    showToast(`永久参数保存失败：${error.message}`, true);
  }
}

function scheduleSettingsSave() {
  clearTimeout(settingsTimer);
  settingsTimer = setTimeout(saveSettings, 250);
}

function renderContacts() {
  HANDS.forEach(({ id }) => {
    const state = service.states?.[id] || {};
    const monitoring = Boolean(state.contact_monitoring);
    document.querySelector(`[data-contact-mode="${id}"]`).textContent =
      monitoring ? "接触监测中" : "接触监测暂停";
    JOINTS.forEach((_, index) => {
      const indicator = document.querySelector(`[data-contact="${id}:${index}"]`);
      indicator.classList.toggle("contact", monitoring && Boolean(state.contact?.[index]));
    });
  });
}

function fingerPoints(hand, index, bend) {
  const bases = hand === "right" ? [78, 103, 130, 158] : [182, 157, 130, 102];
  const lengths = [45, 58, 64, 57];
  const x0 = bases[index];
  const y0 = 151;
  const curlDirection = hand === "right" ? 1 : -1;
  const theta1 = -Math.PI / 2 + curlDirection * bend * 0.55;
  const theta2 = theta1 + curlDirection * bend * 1.25;
  const first = lengths[index] * 0.58;
  const second = lengths[index] * 0.56;
  const x1 = x0 + Math.cos(theta1) * first;
  const y1 = y0 + Math.sin(theta1) * first;
  const x2 = x1 + Math.cos(theta2) * second;
  const y2 = y1 + Math.sin(theta2) * second;
  return { path: `M ${x0} ${y0} L ${x1.toFixed(1)} ${y1.toFixed(1)} L ${x2.toFixed(1)} ${y2.toFixed(1)}`, joint: [x1, y1] };
}

function thumbPoints(hand, bend, rotation) {
  const side = hand === "right" ? 1 : -1;
  const x0 = 130 + side * 69;
  const y0 = 180;
  const theta1 = (hand === "right" ? -0.22 : Math.PI + 0.22) + side * rotation * 0.42;
  const theta2 = theta1 + side * (0.28 + bend * 1.15);
  const x1 = x0 + Math.cos(theta1) * 34;
  const y1 = y0 + Math.sin(theta1) * 34;
  const x2 = x1 + Math.cos(theta2) * 31;
  const y2 = y1 + Math.sin(theta2) * 31;
  return { path: `M ${x0} ${y0} L ${x1.toFixed(1)} ${y1.toFixed(1)} L ${x2.toFixed(1)} ${y2.toFixed(1)}`, joint: [x1, y1] };
}

function renderHand(hand) {
  const card = document.querySelector(`[data-card="${hand}"]`);
  if (!card) return;

  model[hand].forEach((q, index) => {
    const slider = card.querySelector(`[data-joint="${index}"]`);
    const output = card.querySelector(`[data-output="${index}"]`);
    const shown = sliderValueFromQ(JOINTS[index], q);
    slider.value = shown;
    slider.style.setProperty("--fill", `${shown}%`);
    output.textContent = `${shown}%`;
  });

  for (let index = 0; index < 4; index += 1) {
    const geometry = fingerPoints(hand, index, 1 - model[hand][index]);
    card.querySelector(`[data-finger="${index}"]`).setAttribute("d", geometry.path);
    card.querySelector(`[data-shadow="${index}"]`).setAttribute("d", geometry.path);
    const dot = card.querySelector(`[data-joint-dot="${index}"]`);
    dot.setAttribute("cx", geometry.joint[0]);
    dot.setAttribute("cy", geometry.joint[1]);
  }

  const thumb = thumbPoints(hand, 1 - model[hand][4], model[hand][5] - 0.5);
  card.querySelector('[data-finger="4"]').setAttribute("d", thumb.path);
  card.querySelector('[data-shadow="4"]').setAttribute("d", thumb.path);
  const thumbDot = card.querySelector('[data-joint-dot="4"]');
  thumbDot.setAttribute("cx", thumb.joint[0]);
  thumbDot.setAttribute("cy", thumb.joint[1]);
}

function renderAll() {
  HANDS.forEach(({ id }) => renderHand(id));
}

function showToast(message, error = false) {
  const toast = document.querySelector("#toast");
  toast.textContent = message;
  toast.classList.toggle("error", error);
  toast.classList.add("visible");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => toast.classList.remove("visible"), 2600);
}

function setStatus(kind, text) {
  statusDot.className = `status-dot ${kind}`;
  statusText.textContent = text;
}

async function postCommand(fields) {
  const body = new URLSearchParams({ ...fields, confirm: "move", token: service.token });
  const response = await fetch("/api/command", {
    method: "POST",
    headers: { "Content-Type": "application/x-www-form-urlencoded" },
    body,
  });
  const result = await response.json().catch(() => ({ error: `HTTP ${response.status}` }));
  if (!response.ok || !result.ok) throw new Error(result.error || `HTTP ${response.status}`);
}

async function sendJoint(hand, joint) {
  if (!armed) return;
  try {
    await postCommand({ hand, joint: String(joint), q: model[hand][joint].toFixed(3) });
    document.querySelector(`[data-card="${hand}"] [data-finger="${Math.min(joint, 4)}"]`)?.classList.add("active");
  } catch (error) {
    setStatus("error", "命令发送失败");
    showToast(error.message, true);
  }
}

function scheduleJoint(hand, joint) {
  const key = `${hand}:${joint}`;
  clearTimeout(pending.get(key));
  pending.set(key, setTimeout(() => {
    pending.delete(key);
    sendJoint(hand, joint);
  }, 110));
}

async function applyHand(hand) {
  if (!armed) {
    showToast("请先解锁硬件控制", true);
    return;
  }
  try {
    await postCommand({ hand, values: model[hand].map(value => value.toFixed(3)).join(",") });
    showToast(`${hand === "right" ? "右手" : "左手"}目标已发送`);
  } catch (error) {
    setStatus("error", "命令发送失败");
    showToast(error.message, true);
  }
}

async function applyGrip(hand) {
  if (!armed) {
    showToast("请先解锁硬件控制", true);
    return;
  }
  const card = document.querySelector(`[data-card="${hand}"]`);
  const force = card.querySelector("[data-grip-force]").value;
  const current = card.querySelector("[data-grip-current]").value;
  const accepted = window.confirm(
    `将以 ${force} g 力阈值、${current} mA 电流上限施加抓握。确认物体和手指位置安全？`
  );
  if (!accepted) return;
  try {
    const response = await fetch("/api/grip", {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: new URLSearchParams({
        hand,
        force,
        current,
        values: poseValues(model[hand]),
        token: service.token,
        confirm: "grip",
      }),
    });
    const result = await response.json().catch(() => ({ error: `HTTP ${response.status}` }));
    if (!response.ok || !result.ok) throw new Error(result.error || `HTTP ${response.status}`);
    showToast(`${hand === "right" ? "右手" : "左手"}已施加 ${result.force} g 抓握力阈值`);
    refreshRegisters(hand);
  } catch (error) {
    setStatus("error", "抓握命令失败");
    showToast(error.message, true);
  }
}

async function clearJointFault(hand, joint) {
  if (!armed) {
    showToast("请先解锁硬件控制；清故障会重新使能电缸", true);
    return;
  }
  const jointName = JOINTS[joint]?.name || `通道 ${joint}`;
  const accepted = window.confirm(
    `即将清除${hand === "right" ? "右手" : "左手"}${jointName}故障。` +
    "服务会先把故障通道目标锁定在当前反馈位置；清错寄存器会作用于本手全部可清除故障。" +
    "确认物体已取下、周围无人无障碍且物理急停触手可及？"
  );
  if (!accepted) return;
  try {
    const response = await fetch("/api/faults/clear", {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: new URLSearchParams({
        hand,
        joint: String(joint),
        token: service.token,
        confirm: "clear-fault",
      }),
    });
    const result = await response.json().catch(() => ({ error: `HTTP ${response.status}` }));
    if (!response.ok || !result.ok) throw new Error(result.error || result.message || `HTTP ${response.status}`);
    showToast(result.cleared ? `${jointName}故障已清除` : `${jointName}当前没有故障`);
    await refreshRegisters(hand);
  } catch (error) {
    setStatus("error", "清除故障失败");
    showToast(error.message, true);
  }
}

const registerBusy = new Set();

function hexByte(value) {
  return `0x${Number(value).toString(16).padStart(2, "0").toUpperCase()}`;
}

async function refreshRegisters(hand) {
  const panel = document.querySelector(`[data-register-panel="${hand}"]`);
  if (!panel?.open || registerBusy.has(hand)) return;
  const body = panel.querySelector("[data-register-body]");
  const state = panel.querySelector("[data-register-state]");
  registerBusy.add(hand);
  state.textContent = "读取中…";
  try {
    const response = await fetch(`/api/registers/${hand}`, { cache: "no-store" });
    const data = await response.json().catch(() => ({ error: `HTTP ${response.status}` }));
    if (!response.ok || !data.ok) throw new Error(data.error || `HTTP ${response.status}`);
    const fields = [data.position, data.force, data.current, data.force_limit,
      data.current_limit, data.error, data.status, data.temperature];
    if (!fields.every(values => Array.isArray(values) && values.length === 6))
      throw new Error("寄存器响应格式错误");
    const faulted = data.error
      .map((value, index) => Number(value) !== 0 ? index : -1)
      .filter(index => index >= 0);
    const faultSummary = panel.querySelector("[data-fault-summary]");
    const clearButton = panel.querySelector("[data-clear-hand-fault]");
    if (faulted.length) {
      faultSummary.textContent = `检测到 ${faulted.length} 个故障通道`;
      clearButton.dataset.joint = String(faulted[0]);
      clearButton.disabled = !armed;
    } else {
      faultSummary.textContent = "当前无故障";
      delete clearButton.dataset.joint;
      clearButton.disabled = true;
    }
    body.innerHTML = JOINTS.map((joint, index) => {
      const q = Math.max(0, Math.min(1, data.position[index] / 1000));
      const error = Number(data.error[index]);
      return `<tr>
        <th>${joint.name}</th>
        <td>${data.position[index]} <small>q=${q.toFixed(3)}</small></td>
        <td>${data.force[index]}</td>
        <td class="${Math.abs(data.current[index]) >= 1000 ? "register-warn" : ""}">${data.current[index]}</td>
        <td>${data.force_limit[index]}</td>
        <td>${data.current_limit[index]}</td>
        <td class="${error ? "register-alarm" : ""}">
          ${hexByte(error)}
          ${error ? `<button class="fault-clear-button" data-clear-fault="${hand}:${index}">清除</button>` : ""}
        </td>
        <td>${hexByte(data.status[index])}</td>
        <td class="${data.temperature[index] >= 60 ? "register-alarm" : ""}">${data.temperature[index]}</td>
      </tr>`;
    }).join("");
    state.textContent = `实时 · ${new Date().toLocaleTimeString("zh-CN", { hour12: false })}`;
  } catch (error) {
    state.textContent = "读取异常";
    panel.querySelector("[data-fault-summary]").textContent = "故障状态读取失败";
    panel.querySelector("[data-clear-hand-fault]").disabled = true;
    body.innerHTML = `<tr><td colspan="9" class="register-empty register-alarm">${escapeHtml(error.message)}</td></tr>`;
  } finally {
    registerBusy.delete(hand);
  }
}

function refreshOpenRegisters() {
  document.querySelectorAll("[data-register-panel][open]").forEach(panel => {
    refreshRegisters(panel.dataset.registerPanel);
  });
}

function applyServiceState() {
  HANDS.forEach(({ id }) => {
    const card = document.querySelector(`[data-card="${id}"]`);
    const enabled = enabledHand(id);
    card.hidden = !enabled;
    card.querySelectorAll("input, button").forEach(control => {
      if (control.matches("[data-clear-hand-fault]"))
        control.disabled = !enabled || !armed || !control.dataset.joint;
      else
        control.disabled = !enabled ||
          (control.matches("[data-apply], [data-grip]") && !armed);
    });
  });
  armButton.disabled = Boolean(service.errors?.length);
  armButton.classList.toggle("armed", armed);
  armButton.textContent = armed ? "锁定硬件控制" : "解锁硬件控制";
  renderContacts();
}

async function refreshStatus({ quiet = false } = {}) {
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const data = await response.json();
    service = data;
    if (!dragging && !armed) {
      Object.entries(data.states || {}).forEach(([hand, state]) => {
        const values = state.feedback_q;
        if (model[hand] && Array.isArray(values) && values.length === 6)
          model[hand] = values.map(value => Math.max(0, Math.min(1, Number(value))));
      });
      renderAll();
    }
    if (data.errors?.length) {
      armed = false;
      setStatus("error", "硬件读取异常");
      if (!quiet) showToast(data.errors.join("；"), true);
    } else if (armed) {
      setStatus("armed", "硬件控制已解锁");
    } else {
      setStatus("ready", "硬件在线 · 控制已锁定");
    }
    if (!data.errors?.length)
      lastUpdate.textContent = `最近读取 ${new Date().toLocaleTimeString("zh-CN", { hour12: false })}`;
    applyServiceState();
  } catch (error) {
    armed = false;
    setStatus("error", "无法连接控制服务");
    armButton.disabled = true;
    if (!quiet) showToast(error.message, true);
  }
}

document.querySelectorAll('input[type="range"][data-hand][data-joint]').forEach(slider => {
  const hand = slider.dataset.hand;
  const joint = Number(slider.dataset.joint);
  slider.addEventListener("pointerdown", () => { dragging += 1; });
  slider.addEventListener("pointerup", () => { dragging = Math.max(0, dragging - 1); });
  slider.addEventListener("pointercancel", () => { dragging = Math.max(0, dragging - 1); });
  slider.addEventListener("input", () => {
    model[hand][joint] = qFromSlider(JOINTS[joint], slider.value);
    renderHand(hand);
    if (armed && liveFollow.checked) scheduleJoint(hand, joint);
  });
  slider.addEventListener("change", () => {
    if (armed && !liveFollow.checked) sendJoint(hand, joint);
  });
});

document.querySelectorAll("[data-preset]").forEach(button => {
  button.addEventListener("click", () => {
    const hand = button.dataset.hand;
    const q = { open: 1, neutral: 0.5, closed: 0 }[button.dataset.preset];
    model[hand] = Array(6).fill(q);
    renderHand(hand);
    if (armed) applyHand(hand);
  });
});

document.querySelectorAll("[data-apply]").forEach(button => {
  button.addEventListener("click", () => applyHand(button.dataset.apply));
});

document.querySelectorAll("[data-setting]").forEach(input => {
  input.addEventListener("input", () => {
    document.querySelector(`[data-setting-output="${input.dataset.setting}"]`).textContent =
      input.value + (input.dataset.unit || "");
  });
  input.addEventListener("change", scheduleSettingsSave);
});

document.querySelectorAll("[data-grip]").forEach(button => {
  button.addEventListener("click", () => applyGrip(button.dataset.grip));
});

handsRoot.addEventListener("click", event => {
  const rowButton = event.target.closest("[data-clear-fault]");
  const handButton = event.target.closest("[data-clear-hand-fault]");
  if (rowButton) {
    const [hand, joint] = rowButton.dataset.clearFault.split(":");
    clearJointFault(hand, Number(joint));
  } else if (handButton?.dataset.joint) {
    clearJointFault(handButton.dataset.clearHandFault,
      Number(handButton.dataset.joint));
  }
});

savePoseButton.addEventListener("click", saveCurrentPose);
poseNameInput.addEventListener("keydown", event => {
  if (event.key === "Enter") saveCurrentPose();
});

poseList.addEventListener("click", event => {
  const replay = event.target.closest("[data-replay-pose]");
  const rename = event.target.closest("[data-rename-pose]");
  const remove = event.target.closest("[data-delete-pose]");
  const saveDelays = event.target.closest("[data-save-delays]");
  const preset = event.target.closest("[data-delay-preset]");
  if (replay) replayPose(replay.dataset.replayPose);
  if (rename) renamePose(rename.dataset.renamePose);
  if (remove) deletePose(remove.dataset.deletePose);
  if (saveDelays) savePoseDelays(saveDelays.dataset.saveDelays);
  if (preset) {
    const [id, name] = preset.dataset.delayPreset.split(":");
    savePoseDelays(id, name);
  }
});

document.querySelectorAll("[data-register-panel]").forEach(panel => {
  panel.addEventListener("toggle", () => {
    const state = panel.querySelector("[data-register-state]");
    if (panel.open) refreshRegisters(panel.dataset.registerPanel);
    else state.textContent = "展开后开始读取";
  });
});

armButton.addEventListener("click", () => {
  if (armed) {
    armed = false;
    setStatus("ready", "硬件在线 · 控制已锁定");
    applyServiceState();
    return;
  }
  const accepted = window.confirm("确认双手周围无人员和障碍物，并且物理急停触手可及？");
  if (!accepted) return;
  armed = true;
  setStatus("armed", "硬件控制已解锁");
  applyServiceState();
});

window.addEventListener("blur", () => {
  if (armed && liveFollow.checked) {
    liveFollow.checked = false;
    showToast("窗口失去焦点，已关闭连续跟随");
  }
});

async function initialize() {
  renderAll();
  renderPoses();
  savePoseButton.disabled = true;
  await refreshStatus();
  try {
    await Promise.all([loadServerPoses(), loadSettings()]);
  } catch (error) {
    showToast(`永久数据加载失败：${error.message}`, true);
  }
  savePoseButton.disabled = !service.token;
}

initialize();
setInterval(() => refreshStatus({ quiet: true }), 1800);
setInterval(refreshOpenRegisters, 1000);
