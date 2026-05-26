<script setup lang="ts">
import { ref, computed, watch, onMounted, onUnmounted } from 'vue'

/**
 * PI 电流调节器仿真 (dq 电流环)
 * 展示 dq 电流环阶跃响应、带宽与解耦
 *
 * Canvas 920×500，左右两栏:
 *   左栏 (0~455): 框图 — dq 电流控制回路
 *   右栏 (465~910): 时域阶跃响应 — Id/Iq 跟踪曲线
 */

const canvasRef = ref<HTMLCanvasElement | null>(null)
let animFrameId = 0

// --- PI 参数 ---
const kp = ref(5)           // 比例增益 (0.1 ~ 50)
const ki = ref(50)          // 积分增益 (1 ~ 500)
const tuningBw = ref(500)   // 带宽参数化 Hz (50 ~ 2000)

// --- 电机参数 ---
const motorR = ref(0.5)     // 相电阻 Ω
const motorL = ref(1.0)     // 相电感 mH

// --- 阶跃参数 ---
const stepAmp = ref(1.0)    // 阶跃幅值 A
const runSim = ref(false)   // 触发仿真

// --- 仿真状态 ---
interface SimState {
  id: number; iq: number; idErr: number; iqErr: number
  idIntegral: number; iqIntegral: number; t: number
}
const history = ref<SimState[]>([])
const SIM_DURATION = 0.01   // 10ms 仿真时长
const SIM_STEPS = 200
const DT = SIM_DURATION / SIM_STEPS

// --- 带宽参数化: Kp = ωbw·L, Ki = ωbw·R ---
watch(tuningBw, (bw) => {
  const omegaBw = 2 * Math.PI * bw
  kp.value = Math.round(omegaBw * motorL.value * 1e-3 * 10) / 10
  ki.value = Math.round(omegaBw * motorR.value * 10) / 10
})

// 当用户手动改 Kp/Ki 时同步调整带宽显示
const effectiveBw = computed(() => {
  if (motorL.value <= 0) return 0
  return (kp.value / (motorL.value * 1e-3)) / (2 * Math.PI)
})

// --- 运行仿真 (欧拉离散) ---
function runSimulation() {
  const localHistory: SimState[] = []
  const L = motorL.value * 1e-3
  const R = motorR.value
  let id = 0, iq = 0
  let idInt = 0, iqInt = 0

  // Id 阶跃目标 (0→stepAmp), Iq 阶跃目标 (0→stepAmp*2 或用户设定)
  const idRef = stepAmp.value
  const iqRef = stepAmp.value * 1.5

  for (let k = 0; k <= SIM_STEPS; k++) {
    const t = k * DT

    // PI 控制律
    const idErr = idRef - id
    const iqErr = iqRef - iq
    idInt += idErr * DT
    iqInt += iqErr * DT

    // 积分限幅 anti-windup
    const maxIntegral = 10
    idInt = Math.max(-maxIntegral, Math.min(maxIntegral, idInt))
    iqInt = Math.max(-maxIntegral, Math.min(maxIntegral, iqInt))

    let vd = kp.value * idErr + ki.value * idInt
    let vq = kp.value * iqErr + ki.value * iqInt

    // 简单电压限幅
    const vMax = 100
    vd = Math.max(-vMax, Math.min(vMax, vd))
    vq = Math.max(-vMax, Math.min(vMax, vq))

    // 电机模型 (忽略耦合项简化)
    const did = (DT / L) * (vd - R * id)
    const diq = (DT / L) * (vq - R * iq)
    id += did
    iq += diq

    localHistory.push({ id, iq, idErr, iqErr, idIntegral: idInt, iqIntegral: iqInt, t })
  }
  history.value = localHistory
}

// --- 动态指标 ---
const metrics = computed(() => {
  const h = history.value
  if (h.length < 2) return { riseTime: 0, settlingTime: 0, overshoot: 0, phaseMargin: 0, bw: 0 }
  const last = h[h.length - 1]
  const idRef = stepAmp.value
  const iqRef = stepAmp.value * 1.5

  // 上升时间 (10%→90%)
  let t10 = 0, t90 = 0
  for (const s of h) {
    if (!t10 && s.id >= 0.1 * idRef) t10 = s.t
    if (!t90 && s.id >= 0.9 * idRef) t90 = s.t
  }

  let tSettle = 0
  const band2pct = 0.02 * idRef
  for (let i = h.length - 1; i >= 0; i--) {
    if (Math.abs(h[i].id - idRef) > band2pct) { tSettle = i < h.length - 1 ? h[i + 1].t : h[i].t; break }
  }

  let peak = 0
  for (const s of h) if (s.id > peak) peak = s.id
  const overshoot = peak > idRef ? ((peak - idRef) / idRef) * 100 : 0

  // 相位裕度近似 (带宽参数化)
  const omegaBW = effectiveBw.value * 2 * Math.PI
  const tauE = motorL.value * 1e-3 / motorR.value
  const Ti = kp.value / Math.max(ki.value, 0.001)
  const phiPlant = Math.atan(omegaBW * tauE)
  const phiPI = Math.atan(ki.value / (omegaBW * kp.value))
  const phaseMargin = 180 - (180 / Math.PI) * (phiPlant + phiPI)

  return {
    riseTime: (t90 - t10) * 1000,
    settlingTime: tSettle * 1000,
    overshoot,
    phaseMargin: Math.max(0, phaseMargin),
    bw: effectiveBw.value,
  }
})

// ═════════════════════════ 绘图 ═════════════════════════
const draw = () => {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  const W = 920, H = 500
  canvas.width = W; canvas.height = H

  const style = getComputedStyle(document.documentElement)
  const bgColor   = style.getPropertyValue('--bg-color').trim() || '#f8fafc'
  const textColor = style.getPropertyValue('--text-color').trim() || '#1e293b'
  const borderColor = style.getPropertyValue('--border-color').trim() || '#e2e8f0'
  const primaryColor = style.getPropertyValue('--primary-color').trim() || '#3b82f6'
  const textSec = style.getPropertyValue('--text-secondary').trim() || '#64748b'
  const contentBg = style.getPropertyValue('--content-bg').trim() || '#ffffff'

  ctx.clearRect(0, 0, W, H)
  ctx.fillStyle = bgColor; ctx.fillRect(0, 0, W, H)

  // 左右分隔线
  const SEP = 458
  ctx.strokeStyle = borderColor; ctx.lineWidth = 1; ctx.setLineDash([6, 4])
  ctx.beginPath(); ctx.moveTo(SEP, 10); ctx.lineTo(SEP, H - 10); ctx.stroke()
  ctx.setLineDash([])

  // 标题
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('dq 电流控制回路框图', 228, 24)

  // ═════════════════════ 左栏: 框图 ═════════════════════
  // 绘制传递函数方块
  const boxW = 88, boxH = 44
  const arrowLen = 32

  function drawBox(x: number, y: number, w: number, h: number, label: string, sub: string, color: string) {
    ctx.fillStyle = color + '18'
    ctx.fillRect(x, y, w, h)
    ctx.strokeStyle = color; ctx.lineWidth = 1.2
    ctx.strokeRect(x, y, w, h)
    ctx.fillStyle = textColor; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
    ctx.fillText(label, x + w / 2, y + h * 0.35)
    if (sub) {
      ctx.font = '8px monospace'; ctx.fillStyle = textSec
      ctx.fillText(sub, x + w / 2, y + h * 0.72)
    }
    ctx.textBaseline = 'alphabetic'
  }

  function drawArrow(x1: number, y1: number, x2: number, y2: number, c: string, lw: number) {
    ctx.strokeStyle = c; ctx.lineWidth = lw
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke()
    // 箭头
    const angle = Math.atan2(y2 - y1, x2 - x1)
    const al = 7
    ctx.fillStyle = c
    ctx.beginPath()
    ctx.moveTo(x2, y2)
    ctx.lineTo(x2 - al * Math.cos(angle - 0.45), y2 - al * Math.sin(angle - 0.45))
    ctx.lineTo(x2 - al * Math.cos(angle + 0.45), y2 - al * Math.sin(angle + 0.45))
    ctx.closePath(); ctx.fill()
  }

  const flowY = 130
  // + 求和节点
  ctx.fillStyle = textColor; ctx.font = 'bold 14px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('+', 32, flowY + boxH / 2)
  ctx.strokeStyle = textColor; ctx.lineWidth = 1.4
  ctx.beginPath(); ctx.arc(28, flowY + boxH / 2, 9, 0, 2 * Math.PI); ctx.stroke()

  // PI 控制器
  drawBox(50, flowY, boxW, boxH, 'PI 控制器', `Kp=${kp.value} Ki=${ki.value}`, primaryColor)

  // dq 解耦
  drawBox(170, flowY, boxW + 10, boxH, 'dq 解耦', 'ωL·iq/ωL·id', '#8b5cf6')

  // 电机模型
  drawBox(305, flowY, boxW + 10, boxH, 'R + L 模型', `${motorR.value}Ω ${motorL.value}mH`, '#10b981')

  // 反馈线
  drawArrow(28, flowY - 56, 28, flowY - 4, '#ef4444', 1.2)
  ctx.fillStyle = '#ef4444'; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText('Iref', 36, flowY - 56)
  ctx.fillText('dq', 36, flowY - 42)

  // 正向箭头
  drawArrow(28 + 12, flowY + boxH / 2, 50, flowY + boxH / 2, primaryColor, 1.5)
  drawArrow(50 + boxW, flowY + boxH / 2, 170, flowY + boxH / 2, '#8b5cf6', 1.5)
  drawArrow(170 + boxW + 10, flowY + boxH / 2, 305, flowY + boxH / 2, '#10b981', 1.5)

  // 输出
  drawArrow(305 + boxW + 10, flowY + boxH / 2, 438, flowY + boxH / 2, '#10b981', 1.5)
  ctx.fillStyle = '#10b981'; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'right'
  ctx.fillText('Iout,dq', 435, flowY + boxH / 2 - 8)

  // 反馈路径
  ctx.strokeStyle = '#ef4444'; ctx.setLineDash([4, 3]); ctx.lineWidth = 1
  ctx.beginPath()
  ctx.moveTo(28, flowY + boxH + 14)
  ctx.lineTo(28, flowY + boxH + 52)
  ctx.lineTo(385, flowY + boxH + 52)
  ctx.lineTo(385, flowY + boxH + 10)
  ctx.stroke()
  ctx.setLineDash([])
  // 反馈箭头
  const fa = Math.PI / 2
  ctx.fillStyle = '#ef4444'
  ctx.beginPath()
  ctx.moveTo(385, flowY + boxH + 10)
  ctx.lineTo(385 - 4, flowY + boxH + 20)
  ctx.lineTo(385 + 4, flowY + boxH + 20)
  ctx.closePath(); ctx.fill()

  ctx.fillStyle = '#ef4444'; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('反馈 (Iout,dq)', 220, flowY + boxH + 48)

  // 下方公式区
  const formY = flowY + boxH + 70
  ctx.fillStyle = contentBg
  ctx.fillRect(18, formY, 430, 130)
  ctx.strokeStyle = borderColor; ctx.lineWidth = 0.8
  ctx.strokeRect(18, formY, 430, 130)

  ctx.fillStyle = textColor; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('带宽参数化 PI 整定', 235, formY + 18)

  ctx.font = '10px monospace'; ctx.textAlign = 'left'; ctx.fillStyle = textColor
  const formulas = [
    'Kp = ωbw · L          (比例: 抵消电感惯性)',
    'Ki = ωbw · R          (积分: 抵消电阻压降)',
    `ωbw = ${effectiveBw.value.toFixed(0)} Hz   (电流环带宽)`,
    `Kp = ${kp.value} (V/A)   Ki = ${ki.value} (V/A/s)`,
    '闭环传递函数: I(s)/I*(s) = ωbw/(s+ωbw)',
    '解耦项: vd,ff = -ωe·Lq·iq',
    '         vq,ff = +ωe·(Ld·id + ψf)',
  ]
  formulas.forEach((f, i) => {
    ctx.fillText(f, 28, formY + 34 + i * 14)
  })

  // ═════════════════════ 右栏: 时域响应 ═════════════════════
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('时域阶跃响应', 690, 24)

  const rL = 472, rT = 38, rW = 435, rH = 240
  const rB = rT + rH

  // 绘图区
  ctx.strokeStyle = borderColor; ctx.lineWidth = 0.6
  ctx.strokeRect(rL, rT, rW, rH)

  if (history.value.length > 1) {
    const idRef = stepAmp.value
    const iqRef = stepAmp.value * 1.5
    const maxVal = Math.max(idRef, iqRef) * 1.3

    const toX = (t: number) => rL + 8 + (t / SIM_DURATION) * (rW - 16)
    const toY = (v: number) => rB - 8 - (v / maxVal) * (rH - 30)

    // Id 参考线
    ctx.strokeStyle = '#ec4899'; ctx.lineWidth = 0.8; ctx.setLineDash([4, 4])
    ctx.beginPath()
    ctx.moveTo(rL + 8, toY(idRef)); ctx.lineTo(rL + rW - 8, toY(idRef))
    ctx.stroke()
    ctx.fillStyle = '#ec4899'; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'left'
    ctx.fillText(`Id*=${idRef.toFixed(1)}A`, rL + 12, toY(idRef) - 4)

    // Iq 参考线
    ctx.strokeStyle = '#06b6d4'; ctx.lineWidth = 0.8; ctx.setLineDash([4, 4])
    ctx.beginPath()
    ctx.moveTo(rL + 8, toY(iqRef)); ctx.lineTo(rL + rW - 8, toY(iqRef))
    ctx.stroke()
    ctx.fillText(`Iq*=${iqRef.toFixed(1)}A`, rL + 12, toY(iqRef) - 4)
    ctx.setLineDash([])

    const h = history.value

    // Id 实际响应
    ctx.strokeStyle = '#ec4899'; ctx.lineWidth = 2
    ctx.beginPath()
    for (let i = 0; i < h.length; i++) {
      const x = toX(h[i].t), y = toY(h[i].id)
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
    }
    ctx.stroke()

    // Iq 实际响应
    ctx.strokeStyle = '#06b6d4'; ctx.lineWidth = 2
    ctx.beginPath()
    for (let i = 0; i < h.length; i++) {
      const x = toX(h[i].t), y = toY(h[i].iq)
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
    }
    ctx.stroke()

    // 图例
    ctx.fillStyle = '#ec4899'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'left'
    ctx.fillText('Id (实际)', rL + 12, rT + 14)
    ctx.fillStyle = '#06b6d4'
    ctx.fillText('Iq (实际)', rL + 12, rT + 28)

    // 标注超调
    if (metrics.value.overshoot > 0.5) {
      let peakIdx = 0, peakVal = 0
      h.forEach((s, i) => { if (s.id > peakVal) { peakVal = s.id; peakIdx = i } })
      const px = toX(h[peakIdx].t)
      const py = toY(peakVal)
      ctx.fillStyle = '#ef4444'; ctx.beginPath(); ctx.arc(px, py, 4, 0, 2 * Math.PI); ctx.fill()
      ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'left'
      ctx.fillText(`${metrics.value.overshoot.toFixed(1)}% 超调`, px + 8, py - 4)
    }
  }

  // 坐标轴标注
  ctx.fillStyle = textSec; ctx.font = '9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('0', rL + 4, rB - 6)
  ctx.fillText(`${(SIM_DURATION * 1000).toFixed(0)}ms`, rL + rW - 10, rB - 6)
  ctx.fillText('时间', rL + rW / 2, rB + 14)

  // ═════════════════════ 底部指标 ═════════════════════
  const infoY = rB + 28
  const m = metrics.value
  const infoItems = [
    { label: '上升时间 tr', value: `${m.riseTime.toFixed(2)} ms`, color: primaryColor },
    { label: '调节时间 ts', value: `${m.settlingTime.toFixed(2)} ms`, color: '#f59e0b' },
    { label: '超调量', value: `${m.overshoot.toFixed(1)}%`, color: '#ef4444' },
    { label: '相角裕度', value: `${m.phaseMargin.toFixed(1)}°`, color: '#10b981' },
    { label: '电流环带宽', value: `${m.bw.toFixed(0)} Hz`, color: '#8b5cf6' },
  ]

  const infoBoxW = 170, infoBoxH = 54, infoGap = 8
  const infoStartX = rL + (rW - (infoBoxW + infoGap) * 3 + infoGap) / 2

  infoItems.forEach((item, i) => {
    const col = i % 3, row = Math.floor(i / 3)
    const ix = infoStartX + col * (infoBoxW + infoGap)
    const iy = infoY + row * (infoBoxH + 4)
    ctx.fillStyle = contentBg
    ctx.fillRect(ix, iy, infoBoxW, infoBoxH)
    ctx.strokeStyle = borderColor; ctx.lineWidth = 0.6
    ctx.strokeRect(ix, iy, infoBoxW, infoBoxH)
    ctx.fillStyle = textSec; ctx.font = '9px sans-serif'; ctx.textAlign = 'center'
    ctx.fillText(item.label, ix + infoBoxW / 2, iy + 14)
    ctx.fillStyle = item.color; ctx.font = 'bold 14px monospace'
    ctx.fillText(item.value, ix + infoBoxW / 2, iy + 38)
  })
}

// ═════════════════════════ 动画循环 ═════════════════════════
let lastTs2 = 0
const animate = (ts: number) => {
  lastTs2 = ts
  try { draw() } catch { /* keep loop alive */ }
  animFrameId = requestAnimationFrame(animate)
}

onMounted(() => {
  runSimulation()
  animFrameId = requestAnimationFrame(animate)
})
onUnmounted(() => cancelAnimationFrame(animFrameId))
</script>

<template>
  <div class="sim-container">
    <div class="sim-header">
      <h4>PI 电流调节器仿真 -- dq 电流环阶跃响应与带宽参数化</h4>
      <div class="header-controls">
        <button class="auto-btn active" @click="runSimulation">
          &#9654; 运行仿真
        </button>
      </div>
    </div>

    <canvas ref="canvasRef" class="sim-canvas"></canvas>

    <div class="sim-params">
      <div class="param-group">
        <label>带宽参数化 &#969;bw: <span class="param-val">{{ tuningBw }} Hz</span></label>
        <input type="range" v-model.number="tuningBw" :min="50" :max="2000" :step="10" @input="runSimulation" />
        <span class="param-range">50 ~ 2000 Hz</span>
      </div>
      <div class="param-group">
        <label>Kp: <span class="param-val">{{ kp }}</span></label>
        <input type="range" v-model.number="kp" :min="0.1" :max="50" :step="0.5" @input="runSimulation" />
        <span class="param-range">0.1 ~ 50</span>
      </div>
      <div class="param-group">
        <label>Ki: <span class="param-val">{{ ki }}</span></label>
        <input type="range" v-model.number="ki" :min="1" :max="500" :step="1" @input="runSimulation" />
        <span class="param-range">1 ~ 500</span>
      </div>
      <div class="param-group">
        <label>电阻 R: <span class="param-val">{{ motorR }} &#937;</span></label>
        <input type="range" v-model.number="motorR" :min="0.05" :max="5" :step="0.05" @input="runSimulation" />
        <span class="param-range">0.05 ~ 5 &#937;</span>
      </div>
      <div class="param-group">
        <label>电感 L: <span class="param-val">{{ motorL }} mH</span></label>
        <input type="range" v-model.number="motorL" :min="0.1" :max="10" :step="0.1" @input="runSimulation" />
        <span class="param-range">0.1 ~ 10 mH</span>
      </div>
      <div class="param-group">
        <label>阶跃幅值 Id: <span class="param-val">{{ stepAmp }} A</span></label>
        <input type="range" v-model.number="stepAmp" :min="0.2" :max="5" :step="0.1" @input="runSimulation" />
        <span class="param-range">0.2 ~ 5 A</span>
      </div>
    </div>

    <div class="sim-info">
      <div class="info-block">
        <div class="info-block-title">PI 参数</div>
        <div class="info-row"><span class="info-label">Kp:</span><span class="info-value highlight">{{ kp }}</span><span class="info-note">V/A</span></div>
        <div class="info-row"><span class="info-label">Ki:</span><span class="info-value highlight">{{ ki }}</span><span class="info-note">V/A/s</span></div>
        <div class="info-row"><span class="info-label">等效带宽:</span><span class="info-value">{{ effectiveBw.toFixed(0) }} Hz</span></div>
      </div>
      <div class="info-block">
        <div class="info-block-title">负载参数</div>
        <div class="info-row"><span class="info-label">R:</span><span class="info-value">{{ motorR }} &#937;</span></div>
        <div class="info-row"><span class="info-label">L:</span><span class="info-value">{{ motorL }} mH</span></div>
        <div class="info-row"><span class="info-label">&#964;e:</span><span class="info-value">{{ (motorL / motorR).toFixed(2) }} ms</span><span class="info-note">电气时间常数</span></div>
      </div>
      <div class="info-block">
        <div class="info-block-title">动态指标</div>
        <div class="info-row"><span class="info-label">tr:</span><span class="info-value highlight">{{ metrics.riseTime.toFixed(2) }} ms</span></div>
        <div class="info-row"><span class="info-label">ts:</span><span class="info-value highlight">{{ metrics.settlingTime.toFixed(2) }} ms</span></div>
        <div class="info-row"><span class="info-label">超调:</span><span class="info-value" :style="{ color: metrics.overshoot > 10 ? '#ef4444' : '#10b981' }">{{ metrics.overshoot.toFixed(1) }}%</span></div>
        <div class="info-row"><span class="info-label">相角裕度:</span><span class="info-value">{{ metrics.phaseMargin.toFixed(1) }}&deg;</span></div>
      </div>
    </div>

    <div class="explain-panel">
      <details open>
        <summary>PI 电流调节器带宽参数化原理</summary>
        <div class="explain-content">
          <p><strong>内模原理：</strong>FOC 的 dq 电流环是最内层控制回路，控制带宽设计直接影响转矩响应速度和整个驱动系统的动态性能。电流环的典型设计目标是在保证稳定裕度的前提下尽可能提高带宽。</p>
          <p><strong>带宽参数化整定法：</strong>将 PI 参数直接与目标带宽 &#969;bw 关联，是目前工业界最常用的方法：</p>
          <p><strong>Kp = &#969;bw &middot; L</strong> — 比例增益抵消电感产生的惯性延迟，提供快速的动态响应。</p>
          <p><strong>Ki = &#969;bw &middot; R</strong> — 积分增益抵消电阻压降引起的稳态误差，消除静差。</p>
          <p>这种方式下闭环传递函数简化为 <strong>I(s)/I*(s) = &#969;bw/(s + &#969;bw)</strong>，即一阶低通滤波器，无超调无振荡。电流环带宽一般设为 PWM 频率的 1/20~1/10。</p>
          <p><strong>dq 解耦：</strong>实际系统中 dq 轴存在交叉耦合（&#969;e&middot;L&middot;iq 和 &#969;e&middot;L&middot;id），需加入前馈解耦项使 PI 调节器独立工作。</p>
        </div>
      </details>
    </div>
  </div>
</template>

<style scoped>
.sim-container { border: 1px solid var(--border-color); border-radius: 8px; padding: 0.8rem; margin: 1.5rem 0; background: var(--bg-color); }
.sim-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 0.6rem; flex-wrap: wrap; gap: 0.5rem; }
.sim-header h4 { margin: 0; font-size: 0.92rem; color: var(--primary-color); }
.header-controls { display: flex; gap: 0.5rem; align-items: center; }
.auto-btn { padding: 0.28rem 0.75rem; border: 1px solid var(--border-color); border-radius: 6px; background: var(--content-bg); color: var(--text-color); cursor: pointer; font-size: 0.76rem; font-weight: 500; transition: all 0.15s; }
.auto-btn.active { background: #10b981; color: #fff; border-color: #10b981; }
.auto-btn:hover:not(.active) { background: var(--bg-color); }
.sim-canvas { display: block; margin: 0 auto; max-width: 100%; border-radius: 6px; border: 1px solid var(--border-color); }
.sim-params { display: flex; flex-wrap: wrap; gap: 1.5rem; margin-top: 0.6rem; justify-content: center; }
.param-group { display: flex; flex-direction: column; align-items: center; gap: 0.15rem; font-size: 0.78rem; min-width: 185px; }
.param-group label { font-weight: 600; color: var(--text-color); }
.param-val { color: var(--primary-color); font-weight: 700; }
.param-group input[type="range"] { width: 100%; cursor: pointer; }
.param-range { font-size: 0.68rem; color: #0ea5e9; }
.sim-info { display: flex; flex-wrap: wrap; gap: 0.8rem; margin-top: 0.6rem; padding: 0.65rem; background: var(--content-bg); border: 1px solid var(--border-color); border-radius: 6px; font-size: 0.75rem; justify-content: center; }
.info-block { display: flex; flex-direction: column; gap: 0.15rem; min-width: 180px; max-width: 280px; }
.info-block-title { font-weight: 700; color: var(--primary-color); margin-bottom: 0.12rem; border-bottom: 1px solid var(--border-color); padding-bottom: 0.12rem; font-size: 0.8rem; }
.info-row { display: flex; gap: 0.2rem; align-items: center; flex-wrap: wrap; }
.info-label { font-weight: 600; color: #0ea5e9; white-space: nowrap; font-size: 0.75rem; }
.info-value { color: var(--text-color); font-family: 'Consolas','Monaco',monospace; font-size: 0.72rem; }
.info-value.highlight { color: var(--primary-color); font-weight: 700; }
.info-note { font-size: 0.62rem; color: #0ea5e9; font-style: italic; }
.explain-panel { margin-top: 0.6rem; padding: 0.55rem 0.7rem; background: var(--content-bg); border: 1px solid var(--border-color); border-radius: 6px; font-size: 0.76rem; }
.explain-panel details summary { font-weight: 600; color: var(--primary-color); cursor: pointer; font-size: 0.82rem; }
.explain-panel .explain-content { margin-top: 0.45rem; }
.explain-content p { margin: 0.35rem 0; color: var(--text-secondary); line-height: 1.45; }
</style>