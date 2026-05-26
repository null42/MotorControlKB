<script setup lang="ts">
import { ref, computed, watch, onMounted, onUnmounted } from 'vue'

/**
 * 初始位置检测仿真 -- 电感饱和法 / 预定位 / 高频注入法
 *
 * Canvas 920×560 三栏:
 *   上栏 (0~220): 电感饱和法 — 6方向脉冲注入, 电流响应柱状图, 最大方向标注
 *   中栏 (225~420): 预定位 — 转子从初始角→目标角(0°)的指数收敛动画
 *   下栏 (425~560): 高频注入法 — 脉振电压注入, q轴电流, PLL锁定
 */

const canvasRef = ref<HTMLCanvasElement | null>(null)
let animFrameId = 0
let lastTs = 0

const initAngle = ref(135)
const detectionMethod = ref<'inductance' | 'preposition' | 'hfi'>('inductance')
const Ld = ref(5)
const Lq = ref(10)
const injectionVoltage = ref(8)
const injectionFreq = ref(1000)
const kpPll = ref(80)
const kiPll = ref(400)

const simTime = ref(0)
const SIM_DURATION = 0.3

const detectedAngle = ref(0)
const detectionStatus = ref<'idle' | 'detecting' | 'done'>('idle')
const detectionStep = ref(0)
let stepTimer: ReturnType<typeof setTimeout> | null = null
let stepInterval: ReturnType<typeof setInterval> | null = null

const barCurrents = ref<number[]>(new Array(6).fill(0))
const maxBarIndex = ref(-1)

const prepositionAngle = ref(0)
const hfiThetaEst = ref(0)
const hfiTime = ref(0)
const HFI_HISTORY_LEN = 200
const hfiEstHistory = ref<number[]>([])
const hfiQCurrentHistory = ref<number[]>([])

const toRad = (deg: number) => (deg * Math.PI) / 180
const angleDiff = (a: number, b: number) => {
  let d = ((a - b) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI)
  if (d > Math.PI) d -= 2 * Math.PI
  return d
}

const metrics = computed(() => {
  if (detectionMethod.value === 'inductance') {
    const err = Math.abs(angleDiff(toRad(detectedAngle.value), toRad(initAngle.value)))
    return { errDeg: (err * 180 / Math.PI).toFixed(1), state: detectionStatus.value === 'done' ? '完成' : '检测中...' }
  } else if (detectionMethod.value === 'preposition') {
    const err = Math.abs(angleDiff(toRad(prepositionAngle.value * 180 / Math.PI), 0))
    return { errDeg: (err * 180 / Math.PI).toFixed(1), state: err < 0.02 ? '已对齐' : '对齐中...' }
  } else {
    const err = Math.abs(angleDiff(hfiThetaEst.value, toRad(initAngle.value)))
    return { errDeg: (err * 180 / Math.PI).toFixed(1), state: err < 0.03 ? '已锁定' : '锁定中...' }
  }
})

function cleanupStepTimers() {
  if (stepTimer) { clearTimeout(stepTimer); stepTimer = null }
  if (stepInterval) { clearInterval(stepInterval); stepInterval = null }
}

function runInductanceDetect() {
  cleanupStepTimers()
  detectionStatus.value = 'detecting'
  detectionStep.value = 1
  barCurrents.value = new Array(6).fill(0)
  maxBarIndex.value = -1
  draw()

  const realAngleRad = toRad(initAngle.value)
  const L_avg = (Ld.value + Lq.value) / 2
  const dL = (Ld.value - Lq.value) / 2
  const Uinj = injectionVoltage.value
  const dt = 50 * 1e-6

  stepTimer = setTimeout(() => {
    detectionStep.value = 2
    const currents: number[] = []
    let maxVal = 0
    let maxIdx = 0
    let k = 0

    stepInterval = setInterval(() => {
      const theta_k = toRad(k * 60)
      const Lk = L_avg + dL * Math.cos(2 * (theta_k - realAngleRad))
      const Ik = (Uinj / Math.max(Lk * 1e-3, 0.01)) * dt
      const noise = (Math.random() - 0.5) * 0.1
      const val = Math.max(0, Ik + noise)
      currents.push(val)
      if (val > maxVal) { maxVal = val; maxIdx = k }
      barCurrents.value = [...currents]
      draw()

      k++
      if (k >= 6) {
        clearInterval(stepInterval!)
        stepInterval = null
        maxBarIndex.value = maxIdx

        stepTimer = setTimeout(() => {
          detectionStep.value = 3
          draw()

          stepTimer = setTimeout(() => {
            detectionStep.value = 4
            const detectedDeg = maxIdx * 60
            const thetaD = toRad(detectedDeg)
            const errAlign = angleDiff(thetaD, realAngleRad)
            const satSign = Math.cos(errAlign)
            const satFactor = 0.12
            const LdBase = (L_avg + dL) * 1e-3
            const L_eff = LdBase * (1 - satSign * satFactor)
            const iN = (Uinj * 1.3 / Math.max(L_eff, 0.00001)) * dt
            const L_eff_opp = LdBase * (1 + satSign * satFactor)
            const iS = (Uinj * 1.3 / Math.max(L_eff_opp, 0.00001)) * dt
            draw()

            stepTimer = setTimeout(() => {
              detectionStep.value = 5
              if (iN > iS) {
                detectedAngle.value = detectedDeg
              } else {
                detectedAngle.value = (detectedDeg + 180) % 360
              }
              detectionStatus.value = 'done'
              detectionStep.value = 6
              draw()
            }, 400)
          }, 500)
        }, 400)
      }
    }, 180)
  }, 350)
}

function runPreposition(ts: number) {
  if (lastTs === 0) { lastTs = ts; prepositionAngle.value = toRad(initAngle.value); return }
  const dt = Math.min((ts - lastTs) / 1000, 0.05)
  lastTs = ts

  const tau = 0.08
  const err = angleDiff(0, prepositionAngle.value)
  prepositionAngle.value += err * (1 - Math.exp(-dt / tau))
  prepositionAngle.value = ((prepositionAngle.value % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)
}

function runHFI(ts: number) {
  if (lastTs === 0) {
    lastTs = ts
    hfiTime.value = 0
    hfiThetaEst.value = toRad((initAngle.value + 180) % 360)
    hfiEstHistory.value = []
    hfiQCurrentHistory.value = []
    return
  }
  const dt = Math.min((ts - lastTs) / 1000, 0.05)
  lastTs = ts
  hfiTime.value += dt
  if (hfiTime.value > SIM_DURATION) hfiTime.value = SIM_DURATION

  const thetaReal = toRad(initAngle.value)
  const Uh = injectionVoltage.value
  const wh = 2 * Math.PI * injectionFreq.value
  const L_avg = ((Ld.value + Lq.value) / 2) * 1e-3
  const dL = ((Lq.value - Ld.value) / 2) * 1e-3
  const errRad = angleDiff(thetaReal, hfiThetaEst.value)

  const iqh = (Uh / (wh * L_avg + 0.00001)) * (dL / (L_avg + 0.00001)) * Math.sin(2 * errRad) * Math.sin(wh * hfiTime.value)
  const demodSignal = iqh * Math.sin(wh * hfiTime.value)
  const lpAlpha = 0.05
  const filteredErr = demodSignal * lpAlpha + (hfiQCurrentHistory.value.length > 0 ? parseFloat(hfiQCurrentHistory.value[hfiQCurrentHistory.value.length - 1]?.split(',')[1] || '0') : 0) * (1 - lpAlpha)

  const kp = kpPll.value
  const ki = kiPll.value
  const omegaEst = kp * filteredErr + ki * filteredErr * 0.001
  hfiThetaEst.value += omegaEst * dt
  hfiThetaEst.value = ((hfiThetaEst.value % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)

  if (hfiEstHistory.value.length >= HFI_HISTORY_LEN) {
    hfiEstHistory.value.shift()
    hfiQCurrentHistory.value.shift()
  }
  hfiEstHistory.value.push(`${hfiTime.value.toFixed(4)},${hfiThetaEst.value.toFixed(4)}`)
  hfiQCurrentHistory.value.push(`${hfiTime.value.toFixed(4)},${iqh.toFixed(4)}`)
}

function animationLoop(ts: number) {
  try {
    if (detectionMethod.value === 'preposition') {
      runPreposition(ts)
    } else if (detectionMethod.value === 'hfi') {
      runHFI(ts)
    }
    draw()
  } catch { /* keep loop alive */ }
  animFrameId = requestAnimationFrame(animationLoop)
}

function resetAnimation() {
  cleanupStepTimers()
  lastTs = 0
  prepositionAngle.value = toRad(initAngle.value)
  hfiThetaEst.value = toRad((initAngle.value + 180) % 360)
  hfiTime.value = 0
  hfiEstHistory.value = []
  hfiQCurrentHistory.value = []
  detectionStatus.value = 'idle'
  detectionStep.value = 0
  barCurrents.value = new Array(6).fill(0)
  maxBarIndex.value = -1
  if (detectionMethod.value === 'inductance') {
    runInductanceDetect()
  }
}

watch(detectionMethod, () => resetAnimation())
watch(initAngle, () => resetAnimation())
watch([Ld, Lq, injectionVoltage, injectionFreq, kpPll, kiPll], () => {
  if (detectionMethod.value === 'inductance') runInductanceDetect()
  else { lastTs = 0; hfiTime.value = 0; hfiEstHistory.value = []; hfiQCurrentHistory.value = [] }
})

function draw() {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  const W = 920, H = 560
  canvas.width = W; canvas.height = H

  const style = getComputedStyle(document.documentElement)
  const bgColor = style.getPropertyValue('--bg-color').trim() || '#f8fafc'
  const textColor = style.getPropertyValue('--text-color').trim() || '#1e293b'
  const borderColor = style.getPropertyValue('--border-color').trim() || '#e2e8f0'
  const primaryColor = style.getPropertyValue('--primary-color').trim() || '#3b82f6'
  const textSec = style.getPropertyValue('--text-secondary').trim() || '#64748b'
  const contentBg = style.getPropertyValue('--content-bg').trim() || '#ffffff'

  ctx.clearRect(0, 0, W, H)
  ctx.fillStyle = bgColor; ctx.fillRect(0, 0, W, H)

  const SEP1 = 222, SEP2 = 422

  // 分隔线
  ctx.strokeStyle = borderColor; ctx.lineWidth = 1; ctx.setLineDash([6, 4])
  ctx.beginPath(); ctx.moveTo(12, SEP1); ctx.lineTo(W - 12, SEP1); ctx.stroke()
  ctx.beginPath(); ctx.moveTo(12, SEP2); ctx.lineTo(W - 12, SEP2); ctx.stroke()
  ctx.setLineDash([])

  drawInductanceSection(ctx, W, 0, 222, textColor, textSec, borderColor, primaryColor, contentBg)
  drawPrepositionSection(ctx, W, 222, 200, textColor, textSec, borderColor, primaryColor, contentBg)
  drawHFISection(ctx, W, 422, 138, textColor, textSec, borderColor, primaryColor, contentBg)
}

function drawInductanceSection(ctx: CanvasRenderingContext2D, W: number, top: number, h: number, textColor: string, textSec: string, borderColor: string, primaryColor: string, contentBg: string) {
  const pad = 40, barAreaW = W - pad * 2, barAreaTop = top + 32, barAreaH = h - 48
  ctx.fillStyle = textColor; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('电感饱和法 — 6方向脉冲注入 & 电流响应柱状图', W / 2, top + 14)

  if (maxBarIndex.value < 0 && detectionStatus.value === 'idle') {
    ctx.fillStyle = textSec; ctx.font = '10px sans-serif'
    ctx.fillText('调整参数后自动检测...', W / 2, barAreaTop + barAreaH / 2)
    return
  }

  const angles = ['0°', '60°', '120°', '180°', '240°', '300°']
  const colors = ['#ef4444', '#f97316', '#eab308', '#22c55e', '#06b6d4', '#8b5cf6']
  const barW = (barAreaW - 40) / 6 - 12
  const maxVal = Math.max(...barCurrents.value, 1)

  let totalWidth = 6 * (barW + 12) - 12
  let startX = pad + (barAreaW - totalWidth) / 2

  for (let k = 0; k < 6; k++) {
    const x = startX + k * (barW + 12)
    const barH = (barCurrents.value[k] / maxVal) * (barAreaH - 30)
    const y = barAreaTop + barAreaH - barH - 4

    ctx.fillStyle = k === maxBarIndex.value ? '#ef4444' : colors[k] + '88'
    ctx.fillRect(x, y, barW, barH)

    if (k === maxBarIndex.value) {
      ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 2
      ctx.strokeRect(x - 2, y - 2, barW + 4, barH + 4)
      ctx.fillStyle = '#ef4444'; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'center'
      ctx.fillText('MAX', x + barW / 2, y - 6)
    }

    ctx.fillStyle = textColor; ctx.font = '9px sans-serif'; ctx.textAlign = 'center'
    ctx.fillText(angles[k], x + barW / 2, barAreaTop + barAreaH - 2)

    ctx.fillStyle = textSec; ctx.font = '8px monospace'; ctx.textAlign = 'center'
    ctx.fillText(barCurrents.value[k].toFixed(2), x + barW / 2, y - 2)
  }

  // 电角度指示
  ctx.fillStyle = textSec; ctx.font = '9px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText(`实际转子角: ${initAngle.value}° | 检测结果: ${detectionStatus.value === 'done' ? detectedAngle.value + '°' : '--'}`, pad, barAreaTop + barAreaH + 6)
}

function drawPrepositionSection(ctx: CanvasRenderingContext2D, W: number, top: number, h: number, textColor: string, textSec: string, borderColor: string, primaryColor: string, contentBg: string) {
  ctx.fillStyle = textColor; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('预定位 (DC Alignment) — 转子收敛到 d 轴 (0°)', W / 2, top + 14)

  const cx = W / 2, cy = top + 110, radius = 60
  const initialRad = toRad(initAngle.value)
  const currentRad = detectionMethod.value === 'preposition' ? prepositionAngle.value : initialRad
  const err = angleDiff(0, currentRad)

  ctx.strokeStyle = borderColor; ctx.lineWidth = 1
  ctx.beginPath(); ctx.arc(cx, cy, radius, 0, 2 * Math.PI); ctx.stroke()

  // d轴 (0°) 标记
  const dX = cx + radius * 1.15, dY = cy
  ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 2; ctx.setLineDash([4, 3])
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(dX, dY); ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#ef4444'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('d轴 (0°)', dX + 20, dY + 4)

  // 转子指示线
  const rX = cx + radius * Math.cos(currentRad)
  const rY = cy - radius * Math.sin(currentRad)
  ctx.strokeStyle = primaryColor; ctx.lineWidth = 2.5
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(rX, rY); ctx.stroke()

  // 转子圆圈
  ctx.fillStyle = primaryColor; ctx.beginPath(); ctx.arc(rX, rY, 5, 0, 2 * Math.PI); ctx.fill()

  // N/S 标记
  ctx.fillStyle = '#ef4444'; ctx.font = 'bold 8px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('N', cx + radius * Math.cos(currentRad) * 0.75, cy - radius * Math.sin(currentRad) * 0.75 - 8)
  ctx.fillText('S', cx - radius * Math.cos(currentRad) * 0.75, cy + radius * Math.sin(currentRad) * 0.75 + 12)

  // 初始角度虚线
  ctx.strokeStyle = textSec + '66'; ctx.lineWidth = 1; ctx.setLineDash([3, 3])
  const iX = cx + (radius + 10) * Math.cos(initialRad)
  const iY = cy - (radius + 10) * Math.sin(initialRad)
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(iX, iY); ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = textSec; ctx.font = '8px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText(`初始 ${initAngle.value}°`, iX + (iX > cx ? 18 : -18), iY - 6)

  // 误差显示
  const errDeg = (err * 180 / Math.PI)
  ctx.fillStyle = errDeg < 5 ? '#10b981' : '#f59e0b'; ctx.font = 'bold 10px monospace'; ctx.textAlign = 'center'
  ctx.fillText(`角度误差: ${errDeg.toFixed(1)}° ${errDeg < 2 ? '✓ 已对齐' : ''}`, cx, cy + radius + 24)

  // 收敛曲线提示
  ctx.fillStyle = textSec; ctx.font = '9px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText('τ = 2J / (3P²·ψf·I_align)', 24, top + h - 6)
}

function drawHFISection(ctx: CanvasRenderingContext2D, W: number, top: number, h: number, textColor: string, textSec: string, borderColor: string, primaryColor: string, contentBg: string) {
  ctx.fillStyle = textColor; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('高频注入法 (脉振) — d轴正弦注入 → q轴电流 → PLL锁定', W / 2, top + 14)

  const plotL = 50, plotR = W - 30, plotW = plotR - plotL, plotT = top + 22, plotH = h - 34

  ctx.strokeStyle = borderColor; ctx.lineWidth = 0.6
  ctx.strokeRect(plotL, plotT, plotW, plotH)

  const targetRad = toRad(initAngle.value)

  // 真实位置线
  ctx.strokeStyle = '#10b981'; ctx.lineWidth = 1; ctx.setLineDash([4, 3])
  const targetY = plotT + plotH / 2
  ctx.beginPath(); ctx.moveTo(plotL, targetY); ctx.lineTo(plotR, targetY); ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#10b981'; ctx.font = '8px sans-serif'; ctx.textAlign = 'right'
  ctx.fillText(`θ_real = ${initAngle.value}°`, plotL - 4, targetY + 3)

  // 估算曲线
  if (hfiEstHistory.value.length > 1) {
    ctx.strokeStyle = primaryColor; ctx.lineWidth = 2
    ctx.beginPath()
    for (let i = 0; i < hfiEstHistory.value.length; i++) {
      const [tStr, thStr] = hfiEstHistory.value[i].split(',')
      const x = plotL + (parseFloat(tStr) / SIM_DURATION) * plotW
      const thRad = parseFloat(thStr)
      const err = angleDiff(targetRad, thRad)
      const y = targetY - (err / 1.5) * (plotH / 2 - 8)
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
    }
    ctx.stroke()

    // 当前估算标记
    if (hfiEstHistory.value.length > 0) {
      const last = hfiEstHistory.value[hfiEstHistory.value.length - 1]
      const [lt, lth] = last.split(',')
      const lx = plotL + (parseFloat(lt) / SIM_DURATION) * plotW
      const lerr = angleDiff(targetRad, parseFloat(lth))
      const ly = targetY - (lerr / 1.5) * (plotH / 2 - 8)
      ctx.fillStyle = primaryColor; ctx.beginPath(); ctx.arc(lx, ly, 4, 0, 2 * Math.PI); ctx.fill()
    }
  }

  // q轴电流
  if (hfiQCurrentHistory.value.length > 1) {
    const iqhPlotT = targetY + plotH / 4
    ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 1
    ctx.beginPath()
    let maxIqh = 0
    for (const rec of hfiQCurrentHistory.value) {
      const iqhVal = Math.abs(parseFloat(rec.split(',')[1]))
      if (iqhVal > maxIqh) maxIqh = iqhVal
    }
    maxIqh = Math.max(maxIqh, 0.5)
    for (let i = 0; i < hfiQCurrentHistory.value.length; i++) {
      const [tStr, iqhStr] = hfiQCurrentHistory.value[i]
      const x = plotL + (parseFloat(tStr) / SIM_DURATION) * plotW
      const y = iqhPlotT - (parseFloat(iqhStr) / maxIqh) * (plotH / 4 - 4)
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
    }
    ctx.stroke()
    ctx.fillStyle = '#f59e0b'; ctx.font = '8px sans-serif'; ctx.textAlign = 'right'
    ctx.fillText('iq_hf', plotL - 4, iqhPlotT + 3)
  }

  // 图例
  ctx.fillStyle = primaryColor; ctx.font = 'bold 8px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText('θ_est (PLL)', plotL + 6, plotT + 12)
  ctx.fillStyle = '#10b981'
  ctx.fillText('θ_real', plotL + 6, plotT + 24)
  ctx.fillStyle = '#f59e0b'
  ctx.fillText('iq_hf', plotL + 6, plotT + 36)
}

onMounted(() => {
  runInductanceDetect()
  animFrameId = requestAnimationFrame(animationLoop)
})
onUnmounted(() => {
  cancelAnimationFrame(animFrameId)
  cleanupStepTimers()
})
</script>

<template>
  <div class="sim-container">
    <div class="sim-header">
      <h4>启动定位与预定位仿真 -- 电感饱和法 / 预定位 / 高频注入法对比</h4>
      <div class="header-controls">
        <div class="mode-selector">
          <button :class="{ active: detectionMethod === 'inductance' }"
            @click="detectionMethod = 'inductance'">电感饱和法</button>
          <button :class="{ active: detectionMethod === 'preposition' }"
            @click="detectionMethod = 'preposition'">预定位</button>
          <button :class="{ active: detectionMethod === 'hfi' }"
            @click="detectionMethod = 'hfi'">高频注入</button>
        </div>
      </div>
    </div>

    <div class="workflow-bar" v-if="detectionMethod === 'inductance'">
      <div class="workflow-steps">
        <div class="wf-step" :class="{ active: detectionStep === 1, done: detectionStep > 1 }">
          <div class="wf-number">1</div>
          <div class="wf-label">电压脉冲注入</div>
        </div>
        <div class="wf-arrow">→</div>
        <div class="wf-step" :class="{ active: detectionStep === 2, done: detectionStep > 2 }">
          <div class="wf-number">2</div>
          <div class="wf-label">电流响应比较</div>
        </div>
        <div class="wf-arrow">→</div>
        <div class="wf-step" :class="{ active: detectionStep === 3, done: detectionStep > 3 }">
          <div class="wf-number">3</div>
          <div class="wf-label">最大电流方向</div>
        </div>
        <div class="wf-arrow">→</div>
        <div class="wf-step" :class="{ active: detectionStep === 4, done: detectionStep > 4 }">
          <div class="wf-number">4</div>
          <div class="wf-label">N/S 极性判断</div>
        </div>
        <div class="wf-arrow">→</div>
        <div class="wf-step" :class="{ active: detectionStep === 5, done: detectionStep > 5 }">
          <div class="wf-number">5</div>
          <div class="wf-label">确定最终角度</div>
        </div>
      </div>
    </div>

    <canvas ref="canvasRef" class="sim-canvas"></canvas>

    <div class="sim-params">
      <div class="param-group">
        <label>初始角度: <span class="param-val">{{ initAngle }}&deg;</span></label>
        <input type="range" v-model.number="initAngle" :min="0" :max="360" :step="1" />
        <span class="param-range">0&deg; ~ 360&deg;</span>
      </div>
      <div class="param-group">
        <label>Ld (mH): <span class="param-val">{{ Ld }}</span></label>
        <input type="range" v-model.number="Ld" :min="1" :max="20" :step="0.5" />
        <span class="param-range">1 ~ 20 mH</span>
      </div>
      <div class="param-group">
        <label>Lq (mH): <span class="param-val">{{ Lq }}</span></label>
        <input type="range" v-model.number="Lq" :min="1" :max="30" :step="0.5" />
        <span class="param-range">1 ~ 30 mH</span>
      </div>
      <div class="param-group">
        <label>注入电压: <span class="param-val">{{ injectionVoltage }}V</span></label>
        <input type="range" v-model.number="injectionVoltage" :min="2" :max="20" :step="0.5" />
        <span class="param-range">2 ~ 20 V</span>
      </div>
      <div class="param-group" v-if="detectionMethod === 'hfi'">
        <label>注入频率: <span class="param-val">{{ injectionFreq }}Hz</span></label>
        <input type="range" v-model.number="injectionFreq" :min="200" :max="2000" :step="100" />
        <span class="param-range">200 ~ 2000 Hz</span>
      </div>
      <div class="param-group" v-if="detectionMethod === 'hfi'">
        <label>PLL Kp: <span class="param-val">{{ kpPll }}</span></label>
        <input type="range" v-model.number="kpPll" :min="10" :max="200" :step="5" />
        <span class="param-range">10 ~ 200</span>
      </div>
    </div>

    <div class="sim-info">
      <div class="info-block">
        <div class="info-block-title">{{ detectionMethod === 'inductance' ? '电感饱和法' : detectionMethod === 'preposition' ? '预定位' : '高频注入' }} 状态</div>
        <div class="info-row"><span class="info-label">检测状态:</span><span class="info-value highlight">{{ metrics.state }}</span></div>
        <div class="info-row"><span class="info-label">位置误差:</span><span class="info-value highlight">{{ metrics.errDeg }}&deg;</span></div>
      </div>
      <div class="info-block">
        <div class="info-block-title">检测结果</div>
        <div class="info-row" v-if="detectionMethod === 'inductance'">
          <span class="info-label">检测角度:</span><span class="info-value">{{ detectionStatus === 'done' ? detectedAngle + '°' : '--' }}</span>
        </div>
        <div class="info-row" v-if="detectionMethod === 'inductance'">
          <span class="info-label">最大扇区:</span><span class="info-value">{{ maxBarIndex >= 0 ? maxBarIndex + 1 : '--' }}</span>
        </div>
        <div class="info-row" v-if="detectionMethod === 'preposition'">
          <span class="info-label">当前角度:</span><span class="info-value">{{ (prepositionAngle * 180 / Math.PI).toFixed(1) }}&deg;</span>
        </div>
        <div class="info-row" v-if="detectionMethod === 'hfi'">
          <span class="info-label">估算角度:</span><span class="info-value">{{ (hfiThetaEst * 180 / Math.PI).toFixed(1) }}&deg;</span>
        </div>
      </div>
      <div class="info-block">
        <div class="info-block-title">电机参数</div>
        <div class="info-row"><span class="info-label">Ld:</span><span class="info-value">{{ Ld }} mH</span></div>
        <div class="info-row"><span class="info-label">Lq:</span><span class="info-value">{{ Lq }} mH</span></div>
        <div class="info-row"><span class="info-label">凸极率:</span><span class="info-value">{{ ((Lq - Ld) / (Ld + Lq + 0.001)).toFixed(3) }}</span></div>
      </div>
    </div>

    <div class="explain-panel">
      <details open>
        <summary>初始位置检测方法原理对比</summary>
        <div class="explain-content">
          <p><strong>电感饱和法：</strong>利用凸极效应（$L_d \neq L_q$），向6个方向注入等幅等宽电压脉冲，比较各方向电流响应。电感最小的方向（即d轴方向）电流最大。通过N/S极判断消除180°模糊。优点：转子不转动；缺点：需要凸极效应。</p>
          <p><strong>预定位 (DC Alignment)：</strong>给定固定电压矢量（$I_d$=常数, $I_q$=0），转子在电磁转矩作用下旋转到与d轴对齐。定位时间取决于转动惯量和对齐电流。优点：实现极简，适用于所有电机；缺点：转子会转动一个角度。</p>
          <p><strong>高频注入法 (脉振)：</strong>在估计d轴注入高频正弦电压，q轴高频电流响应包含位置误差信息 $\propto \sin(2\theta_{err})$。通过乘法解调+低通滤波提取误差信号，PLL跟踪收敛到真实位置。优点：精度最高（±3°），转子不转动；缺点：实现复杂，需要凸极效应。</p>
          <p><strong>选型建议：</strong>允许转子转动 → 预定位（首选）；不允许转动 + 凸极效应 → 高频注入（最佳精度）或电感饱和法（实现简单）；SPMSM不允许转动 → INFORM法。</p>
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
.mode-selector { display: flex; gap: 0; border-radius: 6px; overflow: hidden; border: 1px solid var(--border-color); }
.mode-selector button { padding: 0.28rem 0.75rem; border: none; background: var(--content-bg); color: var(--text-secondary); cursor: pointer; font-size: 0.76rem; font-weight: 500; transition: all 0.15s; white-space: nowrap; }
.mode-selector button.active { background: var(--primary-color); color: #fff; }
.mode-selector button:hover:not(.active) { background: var(--bg-color); }
.workflow-bar {
  padding: 0.5rem 0.8rem;
  margin-bottom: 0.6rem;
  background: var(--content-bg);
  border: 1px solid var(--border-color);
  border-radius: 8px;
  display: flex;
  justify-content: center;
}
.workflow-steps {
  display: flex;
  align-items: center;
  gap: 0.15rem;
  flex-wrap: wrap;
  justify-content: center;
}
.wf-step {
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 0.3rem 0.5rem;
  border-radius: 8px;
  min-width: 80px;
  transition: all 0.3s;
  opacity: 0.4;
  background: var(--bg-color);
}
.wf-step.active {
  opacity: 1;
  background: var(--primary-color)15;
  border: 1px solid var(--primary-color)30;
}
.wf-step.done {
  opacity: 1;
  background: #10b98115;
  border: 1px solid #10b98130;
}
.wf-number {
  width: 24px;
  height: 24px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 0.72rem;
  font-weight: 700;
  background: var(--border-color);
  color: var(--text-secondary);
  transition: all 0.3s;
}
.wf-step.active .wf-number {
  background: var(--primary-color);
  color: white;
}
.wf-step.done .wf-number {
  background: #10b981;
  color: white;
}
.wf-label {
  font-size: 0.68rem;
  color: var(--text-secondary);
  margin-top: 0.2rem;
  text-align: center;
  white-space: nowrap;
}
.wf-step.active .wf-label, .wf-step.done .wf-label {
  color: var(--text-color);
  font-weight: 600;
}
.wf-arrow {
  font-size: 0.9rem;
  color: var(--border-color);
  margin: 0 -0.1rem;
}
.sim-canvas { display: block; margin: 0 auto; max-width: 100%; border-radius: 6px; border: 1px solid var(--border-color); }
.sim-params { display: flex; flex-wrap: wrap; gap: 1.2rem; margin-top: 0.6rem; justify-content: center; }
.param-group { display: flex; flex-direction: column; align-items: center; gap: 0.15rem; font-size: 0.78rem; min-width: 130px; }
.param-group label { font-weight: 600; color: var(--text-color); }
.param-val { color: var(--primary-color); font-weight: 700; }
.param-group input[type="range"] { width: 100%; cursor: pointer; }
.param-range { font-size: 0.68rem; color: #0ea5e9; }
.sim-info { display: flex; flex-wrap: wrap; gap: 0.8rem; margin-top: 0.6rem; padding: 0.65rem; background: var(--content-bg); border: 1px solid var(--border-color); border-radius: 6px; font-size: 0.75rem; justify-content: center; }
.info-block { display: flex; flex-direction: column; gap: 0.15rem; min-width: 170px; max-width: 260px; }
.info-block-title { font-weight: 700; color: var(--primary-color); margin-bottom: 0.12rem; border-bottom: 1px solid var(--border-color); padding-bottom: 0.12rem; font-size: 0.8rem; }
.info-row { display: flex; gap: 0.2rem; align-items: center; flex-wrap: wrap; }
.info-label { font-weight: 600; color: #0ea5e9; white-space: nowrap; font-size: 0.75rem; }
.info-value { color: var(--text-color); font-family: 'Consolas','Monaco',monospace; font-size: 0.72rem; }
.info-value.highlight { color: var(--primary-color); font-weight: 700; }
.explain-panel { margin-top: 0.6rem; padding: 0.55rem 0.7rem; background: var(--content-bg); border: 1px solid var(--border-color); border-radius: 6px; font-size: 0.76rem; }
.explain-panel details summary { font-weight: 600; color: var(--primary-color); cursor: pointer; font-size: 0.82rem; }
.explain-panel .explain-content { margin-top: 0.45rem; }
.explain-content p { margin: 0.35rem 0; color: var(--text-secondary); line-height: 1.45; }
</style>