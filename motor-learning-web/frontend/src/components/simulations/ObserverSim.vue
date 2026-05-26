<script setup lang="ts">
import { ref, computed, watch, onMounted, onUnmounted } from 'vue'

/**
 * 位置/速度观测器仿真 (SMO / Luenberger / PLL)
 * 对比估算 vs 实际转子位置与速度
 *
 * Canvas 920×520，上下两栏:
 *   上栏 (0~265): 波形对比 — θe实际/估算, 位置误差, 速度ω
 *   下栏 (270~520): 观测器收敛分析 — 收敛时间 vs 增益, 滑模面/极点
 */

const canvasRef = ref<HTMLCanvasElement | null>(null)
let animFrameId = 0

// --- 观测器参数 ---
const observerType = ref<'smo' | 'luenberger' | 'pll'>('smo')
const observerGain = ref(2.0)     // 观测器增益 (0.5 ~ 10)
const initAngleError = ref(30)    // 初始角度误差 °
const motorSpeed = ref(1500)      // 电机转速 RPM

// --- 仿真状态 ---
interface ObsPoint {
  t: number
  thetaActual: number    // 实际电角度 rad
  thetaEst: number       // 估算电角度 rad
  thetaErr: number       // 位置误差 rad
  omegaActual: number    // 实际电角速度 rad/s
  omegaEst: number       // 估算电角速度 rad/s
  slidingSurface: number // SMO 滑模面值
}
const history = ref<ObsPoint[]>([])
const SIM_TIME = 0.15           // 150ms 仿真
const SIM_STEPS = 300
const DT = SIM_TIME / SIM_STEPS

// 位置误差绝对值（用于显示）
const currentErr = ref(0)
const converged = computed(() => currentErr.value < 0.05)

// --- 运行观测器仿真 ---
function runSimulation() {
  const localHistory: ObsPoint[] = []
  const gain = observerGain.value * 100     // 缩放增益
  const omegaE = (motorSpeed.value / 60) * 2 * Math.PI  // 电角速度 (1对极简化)
  const initErrRad = (initAngleError.value * Math.PI) / 180

  let thetaAct = 0
  let thetaEst = initErrRad          // 估算初始值有偏差
  let omegaEst = 0
  let s = 0                          // 滑模面
  let integrator = 0                 // PLL 积分器

  for (let k = 0; k <= SIM_STEPS; k++) {
    const t = k * DT

    // 实际值: 匀速旋转
    thetaAct = (omegaE * t) % (2 * Math.PI)
    if (thetaAct < 0) thetaAct += 2 * Math.PI

    // 位置误差
    let err = thetaAct - thetaEst
    // 归一化到 [-PI, PI]
    while (err > Math.PI) err -= 2 * Math.PI
    while (err < -Math.PI) err += 2 * Math.PI

    // 根据不同观测器类型更新估算值
    if (observerType.value === 'smo') {
      // SMO: 符号函数滑模控制
      s = err  // 滑模面 = 位置误差
      const sw = Math.sign(s)
      const kSlide = gain * (1 - Math.exp(-t * 20))  // 滑模增益渐增
      omegaEst += (-kSlide * sw + (omegaE - omegaEst) * 0.3) * DT
      thetaEst += omegaEst * DT
    } else if (observerType.value === 'luenberger') {
      // Luenberger: 线性极点配置
      const lp = gain  // 极点位置
      const eOmega = omegaE - omegaEst
      thetaEst += (omegaEst + lp * lp * err) * DT
      omegaEst += (lp * lp * err) * DT
    } else {
      // PLL: 鉴相器 → PI 环路滤波器 → 积分器 → θ̂
      const kp_pll = gain * 0.5
      const ki_pll = gain * 0.1
      integrator += ki_pll * err * DT
      omegaEst = kp_pll * err + integrator
      thetaEst += omegaEst * DT
    }

    // 归一化估算角度
    thetaEst = ((thetaEst % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)

    // 更新误差
    let newErr = thetaAct - thetaEst
    while (newErr > Math.PI) newErr -= 2 * Math.PI
    while (newErr < -Math.PI) newErr += 2 * Math.PI

    localHistory.push({
      t, thetaActual: thetaAct, thetaEst,
      thetaErr: newErr,
      omegaActual: omegaE, omegaEst,
      slidingSurface: s,
    })
  }

  history.value = localHistory
  const last = localHistory[localHistory.length - 1]
  currentErr.value = Math.abs(last.thetaErr)
}

// --- 显示指标 ---
const metrics = computed(() => {
  const h = history.value
  if (h.length < 2) return { convergeTime: 0, steadyErr: 0, estOmega: 0 }
  let convTime = 0
  const threshold = (5 * Math.PI) / 180
  let settled = false
  for (let i = 1; i < h.length; i++) {
    if (!settled && Math.abs(h[i].thetaErr) < threshold) {
      let allGood = true
      for (let j = i; j < h.length; j++) {
        if (Math.abs(h[j].thetaErr) > threshold) { allGood = false; break }
      }
      if (allGood) { convTime = h[i].t * 1000; settled = true }
    }
  }
  if (!settled) convTime = SIM_TIME * 1000

  const last = h[h.length - 1]
  return {
    convergeTime: convTime,
    steadyErr: (Math.abs(last.thetaErr) * 180) / Math.PI,
    estOmega: last.omegaEst,
  }
})

// ═════════════════════════ 绘图 ═════════════════════════
const draw = () => {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  const W = 920, H = 520
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

  // 水平分隔线 y=265
  const HSEP = 268
  ctx.strokeStyle = borderColor; ctx.lineWidth = 1; ctx.setLineDash([6, 4])
  ctx.beginPath(); ctx.moveTo(12, HSEP); ctx.lineTo(W - 12, HSEP); ctx.stroke()
  ctx.setLineDash([])

  // ═════════════════════ 上栏: 波形对比 ═════════════════════
  const TL = 50, TT = 28, TW = W - 100, TH1 = 110, TH2 = 100
  const TB1 = TT + TH1, TB2 = HSEP - 14

  // 标题
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('θe 实际 (黑色) vs 估算 (红色虚线)', W / 2, TT - 8)

  const h = history.value
  if (h.length > 1) {
    const toX = (t: number) => TL + (t / SIM_TIME) * TW
    const thToY = (th: number) => TB1 - ((th % (2 * Math.PI)) / (2 * Math.PI)) * (TH1 - 16)
    const errToY = (e: number) => (TB1 + 24) + TH2 / 2 - (e / 0.8) * (TH2 / 2 - 8)

    // 子图1: θ 实际
    ctx.strokeStyle = '#1a1a2e'; ctx.lineWidth = 2.2
    ctx.beginPath()
    for (let i = 0; i < h.length; i++) {
      const x = toX(h[i].t), y = thToY(h[i].thetaActual)
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
    }
    ctx.stroke()

    // θ 估算
    ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 2; ctx.setLineDash([6, 3])
    ctx.beginPath()
    for (let i = 0; i < h.length; i++) {
      const x = toX(h[i].t), y = thToY(h[i].thetaEst)
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
    }
    ctx.stroke()
    ctx.setLineDash([])

    // 图例
    ctx.fillStyle = '#1a1a2e'; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'left'
    ctx.fillText('θe 实际', TL + 8, TT + 14)
    ctx.fillStyle = '#ef4444'
    ctx.fillText('θe 估算', TL + 80, TT + 14)
    ctx.textBaseline = 'alphabetic'

    // 子图2: 位置误差 (θ-θ̂)
    ctx.strokeStyle = borderColor; ctx.lineWidth = 0.5; ctx.setLineDash([2, 4])
    ctx.beginPath(); ctx.moveTo(TL, (TB1 + 24) + TH2 / 2); ctx.lineTo(TL + TW, (TB1 + 24) + TH2 / 2); ctx.stroke()
    ctx.setLineDash([])

    ctx.fillStyle = textSec; ctx.font = '9px sans-serif'; ctx.textAlign = 'center'
    ctx.fillText('位置误差 (rad)', TL - 32, (TB1 + 24) + TH2 / 2 + 3)

    ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 1.8
    ctx.beginPath()
    for (let i = 0; i < h.length; i++) {
      const x = toX(h[i].t), y = errToY(h[i].thetaErr)
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
    }
    ctx.stroke()

    // 收敛标注
    if (converged.value) {
      ctx.fillStyle = '#10b981'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'right'
      ctx.fillText('已收敛 ✓', TL + TW - 8, (TB1 + 24) + 12)
    }

    // 子图3: 速度 ω
    ctx.fillStyle = textSec; ctx.font = '9px sans-serif'; ctx.textAlign = 'center'
    ctx.fillText('速度 (rad/s)', TL - 32, TB2 - 22)

    const omegaVal = (motorSpeed.value / 60) * 2 * Math.PI
    const omegaMax = omegaVal * 1.5
    const wToY = (w: number) => TB2 - (w / omegaMax) * (TB2 - HSEP + 26)

    ctx.strokeStyle = '#1a1a2e'; ctx.lineWidth = 2
    ctx.beginPath()
    for (let i = 0; i < h.length; i++) {
      const x = toX(h[i].t), y = wToY(h[i].omegaActual)
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
    }
    ctx.stroke()

    ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 1.8; ctx.setLineDash([5, 4])
    ctx.beginPath()
    for (let i = 0; i < h.length; i++) {
      const x = toX(h[i].t), y = wToY(h[i].omegaEst)
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
    }
    ctx.stroke()
    ctx.setLineDash([])

    ctx.fillStyle = '#1a1a2e'; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'left'
    ctx.fillText('ω 实际', TL + 8, TB2 - 36)
    ctx.fillStyle = '#ef4444'
    ctx.fillText('ω 估算', TL + 80, TB2 - 36)
  } else {
    ctx.fillStyle = textSec; ctx.font = '14px sans-serif'; ctx.textAlign = 'center'
    ctx.fillText('仿真数据计算中，请稍候...', W / 2, TT + (TB2 - TT) / 2)
  }

  // 坐标 X 轴标签
  ctx.fillStyle = textSec; ctx.font = '9px sans-serif'; ctx.textAlign = 'right'
  ctx.fillText('0', TL - 4, TB1 - 4)
  ctx.fillText(`${(SIM_TIME * 1000).toFixed(0)}ms`, TL + TW, TB1 - 4)
  ctx.fillText('0', TL - 4, TB2 - 4)
  ctx.fillText(`${(SIM_TIME * 1000).toFixed(0)}ms`, TL + TW, TB2 - 4)

  // 上栏子图分隔线
  ctx.strokeStyle = borderColor; ctx.lineWidth = 0.4
  ctx.beginPath(); ctx.moveTo(TL, TB1 + 20); ctx.lineTo(TL + TW, TB1 + 20); ctx.stroke()

  // ═════════════════════ 下栏: 观测器收敛分析 ═════════════════════
  const BL = 50, BT = HSEP + 22, BW = W - 100, BH = H - BT - 14

  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('观测器收敛分析', W / 2, BT - 6)

  // 左侧: 收敛时间 vs 增益图
  const gainMin = 0.5, gainMax = 10
  const cPlotL = BL, cPlotW = BW * 0.45, cPlotT = BT + 10, cPlotH = BH * 0.65
  ctx.strokeStyle = borderColor; ctx.lineWidth = 0.6
  ctx.strokeRect(cPlotL, cPlotT, cPlotW, cPlotH)

  // 理论收敛曲线: τ_converge ∝ 1/gain
  ctx.strokeStyle = primaryColor; ctx.lineWidth = 2
  ctx.beginPath()
  const N_CURVE = 100
  for (let i = 0; i <= N_CURVE; i++) {
    const g = gainMin + (i / N_CURVE) * (gainMax - gainMin)
    const tau = 50 / g  // 收敛时间 ms
    const x = cPlotL + ((g - gainMin) / (gainMax - gainMin)) * cPlotW
    const y = cPlotT + cPlotH - (tau / 120) * cPlotH
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
  }
  ctx.stroke()

  // 当前增益标记
  const curGainX = cPlotL + ((observerGain.value - gainMin) / (gainMax - gainMin)) * cPlotW
  const curConvMs = metrics.value.convergeTime
  const curConvY = cPlotT + cPlotH - (curConvMs / 120) * cPlotH
  ctx.fillStyle = '#ef4444'; ctx.beginPath(); ctx.arc(curGainX, Math.max(cPlotT + 4, Math.min(curConvY, cPlotT + cPlotH - 4)), 5, 0, 2 * Math.PI); ctx.fill()

  ctx.fillStyle = textSec; ctx.font = '9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('增益 k', cPlotL + cPlotW / 2, cPlotT + cPlotH + 12)
  ctx.fillText('收敛时间 (ms)', cPlotL - 30, cPlotT + cPlotH / 2, 30)

  // 右侧: 极点/滑模面分析
  const rtL = cPlotL + cPlotW + 30, rtW = BW - cPlotW - 30, rtT = cPlotT, rtH = cPlotH
  ctx.strokeStyle = borderColor; ctx.lineWidth = 0.6
  ctx.strokeRect(rtL, rtT, rtW, rtH)

  if (observerType.value === 'smo') {
    // SMO: 滑模面
    ctx.strokeStyle = '#8b5cf6'; ctx.lineWidth = 1.5
    ctx.beginPath()
    for (let i = 0; i < h.length; i++) {
      const frac = i / (h.length - 1)
      const x = rtL + frac * rtW
      const sVal = h[i].slidingSurface
      const y = rtT + rtH / 2 - (sVal / 1.5) * (rtH / 2 - 8)
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
    }
    ctx.stroke()
    ctx.fillStyle = textColor; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'center'
    ctx.fillText('SMO 滑模面 s = is - is', rtL + rtW / 2, rtT + 12)
  } else if (observerType.value === 'luenberger') {
    // Luenberger: 极点分布
    const ppCX = rtL + rtW / 2, ppCY = rtT + rtH / 2
    ctx.strokeStyle = borderColor; ctx.lineWidth = 0.5
    ctx.beginPath(); ctx.moveTo(rtL + 20, ppCY); ctx.lineTo(rtL + rtW - 20, ppCY); ctx.stroke()
    ctx.beginPath(); ctx.moveTo(ppCX, rtT + 10); ctx.lineTo(ppCX, rtT + rtH - 10); ctx.stroke()
    ctx.fillStyle = '#ef4444'; ctx.beginPath(); ctx.arc(ppCX - 25, ppCY - 18, 4, 0, 2 * Math.PI); ctx.fill()
    ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'left'
    ctx.fillText('x (极点)', ppCX - 18, ppCY - 20)
    ctx.fillStyle = textColor; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'center'
    ctx.fillText('Luenberger 极点配置', rtL + rtW / 2, rtT + 12)
  } else {
    // PLL: 环路滤波器
    ctx.fillStyle = textColor; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'center'
    ctx.fillText('PLL 鉴相器 + PI 环路', rtL + rtW / 2, rtT + rtH / 2 - 8)
    ctx.fillText('θerr -> KPθ + KI∫θ -> ωest', rtL + rtW / 2, rtT + rtH / 2 + 12)
    ctx.fillStyle = textSec; ctx.font = '8px monospace'
    ctx.fillText(`${observerType.value.toUpperCase()} 带宽: ${(observerGain.value * 30).toFixed(0)} Hz`, rtL + rtW / 2, rtT + rtH / 2 + 30)
  }

  // 下方指标卡片
  const cardY = cPlotT + cPlotH + 22, cardW = 195, cardH = 54, cardGap = 14
  const cards = [
    { label: '收敛时间', value: `${metrics.value.convergeTime.toFixed(1)} ms`, color: primaryColor },
    { label: '稳态误差', value: `${metrics.value.steadyErr.toFixed(3)}°`, color: converged.value ? '#10b981' : '#ef4444' },
    { label: '估算速度 ω', value: `${metrics.value.estOmega.toFixed(1)} rad/s`, color: '#f59e0b' },
    { label: '收敛状态', value: converged.value ? '已收敛' : '收敛中...', color: converged.value ? '#10b981' : '#ef4444' },
  ]
  const cardStartX = BL + (BW - cards.length * (cardW + cardGap) + cardGap) / 2

  cards.forEach((c, i) => {
    const cx = cardStartX + i * (cardW + cardGap)
    ctx.fillStyle = contentBg; ctx.fillRect(cx, cardY, cardW, cardH)
    ctx.strokeStyle = borderColor; ctx.lineWidth = 0.6; ctx.strokeRect(cx, cardY, cardW, cardH)
    ctx.fillStyle = textSec; ctx.font = '9px sans-serif'; ctx.textAlign = 'center'
    ctx.fillText(c.label, cx + cardW / 2, cardY + 16)
    ctx.fillStyle = c.color; ctx.font = 'bold 13px monospace'
    ctx.fillText(c.value, cx + cardW / 2, cardY + 40)
  })
}

// ═════════════════════════ 动画循环 ═════════════════════════
const animate = (ts: number) => {
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
      <h4>位置 &amp; 速度观测器仿真 -- SMO / Luenberger / PLL 对比</h4>
      <div class="header-controls">
        <div class="mode-selector">
          <button :class="{ active: observerType === 'smo' }" @click="observerType = 'smo'; runSimulation()">SMO</button>
          <button :class="{ active: observerType === 'luenberger' }" @click="observerType = 'luenberger'; runSimulation()">Luenberger</button>
          <button :class="{ active: observerType === 'pll' }" @click="observerType = 'pll'; runSimulation()">PLL</button>
        </div>
      </div>
    </div>

    <canvas ref="canvasRef" class="sim-canvas"></canvas>

    <div class="sim-params">
      <div class="param-group">
        <label>观测器增益: <span class="param-val">{{ observerGain.toFixed(1) }}</span></label>
        <input type="range" v-model.number="observerGain" :min="0.5" :max="10" :step="0.1" @input="runSimulation" />
        <span class="param-range">0.5 ~ 10 (越高收敛越快, 噪声越大)</span>
      </div>
      <div class="param-group">
        <label>初始角度误差: <span class="param-val">{{ initAngleError }}&deg;</span></label>
        <input type="range" v-model.number="initAngleError" :min="5" :max="90" :step="1" @input="runSimulation" />
        <span class="param-range">5&deg; ~ 90&deg;</span>
      </div>
      <div class="param-group">
        <label>电机转速: <span class="param-val">{{ motorSpeed }} RPM</span></label>
        <input type="range" v-model.number="motorSpeed" :min="100" :max="6000" :step="50" @input="runSimulation" />
        <span class="param-range">100 ~ 6000 RPM</span>
      </div>
    </div>

    <div class="sim-info">
      <div class="info-block">
        <div class="info-block-title">观测器状态</div>
        <div class="info-row"><span class="info-label">类型:</span><span class="info-value highlight">{{ observerType.toUpperCase() }}</span></div>
        <div class="info-row"><span class="info-label">收敛状态:</span><span class="info-value" :style="{ color: converged ? '#10b981' : '#ef4444' }">{{ converged ? '已收敛' : '收敛中' }}</span></div>
        <div class="info-row"><span class="info-label">位置误差:</span><span class="info-value highlight">{{ (currentErr * 180 / Math.PI).toFixed(3) }}&deg;</span></div>
      </div>
      <div class="info-block">
        <div class="info-block-title">估算结果</div>
        <div class="info-row"><span class="info-label">θe 估算:</span><span class="info-value">{{ history.length > 0 ? ((history[history.length-1].thetaEst * 180 / Math.PI) % 360).toFixed(1) : '--' }}°</span></div>
        <div class="info-row"><span class="info-label">ωe 估算:</span><span class="info-value">{{ metrics.estOmega.toFixed(1) }} rad/s</span></div>
        <div class="info-row"><span class="info-label">收敛时间:</span><span class="info-value highlight">{{ metrics.convergeTime.toFixed(1) }} ms</span></div>
      </div>
      <div class="info-block">
        <div class="info-block-title">运行参数</div>
        <div class="info-row"><span class="info-label">转速:</span><span class="info-value">{{ motorSpeed }} RPM</span></div>
        <div class="info-row"><span class="info-label">ωe 实际:</span><span class="info-value">{{ ((motorSpeed / 60) * 2 * Math.PI).toFixed(1) }} rad/s</span></div>
        <div class="info-row"><span class="info-label">初始偏差:</span><span class="info-value">{{ initAngleError }}&deg;</span></div>
      </div>
    </div>

    <div class="explain-panel">
      <details open>
        <summary>SMO / Luenberger / PLL 观测器原理对比</summary>
        <div class="explain-content">
          <p><strong>SMO (滑模观测器)：</strong>利用不连续开关函数 sign() 生成滑模运动，迫使估算电流跟踪实际电流。滑模面 s = îs - is，当 s→0 时从等效控制中提取反电动势信息。优点：对参数不敏感、鲁棒性强；缺点：抖振需要低通滤波。</p>
          <p><strong>Luenberger 观测器：</strong>基于线性状态空间模型，通过极点配置设计反馈增益矩阵。估算误差 e = x - x̂ 的动态特性由 (A - LC) 的特征值决定。优点：平滑连续、易于分析；缺点：依赖模型精度。</p>
          <p><strong>PLL (锁相环)：</strong>鉴相器提取 q 轴反电动势误差 → PI 环路滤波器 → 积分得位置。结构简单，对谐波有天然滤波作用。常用于从 SMO 提取的反电动势中进一步精确估计位置。</p>
          <p><strong>收敛速度 vs 噪声权衡：</strong>增益越大收敛越快，但会放大测量噪声。低速时反电动势信噪比低，需降低带宽或用高频注入法。</p>
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
.mode-selector button { padding: 0.28rem 0.75rem; border: none; background: var(--content-bg); color: var(--text-secondary); cursor: pointer; font-size: 0.76rem; font-weight: 500; transition: all 0.15s; }
.mode-selector button.active { background: var(--primary-color); color: #fff; }
.mode-selector button:hover:not(.active) { background: var(--bg-color); }
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