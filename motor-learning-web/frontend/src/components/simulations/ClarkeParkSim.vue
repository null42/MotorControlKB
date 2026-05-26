<script setup lang="ts">
import { ref, computed, watch, onMounted, onUnmounted } from 'vue'

/**
 * Clarke & Park 坐标变换仿真
 * 可视化 FOC 完整坐标变换链: abc(三相) → αβ(静止) → dq(旋转) 框架
 * 
 * Canvas 920×520，三列布局：
 *   左列 (0~300): abc 三相坐标系 — 三个旋转相量，互差120°
 *   中列 (310~610): αβ 静止坐标系 — Vref 矢量旋转，αβ 投影
 *   右列 (620~910): dq 旋转坐标系 — dq 分量随 θe 对齐
 */

const canvasRef = ref<HTMLCanvasElement | null>(null)
let animFrameId = 0

// --- 仿真参数 ---
const thetaE = ref(30)          // 电角度 0-360°
const autoRotate = ref(false)
const rotateSpeed = ref(60)     // °/s
const vmag = ref(1.0)           // Vref 幅值 (标幺)

let lastTs = 0
let cumAngle = thetaE.value

const toRad = (deg: number) => deg * Math.PI / 180

// --- 三相电压计算 ---
const abcVoltages = computed(() => {
  const rad = toRad(thetaE.value)
  return {
    ua: vmag.value * Math.cos(rad),
    ub: vmag.value * Math.cos(rad - 2 * Math.PI / 3),
    uc: vmag.value * Math.cos(rad + 2 * Math.PI / 3),
  }
})

// --- Clarke 变换: abc → αβ (幅值不变型) ---
const alphaBeta = computed(() => {
  const { ua, ub, uc } = abcVoltages.value
  // 幅值不变型: uα = ua,  uβ = (ua + 2*ub) / √3
  const uAlpha = ua
  const uBeta = (ua + 2 * ub) / Math.sqrt(3)
  return { uAlpha, uBeta }
})

// --- Park 变换: αβ → dq ---
const dqValues = computed(() => {
  const { uAlpha, uBeta } = alphaBeta.value
  const cosT = Math.cos(toRad(thetaE.value))
  const sinT = Math.sin(toRad(thetaE.value))
  const ud =  uAlpha * cosT + uBeta * sinT
  const uq = -uAlpha * sinT + uBeta * cosT
  return { ud, uq }
})

// --- 监听 ---
watch(autoRotate, (v) => {
  if (v) { cumAngle = thetaE.value }
})
watch(thetaE, (nv, ov) => {
  if (!autoRotate.value || Math.abs(nv - ov) > 5) cumAngle = nv
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

  ctx.clearRect(0, 0, W, H)
  ctx.fillStyle = bgColor; ctx.fillRect(0, 0, W, H)

  // 列分隔线 (x=305, x=615)
  const SEP1 = 305, SEP2 = 615
  ctx.strokeStyle = borderColor; ctx.lineWidth = 1; ctx.setLineDash([6, 4])
  ctx.beginPath(); ctx.moveTo(SEP1, 10); ctx.lineTo(SEP1, H - 10); ctx.stroke()
  ctx.beginPath(); ctx.moveTo(SEP2, 10); ctx.lineTo(SEP2, H - 10); ctx.stroke()
  ctx.setLineDash([])

  const thetaRad = toRad(thetaE.value)
  const { ua, ub, uc } = abcVoltages.value
  const { uAlpha, uBeta } = alphaBeta.value
  const { ud, uq } = dqValues.value

  // ═══════════════════════════════════════════════════════
  //  左列: abc 三相坐标系 (0 ~ 300)
  // ═══════════════════════════════════════════════════════
  const L_CX = 150, L_CY = 195, L_R = 120
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('abc 三相坐标系 (静止)', L_CX, 26)

  // 圆圈
  ctx.strokeStyle = borderColor; ctx.lineWidth = 0.8
  ctx.beginPath(); ctx.arc(L_CX, L_CY, L_R, 0, 2 * Math.PI); ctx.stroke()

  // 三轴 (A:0°, B:120°, C:240°)
  const axisDefs = [
    { deg: 0,   color: '#ef4444', label: 'A' },
    { deg: 120, color: '#10b981', label: 'B' },
    { deg: 240, color: '#3b82f6', label: 'C' },
  ]

  axisDefs.forEach(ax => {
    const rad = toRad(ax.deg)
    // 绘制轴线
    ctx.strokeStyle = ax.color; ctx.lineWidth = 0.7; ctx.setLineDash([3, 4])
    ctx.beginPath()
    ctx.moveTo(L_CX, L_CY)
    ctx.lineTo(L_CX + L_R * 1.15 * Math.cos(rad), L_CY - L_R * 1.15 * Math.sin(rad))
    ctx.stroke()
    ctx.setLineDash([])
    // 标签
    ctx.fillStyle = ax.color; ctx.font = 'bold 13px sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
    ctx.fillText(ax.label, L_CX + (L_R + 18) * Math.cos(rad), L_CY - (L_R + 18) * Math.sin(rad))
  })
  ctx.textBaseline = 'alphabetic'

  // 旋转相量 (ua, ub, uc 投影到各自轴)
  const phaseData = [
    { val: ua, axisDeg: 0,   color: '#ef4444', name: 'ua' },
    { val: ub, axisDeg: 120, color: '#10b981', name: 'ub' },
    { val: uc, axisDeg: 240, color: '#3b82f6', name: 'uc' },
  ]

  phaseData.forEach(ph => {
    const rad = toRad(ph.axisDeg)
    const len = ph.val * L_R
    ctx.strokeStyle = ph.color; ctx.lineWidth = 2.2
    ctx.beginPath()
    ctx.moveTo(L_CX, L_CY)
    ctx.lineTo(L_CX + len * Math.cos(rad), L_CY - len * Math.sin(rad))
    ctx.stroke()
    // 端点
    ctx.fillStyle = ph.color
    ctx.beginPath(); ctx.arc(L_CX + len * Math.cos(rad), L_CY - len * Math.sin(rad), 4.5, 0, 2 * Math.PI); ctx.fill()
    // 箭头
    ctx.fillStyle = ph.color; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
    const lblR = len + 15
    ctx.fillText(ph.name, L_CX + lblR * Math.cos(rad), L_CY - lblR * Math.sin(rad) + 4)
  })

  // 下方三相正弦波形 (小图)
  const waveTop = L_CY + L_R + 32, waveH = 72, waveW = 280, waveL = 10
  const waveMid = waveTop + waveH / 2
  ctx.fillStyle = textColor; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('三相电压波形', L_CX, waveTop - 6)

  ctx.strokeStyle = textSec; ctx.lineWidth = 0.5
  ctx.beginPath(); ctx.moveTo(waveL, waveMid); ctx.lineTo(waveL + waveW, waveMid); ctx.stroke()

  const N_WAVE = 360
  phaseData.forEach(ph => {
    ctx.strokeStyle = ph.color; ctx.lineWidth = 1.3
    ctx.beginPath()
    for (let i = 0; i <= N_WAVE; i++) {
      const deg = (i / N_WAVE) * 360
      const val = vmag.value * Math.cos(toRad(deg - ph.axisDeg))
      const x = waveL + (i / N_WAVE) * waveW
      const y = waveMid - val * waveH * 0.5
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
    }
    ctx.stroke()
  })
  // 游标
  const cursorFrac = ((thetaE.value % 360) + 360) % 360 / 360
  const cX = waveL + cursorFrac * waveW
  ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 1.3; ctx.setLineDash([3, 3])
  ctx.beginPath(); ctx.moveTo(cX, waveTop); ctx.lineTo(cX, waveTop + waveH); ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#f59e0b'; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText(`${thetaE.value.toFixed(0)}°`, cX, waveTop + waveH + 13)

  // ═══════════════════════════════════════════════════════
  //  中列: αβ 静止坐标系 (310 ~ 610)
  // ═══════════════════════════════════════════════════════
  const M_CX = 460, M_CY = 195, M_R = 120
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('αβ 静止坐标系 (Clarke)', M_CX, 26)

  // α-β 轴
  ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 1; ctx.setLineDash([3, 4])
  ctx.beginPath(); ctx.moveTo(M_CX - M_R * 1.2, M_CY); ctx.lineTo(M_CX + M_R * 1.2, M_CY); ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#f59e0b'; ctx.font = 'bold 14px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('α', M_CX + M_R * 1.2 + 8, M_CY + 5)

  ctx.strokeStyle = '#8b5cf6'; ctx.lineWidth = 1; ctx.setLineDash([3, 4])
  ctx.beginPath(); ctx.moveTo(M_CX, M_CY - M_R * 1.2); ctx.lineTo(M_CX, M_CY + M_R * 1.2); ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#8b5cf6'; ctx.font = 'bold 14px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('β', M_CX + 5, M_CY + M_R * 1.2 + 14)

  // 圆圈
  ctx.strokeStyle = borderColor; ctx.lineWidth = 0.8
  ctx.beginPath(); ctx.arc(M_CX, M_CY, M_R, 0, 2 * Math.PI); ctx.stroke()

  const scaleM = M_R / vmag.value

  // Vref 旋转矢量
  ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 2.5
  ctx.beginPath()
  ctx.moveTo(M_CX, M_CY)
  ctx.lineTo(M_CX + uAlpha * scaleM, M_CY - uBeta * scaleM)
  ctx.stroke()

  // Vref 端点
  ctx.fillStyle = '#ef4444'
  ctx.beginPath(); ctx.arc(M_CX + uAlpha * scaleM, M_CY - uBeta * scaleM, 5, 0, 2 * Math.PI); ctx.fill()
  ctx.fillStyle = '#ef4444'; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText('Vref', M_CX + uAlpha * scaleM + 10, M_CY - uBeta * scaleM + 4)

  // α 投影虚线
  ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 1; ctx.setLineDash([3, 4])
  ctx.beginPath()
  ctx.moveTo(M_CX + uAlpha * scaleM, M_CY - uBeta * scaleM)
  ctx.lineTo(M_CX + uAlpha * scaleM, M_CY)
  ctx.stroke()
  ctx.fillStyle = '#f59e0b'; ctx.beginPath(); ctx.arc(M_CX + uAlpha * scaleM, M_CY, 3.5, 0, 2 * Math.PI); ctx.fill()
  ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText(`uα=${uAlpha.toFixed(2)}`, M_CX + uAlpha * scaleM, M_CY + 16)

  // β 投影虚线
  ctx.strokeStyle = '#8b5cf6'; ctx.lineWidth = 1; ctx.setLineDash([3, 4])
  ctx.beginPath()
  ctx.moveTo(M_CX + uAlpha * scaleM, M_CY - uBeta * scaleM)
  ctx.lineTo(M_CX, M_CY - uBeta * scaleM)
  ctx.stroke()
  ctx.fillStyle = '#8b5cf6'; ctx.beginPath(); ctx.arc(M_CX, M_CY - uBeta * scaleM, 3.5, 0, 2 * Math.PI); ctx.fill()
  ctx.fillText(`uβ=${uBeta.toFixed(2)}`, M_CX - 40, M_CY - uBeta * scaleM)
  ctx.setLineDash([])

  // Clarke 变换公式
  ctx.fillStyle = textSec; ctx.font = '9px monospace'; ctx.textAlign = 'center'
  ctx.fillText('uα = ua     (幅值不变型)', M_CX, M_CY + M_R + 32)
  ctx.fillText('uβ = (ua + 2ub) / √3', M_CX, M_CY + M_R + 46)

  // ═══════════════════════════════════════════════════════
  //  右列: dq 旋转坐标系 (620 ~ 910)
  // ═══════════════════════════════════════════════════════
  const R_CX = 765, R_CY = 195, R_R = 120
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('dq 旋转坐标系 (Park)', R_CX, 26)

  // 旋转 d-q 轴
  const cosT = Math.cos(thetaRad), sinT = Math.sin(thetaRad)

  ctx.strokeStyle = '#ec4899'; ctx.lineWidth = 1.5
  ctx.beginPath()
  ctx.moveTo(R_CX - R_R * 1.25 * cosT, R_CY + R_R * 1.25 * sinT)
  ctx.lineTo(R_CX + R_R * 1.25 * cosT, R_CY - R_R * 1.25 * sinT)
  ctx.stroke()
  ctx.fillStyle = '#ec4899'; ctx.font = 'bold 14px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('d', R_CX + (R_R * 1.25 + 10) * cosT, R_CY - (R_R * 1.25 + 10) * sinT + 4)

  ctx.strokeStyle = '#06b6d4'; ctx.lineWidth = 1.5
  ctx.beginPath()
  ctx.moveTo(R_CX + R_R * 1.25 * sinT, R_CY + R_R * 1.25 * cosT)
  ctx.lineTo(R_CX - R_R * 1.25 * sinT, R_CY - R_R * 1.25 * cosT)
  ctx.stroke()
  ctx.fillStyle = '#06b6d4'; ctx.font = 'bold 14px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('q', R_CX - (R_R * 1.25 + 10) * sinT - 4, R_CY - (R_R * 1.25 + 10) * cosT + 4)

  // 静止 α-β 参考轴 (淡色)
  ctx.strokeStyle = textSec; ctx.lineWidth = 0.5; ctx.setLineDash([2, 4]); ctx.globalAlpha = 0.35
  ctx.beginPath(); ctx.moveTo(R_CX - R_R * 1.15, R_CY); ctx.lineTo(R_CX + R_R * 1.15, R_CY); ctx.stroke()
  ctx.beginPath(); ctx.moveTo(R_CX, R_CY - R_R * 1.15); ctx.lineTo(R_CX, R_CY + R_R * 1.15); ctx.stroke()
  ctx.globalAlpha = 1; ctx.setLineDash([])

  // 圆圈
  ctx.strokeStyle = borderColor; ctx.lineWidth = 0.8
  ctx.beginPath(); ctx.arc(R_CX, R_CY, R_R, 0, 2 * Math.PI); ctx.stroke()

  const scaleR = R_R / vmag.value

  // ud 分量 (沿 d 轴)
  const dLen = ud * scaleR
  ctx.strokeStyle = '#ec4899'; ctx.lineWidth = 2; ctx.setLineDash([3, 3])
  ctx.beginPath()
  ctx.moveTo(R_CX, R_CY)
  ctx.lineTo(R_CX + dLen * cosT, R_CY - dLen * sinT)
  ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#ec4899'; ctx.beginPath(); ctx.arc(R_CX + dLen * cosT, R_CY - dLen * sinT, 4, 0, 2 * Math.PI); ctx.fill()
  ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText(`ud=${ud.toFixed(2)}`, R_CX + (dLen + 12) * cosT + 4, R_CY - (dLen + 12) * sinT + 4)

  // uq 分量 (沿 q 轴)
  const qLen = uq * scaleR
  ctx.strokeStyle = '#06b6d4'; ctx.lineWidth = 2; ctx.setLineDash([3, 3])
  ctx.beginPath()
  ctx.moveTo(R_CX, R_CY)
  ctx.lineTo(R_CX - qLen * sinT, R_CY - qLen * cosT)
  ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#06b6d4'; ctx.beginPath(); ctx.arc(R_CX - qLen * sinT, R_CY - qLen * cosT, 4, 0, 2 * Math.PI); ctx.fill()
  ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText(`uq=${uq.toFixed(2)}`, R_CX - (qLen + 12) * sinT + 4, R_CY - (qLen + 12) * cosT + 4)

  // 旋转角度标注
  ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 1.2
  ctx.beginPath(); ctx.arc(R_CX, R_CY, 28, toRad(-90), toRad(-90) + thetaRad, false); ctx.stroke()
  ctx.fillStyle = '#f59e0b'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText(`θe=${thetaE.value.toFixed(0)}°`, R_CX + 50 * Math.cos(thetaRad / 2 - toRad(90)), R_CY - 50 * Math.sin(thetaRad / 2 - toRad(90)))

  // 原点
  ctx.fillStyle = textColor; ctx.beginPath(); ctx.arc(R_CX, R_CY, 3.5, 0, 2 * Math.PI); ctx.fill()

  // Park 变换公式
  ctx.fillStyle = textSec; ctx.font = '9px monospace'; ctx.textAlign = 'center'
  ctx.fillText('ud =  uα·cosθ + uβ·sinθ', R_CX, R_CY + R_R + 32)
  ctx.fillText('uq = -uα·sinθ + uβ·cosθ', R_CX, R_CY + R_R + 46)

  // ═══════════════════════════════════════════════════════
  //  底部: 三个框架之间的箭头指示
  // ═══════════════════════════════════════════════════════
  const arrowY = H - 22
  ctx.fillStyle = primaryColor; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('Clarke 变换', (SEP1 + 0) / 2 + 40, arrowY)
  ctx.fillText('Park 变换', (SEP2 + SEP1) / 2 + 40, arrowY)
  ctx.font = '11px sans-serif'
  ctx.fillText('abc → αβ', (SEP1 + 0) / 2 + 40, arrowY + 14)
  ctx.fillText('αβ → dq', (SEP2 + SEP1) / 2 + 40, arrowY + 14)

  // 小箭头
  ctx.strokeStyle = primaryColor; ctx.lineWidth = 1.2
  const arrL = SEP1 - 45, arrR = SEP1 - 12
  ctx.beginPath(); ctx.moveTo(arrL, arrowY + 2); ctx.lineTo(arrR, arrowY + 2); ctx.stroke()
  ctx.fillStyle = primaryColor
  ctx.beginPath(); ctx.moveTo(arrR, arrowY + 2); ctx.lineTo(arrR - 6, arrowY - 2); ctx.lineTo(arrR - 6, arrowY + 6); ctx.closePath(); ctx.fill()
  const arrL2 = SEP2 - 45, arrR2 = SEP2 - 12
  ctx.beginPath(); ctx.moveTo(arrL2, arrowY + 2); ctx.lineTo(arrR2, arrowY + 2); ctx.stroke()
  ctx.beginPath(); ctx.moveTo(arrR2, arrowY + 2); ctx.lineTo(arrR2 - 6, arrowY - 2); ctx.lineTo(arrR2 - 6, arrowY + 6); ctx.closePath(); ctx.fill()
}

// ═════════════════════════ 动画循环 ═════════════════════════
const animate = (ts: number) => {
  if (autoRotate.value && lastTs > 0) {
    const dt = Math.min((ts - lastTs) / 1000, 0.1)
    cumAngle += rotateSpeed.value * dt
    thetaE.value = ((cumAngle % 360) + 360) % 360
  }
  lastTs = ts
  try { draw() } catch { /* keep loop alive */ }
  animFrameId = requestAnimationFrame(animate)
}

onMounted(() => { animFrameId = requestAnimationFrame(animate) })
onUnmounted(() => cancelAnimationFrame(animFrameId))
</script>

<template>
  <div class="sim-container">
    <div class="sim-header">
      <h4>Clarke &amp; Park 坐标变换仿真 — abc → αβ → dq</h4>
      <div class="header-controls">
        <button class="auto-btn" :class="{ active: autoRotate }" @click="autoRotate = !autoRotate">
          {{ autoRotate ? '&#9208; 暂停旋转' : '&#9654; 自动旋转' }}
        </button>
      </div>
    </div>

    <canvas ref="canvasRef" class="sim-canvas"></canvas>

    <div class="sim-params">
      <div class="param-group">
        <label>电气角度 &#952;e: <span class="param-val">{{ thetaE.toFixed(1) }}&deg;</span></label>
        <input type="range" v-model.number="thetaE" :min="0" :max="360" :step="0.5" />
        <span class="param-range">0&deg; ~ 360&deg;</span>
      </div>
      <div class="param-group">
        <label>Vref 幅值: <span class="param-val">{{ vmag.toFixed(2) }}</span></label>
        <input type="range" v-model.number="vmag" :min="0.1" :max="1.5" :step="0.05" />
        <span class="param-range">0.1 ~ 1.5 pu</span>
      </div>
      <div class="param-group" v-if="autoRotate">
        <label>旋转速度: <span class="param-val">{{ rotateSpeed }}&deg;/s</span></label>
        <input type="range" v-model.number="rotateSpeed" :min="1" :max="360" :step="1" />
        <span class="param-range">1 ~ 360 &deg;/s</span>
      </div>
    </div>

    <div class="sim-info">
      <div class="info-block">
        <div class="info-block-title">三相值 (abc)</div>
        <div class="info-row"><span class="info-label" style="color:#ef4444">ua:</span><span class="info-value">{{ abcVoltages.ua.toFixed(3) }}</span></div>
        <div class="info-row"><span class="info-label" style="color:#10b981">ub:</span><span class="info-value">{{ abcVoltages.ub.toFixed(3) }}</span></div>
        <div class="info-row"><span class="info-label" style="color:#3b82f6">uc:</span><span class="info-value">{{ abcVoltages.uc.toFixed(3) }}</span></div>
        <div class="info-row"><span class="info-label">&#8721;:</span><span class="info-value highlight">{{ (abcVoltages.ua + abcVoltages.ub + abcVoltages.uc).toFixed(3) }}</span><span class="info-note">(&#8776;0)</span></div>
      </div>
      <div class="info-block">
        <div class="info-block-title">静止坐标系 (&#945;&#946;)</div>
        <div class="info-row"><span class="info-label" style="color:#f59e0b">u&#945;:</span><span class="info-value">{{ alphaBeta.uAlpha.toFixed(3) }}</span></div>
        <div class="info-row"><span class="info-label" style="color:#8b5cf6">u&#946;:</span><span class="info-value">{{ alphaBeta.uBeta.toFixed(3) }}</span></div>
      </div>
      <div class="info-block">
        <div class="info-block-title">旋转坐标系 (dq)</div>
        <div class="info-row"><span class="info-label" style="color:#ec4899">ud:</span><span class="info-value highlight">{{ dqValues.ud.toFixed(3) }}</span><span class="info-note">&#8776;{{ vmag.toFixed(2) }}</span></div>
        <div class="info-row"><span class="info-label" style="color:#06b6d4">uq:</span><span class="info-value">{{ dqValues.uq.toFixed(3) }}</span><span class="info-note">&#8776;0</span></div>
        <div class="info-row"><span class="info-label">&#952;e:</span><span class="info-value highlight">{{ thetaE.toFixed(1) }}&deg;</span></div>
      </div>
    </div>

    <div class="explain-panel">
      <details open>
        <summary>Clarke &amp; Park 坐标变换原理</summary>
        <div class="explain-content">
          <p><strong>Clarke 变换 (abc &rarr; &#945;&#946;)：</strong>将三相 120&deg; 互差的正弦量变换为两相正交的静止坐标系。幅值不变型公式：u&#945; = ua, u&#946; = (ua + 2ub) / &#8730;3。三相平衡时 &#945;&#946; 合成的空间矢量 Vref 以同步角速度旋转，幅值等于相电压幅值。</p>
          <p><strong>Park 变换 (&#945;&#946; &rarr; dq)：</strong>将静止 &#945;&#946; 坐标系旋转电角度 &#952;e，得到与转子磁场同步旋转的 dq 坐标系。d 轴对准转子磁链方向，q 轴与之正交。变换矩阵：ud = u&#945;·cos&#952; + u&#946;·sin&#952;, uq = -u&#945;·sin&#952; + u&#946;·cos&#952;。</p>
          <p><strong>FOC 核心思想：</strong>通过 Clarke + Park 变换，将三相交流量转换为 dq 直流分量。在 dq 坐标系下用 PI 控制器分别调节励磁电流 (id) 和转矩电流 (iq)，再通过 Park 逆变换 + SVPWM 输出三相电压驱动电机。</p>
          <p><strong>颜色编码：</strong>红=Phase A / Vref, 绿=Phase B, 蓝=Phase C, 橙=&#945;轴, 紫=&#946;轴, 粉=d轴, 青=q轴</p>
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