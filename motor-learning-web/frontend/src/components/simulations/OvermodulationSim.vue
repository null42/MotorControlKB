<script setup lang="ts">
import { ref, computed, watch, onMounted, onUnmounted } from 'vue'

/**
 * 过调制与六阶梯波仿真
 * 展示三个调制区域：线性区(m≤1)、过调制I区(1<m≤1.155)、六阶梯波
 *
 * Canvas 920×600 两列布局:
 *   左列 (0~440): αβ六边形 + 内切圆 + 外接圆 + Vref轨迹(含限幅)
 *   右列 (450~910): 相电压波形演化 (纯正弦 → 马鞍波 → 限幅马鞍 → 六阶梯波)
 */

const canvasRef = ref<HTMLCanvasElement | null>(null)
let animFrameId = 0

const modIndex = ref(0.8)         // 调制比 m: 0.3~1.155
const showAngle = ref(30)         // 当前显示的Vref角度
const autoRotate = ref(false)
const rotateSpeed = ref(30)       // °/s
const showIdeal = ref(true)       // 显示理想(未限幅) Vref

let lastTs = 0
let cumAngle = showAngle.value

const toRad = (deg: number) => deg * Math.PI / 180

// 六边形半径归一化为1，顶点在0°,60°,120°...
const HEX_R = 1
// 内切圆半径
const INSC_R = HEX_R * Math.sqrt(3) / 2  // ≈ 0.866
// 外接圆半径 = HEX_R = 1

// 调制区域判定
const zoneInfo = computed(() => {
  if (modIndex.value <= 1.0) return { name: '线性调制区', short: '线性区', color: '#10b981' }
  if (modIndex.value < 1.155) return { name: '过调制 I 区', short: '过调制I区', color: '#f59e0b' }
  return { name: '六阶梯波 (方波)', short: '六阶梯波', color: '#ef4444' }
})

// 六边形顶点
const hexVertices = computed(() => {
  const verts: { x: number; y: number }[] = []
  for (let i = 0; i < 6; i++) {
    const angle = toRad(i * 60)
    verts.push({ x: HEX_R * Math.cos(angle), y: HEX_R * Math.sin(angle) })
  }
  return verts
})

// 六边形边约束 (每条边的法向量和截距)
// 六边形可以表示为6个半平面的交集
interface Edge { nx: number; ny: number; d: number }
const hexEdges: Edge[] = [
  { nx: 0,  ny: 1,  d: INSC_R },            // 上边
  { nx: 0,  ny: -1, d: INSC_R },            // 下边
  { nx: Math.sqrt(3) / 2, ny: 0.5,  d: INSC_R },  // 右上边
  { nx: -Math.sqrt(3) / 2, ny: -0.5, d: INSC_R },  // 左下边
  { nx: Math.sqrt(3) / 2, ny: -0.5, d: INSC_R },  // 右下边
  { nx: -Math.sqrt(3) / 2, ny: 0.5,  d: INSC_R },  // 左上边
]

// Vref 理想幅值 (在六边形坐标系中)
const vrefMag = computed(() => modIndex.value * INSC_R)

// 将Vref限幅到六边形内
function clipToHex(px: number, py: number): { x: number; y: number; clipped: boolean } {
  let cx = px, cy = py
  let clipped = false

  for (const edge of hexEdges) {
    const dist = edge.nx * cx + edge.ny * cy
    if (dist > edge.d) {
      // 投影到边上
      const excess = dist - edge.d
      cx -= excess * edge.nx
      cy -= excess * edge.ny
      clipped = true
    }
  }
  return { x: cx, y: cy, clipped }
}

// 当前Vref的理想值和限幅值
const vrefCurrent = computed(() => {
  const angle = toRad(showAngle.value % 360)
  const mag = vrefMag.value
  const ix = mag * Math.cos(angle)
  const iy = mag * Math.sin(angle)
  const clipped = clipToHex(ix, iy)
  return { idealX: ix, idealY: iy, clipX: clipped.x, clipY: clipped.y, isClipped: clipped.clipped }
})

// 生成某一m值下的相电压波形 (A相)
function computePhaseAWaveform(m: number, nPts: number): number[] {
  const data: number[] = []
  for (let i = 0; i <= nPts; i++) {
    const angle = (i / nPts) * 360
    const rad = toRad(angle)
    // 三相参考
    const va = m * Math.sin(rad)
    const vb = m * Math.sin(rad - 2 * Math.PI / 3)
    const vc = m * Math.sin(rad + 2 * Math.PI / 3)
    // 零序注入
    const vmin = Math.min(va, vb, vc)
    const vmax = Math.max(va, vb, vc)
    const vo = -(vmax + vmin) / 2
    let vaMod = va + vo
    // 限幅到 [-1, 1] (归一化到六边形边界)
    const limit = 1.0
    vaMod = Math.max(-limit, Math.min(limit, vaMod))
    data.push(vaMod)
  }
  return data
}

// 生成Vref轨迹点 (用于绘制旋转轨迹)
function computeTrajectory(m: number, nPts: number): { x: number; y: number; clipped: boolean }[] {
  const pts: { x: number; y: number; clipped: boolean }[] = []
  const mag = m * INSC_R
  for (let i = 0; i <= nPts; i++) {
    const angle = toRad((i / nPts) * 360)
    const ix = mag * Math.cos(angle)
    const iy = mag * Math.sin(angle)
    const clipped = clipToHex(ix, iy)
    pts.push({ x: clipped.x, y: clipped.y, clipped: clipped.clipped })
  }
  return pts
}

// 预计算各m值的波形
const mLevels = [0.5, 0.8, 1.0, 1.1]
const waveforms = computed(() => {
  const result: { m: number; data: number[]; color: string }[] = []
  for (const m of mLevels) {
    let color = '#94a3b8'
    if (m === 0.5) color = '#3b82f6'
    else if (m === 1.0) color = '#10b981'
    else if (m === 1.1) color = '#f59e0b'
    result.push({ m, data: computePhaseAWaveform(m, 720), color })
  }
  // 添加当前m值的波形
  if (!mLevels.includes(modIndex.value)) {
    result.push({ m: modIndex.value, data: computePhaseAWaveform(modIndex.value, 720), color: '#ef4444' })
  }
  return result
})

watch(autoRotate, (v) => {
  if (v) cumAngle = showAngle.value
})
watch(showAngle, (nv, ov) => {
  if (!autoRotate.value || Math.abs(nv - ov) > 5) cumAngle = nv
})

// ═════════════════════════ 绘图 ═════════════════════════
const draw = () => {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  const W = 920, H = 600
  canvas.width = W; canvas.height = H

  const style = getComputedStyle(document.documentElement)
  const bgColor   = style.getPropertyValue('--bg-color').trim() || '#f8fafc'
  const textColor = style.getPropertyValue('--text-color').trim() || '#1e293b'
  const borderColor = style.getPropertyValue('--border-color').trim() || '#e2e8f0'
  const primaryColor = style.getPropertyValue('--primary-color').trim() || '#3b82f6'
  const textSec = style.getPropertyValue('--text-secondary').trim() || '#64748b'

  ctx.clearRect(0, 0, W, H)
  ctx.fillStyle = bgColor; ctx.fillRect(0, 0, W, H)

  // 列分隔线
  const SEP = 450
  ctx.strokeStyle = borderColor; ctx.lineWidth = 1; ctx.setLineDash([6, 4])
  ctx.beginPath(); ctx.moveTo(SEP, 10); ctx.lineTo(SEP, H - 10); ctx.stroke()
  ctx.setLineDash([])

  // ═══════════════════════════════════════════════════════
  //  左列: αβ 六边形 + 轨迹 (0 ~ 440)
  // ═══════════════════════════════════════════════════════
  const L_CX = 220, L_CY = 260, L_SCALE = 170

  const toCX = (mx: number) => L_CX + mx * L_SCALE
  const toCY = (my: number) => L_CY - my * L_SCALE

  // 标题
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('αβ 电压六边形 — 过调制轨迹', L_CX, 24)

  // 内切圆 (m=1, 绿色虚线)
  ctx.strokeStyle = '#10b981'; ctx.lineWidth = 1.2; ctx.setLineDash([5, 4])
  ctx.beginPath(); ctx.arc(toCX(0), toCY(0), INSC_R * L_SCALE, 0, 2 * Math.PI); ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#10b981'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText('内切圆 m=1.000', toCX(INSC_R * 0.55), toCY(-INSC_R * 0.55))

  // 外接圆 (m=1.155, 橙色虚线)
  ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 1; ctx.setLineDash([4, 3])
  ctx.beginPath(); ctx.arc(toCX(0), toCY(0), HEX_R * L_SCALE, 0, 2 * Math.PI); ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#f59e0b'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText('外接圆 m=1.155', toCX(HEX_R * 0.65), toCY(-HEX_R * 0.5))

  // 六边形
  ctx.strokeStyle = primaryColor; ctx.lineWidth = 2
  ctx.beginPath()
  const verts = hexVertices.value
  ctx.moveTo(toCX(verts[0].x), toCY(verts[0].y))
  for (let i = 1; i < 6; i++) ctx.lineTo(toCX(verts[i].x), toCY(verts[i].y))
  ctx.closePath(); ctx.stroke()

  // 顶点标注
  const vecNames = ['V4(100)', 'V6(110)', 'V2(010)', 'V3(011)', 'V1(001)', 'V5(101)']
  verts.forEach((v, i) => {
    ctx.fillStyle = primaryColor; ctx.beginPath(); ctx.arc(toCX(v.x), toCY(v.y), 4, 0, 2 * Math.PI); ctx.fill()
    ctx.fillStyle = textColor; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'center'
    const lblR = L_SCALE * 1.1
    const angle = toRad(i * 60)
    ctx.fillText(vecNames[i], toCX(Math.cos(angle) * 1.08), toCY(Math.sin(angle) * 1.08) + 4)
  })

  // α-β 轴
  const axisLen = HEX_R * 1.3
  ctx.strokeStyle = textSec; ctx.lineWidth = 0.6; ctx.setLineDash([3, 4])
  ctx.beginPath(); ctx.moveTo(toCX(-axisLen), toCY(0)); ctx.lineTo(toCX(axisLen), toCY(0)); ctx.stroke()
  ctx.beginPath(); ctx.moveTo(toCX(0), toCY(-axisLen)); ctx.lineTo(toCX(0), toCY(axisLen)); ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = textSec; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('α', toCX(axisLen + 0.06), toCY(0) + 5)
  ctx.fillText('β', toCX(0) + 5, toCY(axisLen + 0.06) + 4)

  // Vref 轨迹
  const trajPts = computeTrajectory(modIndex.value, 360)
  ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 1.8
  ctx.beginPath()
  for (let i = 0; i <= trajPts.length - 1; i++) {
    const pt = trajPts[i]
    const x = toCX(pt.x), y = toCY(pt.y)
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
  }
  ctx.stroke()

  // 被限幅的弧段用更粗/不同颜色标记
  let inClip = false
  ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 2.5
  for (let i = 0; i < trajPts.length; i++) {
    const pt = trajPts[i]
    if (pt.clipped && !inClip) {
      ctx.beginPath()
      ctx.moveTo(toCX(pt.x), toCY(pt.y))
      inClip = true
    } else if (!pt.clipped && inClip) {
      const prevPt = trajPts[i - 1]
      ctx.lineTo(toCX(prevPt.x), toCY(prevPt.y))
      ctx.stroke()
      inClip = false
    } else if (pt.clipped && inClip) {
      ctx.lineTo(toCX(pt.x), toCY(pt.y))
    }
  }
  if (inClip) {
    ctx.lineTo(toCX(trajPts[trajPts.length - 1].x), toCY(trajPts[trajPts.length - 1].y))
    ctx.stroke()
  }

  // 当前Vref矢量
  const vc = vrefCurrent.value
  const clipX = toCX(vc.clipX), clipY = toCY(vc.clipY)

  // 理想Vref (如果被限幅则用虚线)
  if (showIdeal.value && vc.isClipped) {
    const idealX = toCX(vc.idealX), idealY = toCY(vc.idealY)
    ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 1.2; ctx.setLineDash([3, 4])
    ctx.beginPath(); ctx.moveTo(toCX(0), toCY(0)); ctx.lineTo(idealX, idealY); ctx.stroke()
    ctx.setLineDash([])
    ctx.fillStyle = 'rgba(239,68,68,0.4)'; ctx.beginPath(); ctx.arc(idealX, idealY, 5, 0, 2 * Math.PI); ctx.fill()
    ctx.fillStyle = '#ef4444'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'left'
    ctx.fillText('理想(越界)', idealX + 8, idealY - 8)
  }

  // 限幅后Vref
  ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 2.5
  ctx.beginPath(); ctx.moveTo(toCX(0), toCY(0)); ctx.lineTo(clipX, clipY); ctx.stroke()
  ctx.fillStyle = '#f59e0b'; ctx.beginPath(); ctx.arc(clipX, clipY, 5.5, 0, 2 * Math.PI); ctx.fill()
  ctx.strokeStyle = '#fff'; ctx.lineWidth = 1
  ctx.beginPath(); ctx.arc(clipX, clipY, 5.5, 0, 2 * Math.PI); ctx.stroke()
  ctx.fillStyle = '#f59e0b'; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText('Vref', clipX + 10, clipY - 8)

  // 调制区域状态标签
  const zone = zoneInfo.value
  ctx.fillStyle = zone.color; ctx.font = 'bold 14px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText(zone.name, L_CX, toCY(-axisLen - 0.05) + 18)

  // ═══════════════════════════════════════════════════════
  //  右列: 相电压波形演化 (450 ~ 910)
  // ═══════════════════════════════════════════════════════
  const R_L = SEP + 10, R_R = W - 10, R_W = R_R - R_L
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('A 相调制波演化 (不同 m 值对比)', (SEP + W) / 2, 24)

  // 四个固定m值的参考波形
  const waveDefs = [
    { m: 0.5,  color: '#3b82f6', label: 'm=0.5 纯正弦' },
    { m: 0.8,  color: '#8b5cf6', label: 'm=0.8' },
    { m: 1.0,  color: '#10b981', label: 'm=1.0 马鞍波' },
    { m: 1.1,  color: '#f59e0b', label: 'm=1.1 限幅' },
  ]

  const N_PTS = 720
  const waveTop = 38
  const numWaves = 4
  const waveH = 118
  const waveGap = 8

  waveDefs.forEach((wd, wi) => {
    const top = waveTop + wi * (waveH + waveGap)
    const midY = top + waveH / 2
    const scaleY = waveH * 0.46

    // 背景
    ctx.fillStyle = wi === 3 ? 'rgba(245,158,11,0.05)' : 'rgba(0,0,0,0.02)'
    ctx.fillRect(R_L, top + 16, R_W, waveH - 10)

    // 参考线
    ctx.strokeStyle = textSec; ctx.lineWidth = 0.4
    ctx.beginPath(); ctx.moveTo(R_L, midY); ctx.lineTo(R_R, midY); ctx.stroke()

    // 标签
    ctx.fillStyle = wd.color; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'left'
    ctx.fillText(wd.label, R_L + 4, top + 6)

    const data = computePhaseAWaveform(wd.m, N_PTS)
    ctx.strokeStyle = wd.color; ctx.lineWidth = wi === 3 ? 2 : 1.3
    ctx.beginPath()
    for (let i = 0; i <= N_PTS; i++) {
      const x = R_L + (i / N_PTS) * R_W
      const y = midY - data[i] * scaleY
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
    }
    ctx.stroke()

    // m=1.0时标注马鞍特征
    if (wd.m === 1.0) {
      ctx.fillStyle = '#10b981'; ctx.font = '9px sans-serif'; ctx.textAlign = 'left'
      ctx.fillText('← 鞍部 (零序注入)', R_L + R_W * 0.24, midY - 0.85 * scaleY - 2)
    }
  })

  // 当前m值的波形 (高亮, 叠加在最下方大图)
  const curWaveTop = waveTop + numWaves * (waveH + waveGap) + 4
  const curWaveH = 110
  const curMidY = curWaveTop + curWaveH / 2
  const curScaleY = curWaveH * 0.46

  ctx.fillStyle = 'rgba(59,130,246,0.06)'; ctx.fillRect(R_L, curWaveTop, R_W, curWaveH - 4)

  ctx.strokeStyle = textSec; ctx.lineWidth = 0.4
  ctx.beginPath(); ctx.moveTo(R_L, curMidY); ctx.lineTo(R_R, curMidY); ctx.stroke()

  ctx.fillStyle = zone.value.color; ctx.font = 'bold 13px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText(`当前 m=${modIndex.value.toFixed(3)} — ${zone.value.name}`, R_L + 4, curWaveTop + 2)

  const curData = computePhaseAWaveform(modIndex.value, N_PTS)
  ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 2.2
  ctx.beginPath()
  for (let i = 0; i <= N_PTS; i++) {
    const x = R_L + (i / N_PTS) * R_W
    const y = curMidY - curData[i] * curScaleY
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
  }
  ctx.stroke()

  // ±1 边界线
  ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 0.6; ctx.setLineDash([2, 4])
  ctx.beginPath(); ctx.moveTo(R_L, curMidY - curScaleY); ctx.lineTo(R_R, curMidY - curScaleY); ctx.stroke()
  ctx.beginPath(); ctx.moveTo(R_L, curMidY + curScaleY); ctx.lineTo(R_R, curMidY + curScaleY); ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#ef4444'; ctx.font = '9px sans-serif'; ctx.textAlign = 'right'
  ctx.fillText('+Vdc/2', R_R, curMidY - curScaleY + 10)
  ctx.fillText('-Vdc/2', R_R, curMidY + curScaleY - 2)

  // 当前角度游标
  const cursorFrac = ((showAngle.value % 360) + 360) % 360 / 360
  const cursorX = R_L + cursorFrac * R_W
  ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 1.3; ctx.setLineDash([3, 3])
  ctx.beginPath(); ctx.moveTo(cursorX, curWaveTop - 6); ctx.lineTo(cursorX, curWaveTop + curWaveH + 4); ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#f59e0b'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText(`θ=${showAngle.value.toFixed(0)}°`, cursorX, curWaveTop - 9)

  // 底部区域状态指示
  const statY = curWaveTop + curWaveH + 16
  const zoneColors: Record<string, string> = { '线性区': '#10b981', '过调制I区': '#f59e0b', '六阶梯波': '#ef4444' }
  const zoneName = zone.value.short

  // 绘制进度条式区域指示器
  const barL = R_L + 40, barR = R_R - 40, barW = barR - barL
  const barY = statY + 6, barH = 12

  ctx.fillStyle = textSec; ctx.font = '11px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('调制区域:', (SEP + W) / 2, statY - 6)

  // 三色进度条
  const frac1 = 1.0 / 1.155  // 线性区占的比例
  const frac2 = (1.155 - 1.0) / 1.155  // 过调制区

  ctx.fillStyle = 'rgba(16,185,129,0.5)'; ctx.fillRect(barL, barY, barW * frac1, barH)
  ctx.fillStyle = 'rgba(245,158,11,0.5)'; ctx.fillRect(barL + barW * frac1, barY, barW * frac2, barH)
  ctx.fillStyle = 'rgba(239,68,68,0.3)'; ctx.fillRect(barL + barW * frac1 + barW * frac2, barY, barW * (1 - frac1 - frac2), barH)

  ctx.strokeStyle = borderColor; ctx.lineWidth = 0.8
  ctx.strokeRect(barL, barY, barW, barH)

  // 指示器游标
  const indicatorX = barL + (modIndex.value / 1.155) * barW
  ctx.fillStyle = zone.value.color
  ctx.beginPath(); ctx.moveTo(indicatorX, barY - 4); ctx.lineTo(indicatorX - 6, barY - 12); ctx.lineTo(indicatorX + 6, barY - 12); ctx.closePath(); ctx.fill()
  ctx.fillStyle = zone.value.color; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText(`m=${modIndex.value.toFixed(3)}`, indicatorX, barY - 14)

  // 区域标签
  ctx.fillStyle = textSec; ctx.font = '9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('m=0', barL, barY + barH + 13)
  ctx.fillText('m=1.0', barL + barW * frac1, barY + barH + 13)
  ctx.fillText('m=1.155', barR, barY + barH + 13)
  ctx.fillText('线性区', barL + barW * frac1 / 2, barY - 6)
  ctx.fillText('过调制I区', barL + barW * (frac1 + frac2 / 2), barY - 6)
}

const animate = (ts: number) => {
  if (autoRotate.value && lastTs > 0) {
    const dt = Math.min((ts - lastTs) / 1000, 0.1)
    cumAngle += rotateSpeed.value * dt
    showAngle.value = ((cumAngle % 360) + 360) % 360
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
      <h4>过调制与六阶梯波仿真 — 线性区 → 过调制 → 六阶梯波</h4>
      <div class="header-controls">
        <button class="auto-btn" :class="{ active: autoRotate }" @click="autoRotate = !autoRotate">
          {{ autoRotate ? '&#9208; 暂停旋转' : '&#9654; 自动旋转' }}
        </button>
        <button class="auto-btn" :class="{ active: showIdeal }" @click="showIdeal = !showIdeal">
          显示理想Vref
        </button>
      </div>
    </div>

    <canvas ref="canvasRef" class="sim-canvas"></canvas>

    <div class="sim-params">
      <div class="param-group">
        <label>调制比 m: <span class="param-val">{{ modIndex.toFixed(3) }}</span></label>
        <input type="range" v-model.number="modIndex" :min="0.3" :max="1.155" :step="0.005" />
        <span class="param-range">0.3 ~ 1.155</span>
      </div>
      <div class="param-group">
        <label>Vref 角度 &#952;: <span class="param-val">{{ showAngle.toFixed(1) }}&deg;</span></label>
        <input type="range" v-model.number="showAngle" :min="0" :max="360" :step="0.5" />
        <span class="param-range">0&deg; ~ 360&deg;</span>
      </div>
      <div class="param-group" v-if="autoRotate">
        <label>旋转速度: <span class="param-val">{{ rotateSpeed }}&deg;/s</span></label>
        <input type="range" v-model.number="rotateSpeed" :min="1" :max="180" :step="1" />
        <span class="param-range">1 ~ 180 &deg;/s</span>
      </div>
    </div>

    <div class="sim-info">
      <div class="info-block">
        <div class="info-block-title">当前状态</div>
        <div class="info-row"><span class="info-label">调制比 m:</span><span class="info-value highlight">{{ modIndex.toFixed(3) }}</span></div>
        <div class="info-row"><span class="info-label">Vref 幅值:</span><span class="info-value">{{ vrefMag.toFixed(3) }} pu</span></div>
        <div class="info-row"><span class="info-label">区域:</span><span class="info-value highlight" :style="{ color: zoneInfo.color }">{{ zoneInfo.name }}</span></div>
        <div class="info-row"><span class="info-label">是否限幅:</span><span class="info-value">{{ vrefCurrent.isClipped ? '是 (六边形边界)' : '否' }}</span></div>
      </div>
      <div class="info-block">
        <div class="info-block-title">区域划分</div>
        <div class="info-row"><span class="info-label" style="color:#10b981">线性区 m&#8804;1.0: </span><span class="info-value">Vref在内切圆内, 正弦调制+零序=SVPWM马鞍波</span></div>
        <div class="info-row"><span class="info-label" style="color:#f59e0b">过调制I区 1.0&lt;m&lt;1.155: </span><span class="info-value">Vref部分超出六边形, 被限幅到边界</span></div>
        <div class="info-row"><span class="info-label" style="color:#ef4444">六阶梯波 m=1.155: </span><span class="info-value">Vref在六个顶点跳跃, 120°方波导通</span></div>
      </div>
    </div>

    <div class="explain-panel">
      <details open>
        <summary>过调制 (Over-modulation) 原理</summary>
        <div class="explain-content">
          <p><strong>线性区 (m&#8804;1)：</strong>SVPWM 的 Vref 轨迹完全位于六边形内切圆内部。相电压为正弦波叠加三次谐波零序分量，形成马鞍形调制波。线电压基波幅值达到 Vdc，直流母线利用率比 SPWM 提升 15.5%。</p>
          <p><strong>过调制 I 区 (1&lt;m&#8804;1.155)：</strong>Vref 部分超出六边形边界。实际输出电压矢量被限幅到六边形边缘上，产生电压幅值损失和相位误差。随 m 增大，输出电压逐渐从马鞍波向梯形波过渡。</p>
          <p><strong>六阶梯波 (m=1.155)：</strong>Vref 轨迹退化为六边形六个顶点的跳跃。每相导通 120&deg; 电角度，相电压波形为典型六阶梯波（方波）。基波幅值达到最大 = 2Vdc/&#960;，但含有大量 5、7、11、13 次谐波。</p>
          <p><strong>工程意义：</strong>过调制用于扩展电机转速范围（弱磁区），以牺牲电流波形质量为代价换取更高的基波电压输出。实际应用中 m 很少超过 1.05，仅在瞬态加速时短暂进入过调制区。</p>
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
.info-block { display: flex; flex-direction: column; gap: 0.15rem; min-width: 200px; max-width: 420px; }
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