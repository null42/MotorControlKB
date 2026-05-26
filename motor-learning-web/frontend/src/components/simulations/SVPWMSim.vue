<script setup lang="ts">
import { ref, computed, watch, onMounted, onUnmounted } from 'vue'

/**
 * SVPWM 矢量合成仿真 — 参考知乎文章 https://zhuanlan.zhihu.com/p/414721065
 *
 * Canvas 920×1620 大间距布局(无重叠，字体统一11px+):
 *   左上 (0~470, 0~445): 图5 αβ六边形矢量图 + 图12 内切圆外接圆
 *   右上 (480~910, 0~445): 图8 三相正弦ua/ub/uc + 图9 合成Vδ
 *   中部全宽 (15~905, 478~900): 图22/25 三角波载波+Tcmp(含↑开通↓关断标注)+PWM+死区+信息层
 *   下中 (15~905, 920~1070): 三相马鞍波(调制波=正弦+零序注入)
 *   底部 (15~905, 1130~1620): 表2/图14 六扇区开关顺序 + 图16对称性说明
 */

const canvasRef = ref<HTMLCanvasElement | null>(null)
let animFrameId = 0

const Vdc = ref(100)
const polePairs = ref(1)
const vrefMagMax = computed(() => Vdc.value / Math.sqrt(3))
const vrefMag = ref(57.7)
const vrefAngle = ref(30)
const svmMode = ref<'seven' | 'five'>('seven')
const autoRotate = ref(false)
const rotateSpeed = ref(60)
const pwmFreq = ref(10)
const deadTimeNs = ref(2000)

const TRAIL_LEN = 18
const trailAngles = ref<number[]>([])

let lastTs = 0

const toRad = (deg: number) => deg * Math.PI / 180

watch(Vdc, (nv) => {
  const mx = nv / Math.sqrt(3)
  if (vrefMag.value > mx) vrefMag.value = Math.round(mx * 10) / 10
})
watch(autoRotate, (v) => {
  if (v) { cumAngle = vrefAngle.value; trailAngles.value = [] }
  else trailAngles.value = []
})
watch(vrefAngle, (nv, ov) => {
  if (!autoRotate.value || Math.abs(nv - ov) > 5) cumAngle = nv
})

interface BasicVector { name: string; state: string; angleDeg: number }
const basicVectors: BasicVector[] = [
  { name: 'V4', state: '100', angleDeg: 0 },
  { name: 'V6', state: '110', angleDeg: 60 },
  { name: 'V2', state: '010', angleDeg: 120 },
  { name: 'V3', state: '011', angleDeg: 180 },
  { name: 'V1', state: '001', angleDeg: 240 },
  { name: 'V5', state: '101', angleDeg: 300 },
]

const nValue = computed(() => {
  const theta = toRad(vrefAngle.value)
  const uAlpha = vrefMag.value * Math.cos(theta)
  const uBeta = vrefMag.value * Math.sin(theta)
  const A = uBeta > 0 ? 1 : 0
  const B = ((Math.sqrt(3) / 2) * uAlpha - 0.5 * uBeta) > 0 ? 1 : 0
  const C = (-(Math.sqrt(3) / 2) * uAlpha - 0.5 * uBeta) > 0 ? 1 : 0
  return { N: 4 * C + 2 * B + A, A, B, C }
})

function getSectorByAngle(thetaDeg: number): number {
  let t = ((thetaDeg % 360) + 360) % 360
  if (t < 60) return 1; if (t < 120) return 2; if (t < 180) return 3
  if (t < 240) return 4; if (t < 300) return 5; return 6
}

const N_TO_SECTOR: Record<number, number> = { 3: 1, 1: 2, 5: 3, 4: 4, 6: 5, 2: 6 }

function getSector(thetaDeg: number): number {
  return N_TO_SECTOR[nValue.value.N] ?? getSectorByAngle(thetaDeg)
}

const xyzVars = computed(() => {
  const theta = toRad(vrefAngle.value)
  const uAlpha = vrefMag.value * Math.cos(theta)
  const uBeta = vrefMag.value * Math.sin(theta)
  const scale = Math.sqrt(3) / Vdc.value
  return {
    X: scale * uBeta,
    Y: scale * ((Math.sqrt(3) / 2) * uAlpha + 0.5 * uBeta),
    Z: scale * (-(Math.sqrt(3) / 2) * uAlpha + 0.5 * uBeta),
  }
})

function calcTimes(mag: number, thetaDeg: number): { T1: number; T2: number; T0: number; m: number } {
  const thetaLocal = ((thetaDeg % 360) + 360) % 360 % 60
  const tRad = toRad(thetaLocal)
  let T1 = (Math.sqrt(3) * mag / Vdc.value) * Math.sin(toRad(60) - tRad)
  let T2 = (Math.sqrt(3) * mag / Vdc.value) * Math.sin(tRad)
  if (T1 + T2 > 1) { const s = 1 / (T1 + T2); T1 *= s; T2 *= s }
  const T0 = Math.max(0, 1 - T1 - T2)
  return { T1, T2, T0, m: (Math.sqrt(3) * mag) / Vdc.value }
}

function getActiveVectors(thetaDeg: number): { v1: BasicVector; v2: BasicVector } {
  const sector = getSector(thetaDeg)
  return { v1: basicVectors[sector - 1], v2: basicVectors[sector % 6] }
}

const tcmTimes = computed(() => {
  const times = calcTimes(vrefMag.value, vrefAngle.value)
  const Ta = (1 - times.T1 - times.T2) / 4
  const Tb = Ta + times.T1 / 2
  const Tc = Tb + times.T2 / 2
  const map: Record<number, { Tcm1: number; Tcm2: number; Tcm3: number }> = {
    3: { Tcm1: Ta, Tcm2: Tb, Tcm3: Tc }, 1: { Tcm1: Tb, Tcm2: Ta, Tcm3: Tc },
    5: { Tcm1: Tc, Tcm2: Ta, Tcm3: Tb }, 4: { Tcm1: Tc, Tcm2: Tb, Tcm3: Ta },
    6: { Tcm1: Tb, Tcm2: Tc, Tcm3: Ta }, 2: { Tcm1: Ta, Tcm2: Tc, Tcm3: Tb },
  }
  return map[nValue.value.N] ?? { Tcm1: 0, Tcm2: 0, Tcm3: 0 }
})

const phaseVoltages = computed(() => {
  const ma = vrefMag.value / (Vdc.value / 2)
  const theta = toRad(vrefAngle.value)
  const vas = (Vdc.value / 2) * ma * Math.sin(theta)
  const vbs = (Vdc.value / 2) * ma * Math.sin(theta - 2 * Math.PI / 3)
  const vcs = (Vdc.value / 2) * ma * Math.sin(theta + 2 * Math.PI / 3)
  const vmax = Math.max(vas, vbs, vcs), vmin = Math.min(vas, vbs, vcs)
  const voffset = -(vmax + vmin) / 2
  return { vas, vbs, vcs, voffset, vasInj: vas + voffset, vbsInj: vbs + voffset, vcsInj: vcs + voffset, ma }
})

const segmentDetail = computed(() => {
  const times = calcTimes(vrefMag.value, vrefAngle.value)
  const sector = getSector(vrefAngle.value)
  const { v1, v2 } = getActiveVectors(vrefAngle.value)
  if (svmMode.value === 'seven') {
    return [
      { label: 'V0', state: '000', dur: times.T0 / 4 },
      { label: v1.name, state: v1.state, dur: times.T1 / 2 },
      { label: v2.name, state: v2.state, dur: times.T2 / 2 },
      { label: 'V7', state: '111', dur: times.T0 / 2 },
      { label: v2.name, state: v2.state, dur: times.T2 / 2 },
      { label: v1.name, state: v1.state, dur: times.T1 / 2 },
      { label: 'V0', state: '000', dur: times.T0 / 4 },
    ]
  }
  if (sector % 2 === 1) {
    return [
      { label: v1.name, state: v1.state, dur: times.T1 / 2 },
      { label: v2.name, state: v2.state, dur: times.T2 / 2 },
      { label: 'V7', state: '111', dur: times.T0 },
      { label: v2.name, state: v2.state, dur: times.T2 / 2 },
      { label: v1.name, state: v1.state, dur: times.T1 / 2 },
    ]
  }
  return [
    { label: 'V0', state: '000', dur: times.T0 / 2 },
    { label: v1.name, state: v1.state, dur: times.T1 / 2 },
    { label: v2.name, state: v2.state, dur: times.T2 / 2 },
    { label: v1.name, state: v1.state, dur: times.T1 / 2 },
    { label: 'V0', state: '000', dur: times.T0 / 2 },
  ]
})

const switchSequenceTable = computed(() => {
  const seq: Record<number, { order: string[]; firstVec: string; secondVec: string }> = {
    1: { order: ['V0','V4','V6','V7','V7','V6','V4','V0'], firstVec: 'V4', secondVec: 'V6' },
    2: { order: ['V0','V6','V2','V7','V7','V2','V6','V0'], firstVec: 'V6', secondVec: 'V2' },
    3: { order: ['V0','V2','V3','V7','V7','V3','V2','V0'], firstVec: 'V2', secondVec: 'V3' },
    4: { order: ['V0','V3','V1','V7','V7','V1','V3','V0'], firstVec: 'V3', secondVec: 'V1' },
    5: { order: ['V0','V1','V5','V7','V7','V5','V1','V0'], firstVec: 'V1', secondVec: 'V5' },
    6: { order: ['V0','V5','V4','V7','V7','V4','V5','V0'], firstVec: 'V5', secondVec: 'V4' },
  }
  return seq[getSector(vrefAngle.value)] ?? seq[1]
})

// ═════════════════════ 绘图 ═════════════════════
const draw = () => {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  const W = 920, H = 1620
  canvas.width = W; canvas.height = H

  const style = getComputedStyle(document.documentElement)
  const bgColor = style.getPropertyValue('--bg-color').trim() || '#f8fafc'
  const textColor = style.getPropertyValue('--text-color').trim() || '#1e293b'
  const borderColor = style.getPropertyValue('--border-color').trim() || '#e2e8f0'
  const primaryColor = style.getPropertyValue('--primary-color').trim() || '#3b82f6'

  const COL_TEXT2 = '#0ea5e9'
  const COL_LINE2 = '#38bdf8'
  const COL_VECLINE = '#8b5cf6'

  ctx.clearRect(0, 0, W, H)
  ctx.fillStyle = bgColor; ctx.fillRect(0, 0, W, H)

  const MID_X = 475, Y1 = 465, Y2 = 900, Y3 = 1115
  ctx.strokeStyle = borderColor; ctx.lineWidth = 1; ctx.setLineDash([6, 4])
  ;[Y1, Y2, Y3].forEach(y => { ctx.beginPath(); ctx.moveTo(15, y); ctx.lineTo(W - 15, y); ctx.stroke() })
  ctx.beginPath(); ctx.moveTo(MID_X, 12); ctx.lineTo(MID_X, Y1); ctx.stroke()
  ctx.setLineDash([])

  const sector = getSector(vrefAngle.value)
  const { v1: av1, v2: av2 } = getActiveVectors(vrefAngle.value)
  const times = calcTimes(vrefMag.value, vrefAngle.value)
  const thetaRad = toRad(vrefAngle.value)

  // ══════════════════════════════════════════════════
  //  左面板: α-β 六边形矢量图 + 内切圆外接圆(图12)
  // ══════════════════════════════════════════════════
  const cx = 235, cy = 210, hexR = 165
  const hexVertices = basicVectors.map(v => ({ x: hexR * Math.cos(toRad(v.angleDeg)), y: hexR * Math.sin(toRad(v.angleDeg)) }))
  const toCX = (mx: number) => cx + mx
  const toCY = (my: number) => cy - my

  ctx.strokeStyle = borderColor; ctx.lineWidth = 1; ctx.setLineDash([3, 5])
  hexVertices.forEach(v => { ctx.beginPath(); ctx.moveTo(toCX(0), toCY(0)); ctx.lineTo(toCX(v.x), toCY(v.y)); ctx.stroke() })
  ctx.setLineDash([])

  // 扇区高亮 — 边界平滑过渡
  const sectorAngleDeg = ((vrefAngle.value % 360) + 360) % 360
  const sectorStartDeg = (sector - 1) * 60
  const distToStart = sectorAngleDeg - sectorStartDeg
  const distToEnd = sectorStartDeg + 60 - sectorAngleDeg
  const blendRange = 4

  ctx.fillStyle = 'rgba(59, 130, 246, 0.08)'
  ctx.beginPath(); ctx.moveTo(toCX(0), toCY(0))
  ctx.lineTo(toCX(hexVertices[sector - 1].x), toCY(hexVertices[sector - 1].y))
  ctx.lineTo(toCX(hexVertices[sector % 6].x), toCY(hexVertices[sector % 6].y)); ctx.closePath(); ctx.fill()

  if (distToEnd < blendRange) {
    const ns = sector % 6
    const alpha = (1 - distToEnd / blendRange) * 0.07
    ctx.fillStyle = `rgba(59, 130, 246, ${alpha})`
    ctx.beginPath(); ctx.moveTo(toCX(0), toCY(0))
    ctx.lineTo(toCX(hexVertices[ns].x), toCY(hexVertices[ns].y))
    ctx.lineTo(toCX(hexVertices[(ns + 1) % 6].x), toCY(hexVertices[(ns + 1) % 6].y)); ctx.closePath(); ctx.fill()
  }
  if (distToStart < blendRange && distToStart >= 0) {
    const ps = sector === 1 ? 5 : sector - 2
    const alpha = (1 - distToStart / blendRange) * 0.07
    ctx.fillStyle = `rgba(59, 130, 246, ${alpha})`
    ctx.beginPath(); ctx.moveTo(toCX(0), toCY(0))
    ctx.lineTo(toCX(hexVertices[ps].x), toCY(hexVertices[ps].y))
    ctx.lineTo(toCX(hexVertices[(ps + 1) % 6].x), toCY(hexVertices[(ps + 1) % 6].y)); ctx.closePath(); ctx.fill()
  }

  const inscribedR = hexR * Math.sqrt(3) / 2
  const circumscribedR = hexR
  ctx.setLineDash([4, 4]); ctx.lineWidth = 1
  ctx.strokeStyle = '#10b981'; ctx.globalAlpha = 0.5
  ctx.beginPath(); ctx.arc(toCX(0), toCY(0), inscribedR, 0, 2 * Math.PI); ctx.stroke()
  ctx.globalAlpha = 1
  ctx.fillStyle = '#10b981'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText('内切圆(m=1)', toCX(inscribedR * 0.65), toCY(-inscribedR * 0.65))

  ctx.strokeStyle = '#f59e0b'; ctx.setLineDash([3, 3])
  ctx.beginPath(); ctx.arc(toCX(0), toCY(0), circumscribedR, 0, 2 * Math.PI); ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#f59e0b'; ctx.font = 'bold 10px sans-serif'
  ctx.fillText('外接圆(m=1.155)', toCX(circumscribedR * 0.72), toCY(-circumscribedR * 0.58))

  ctx.strokeStyle = primaryColor; ctx.lineWidth = 2; ctx.beginPath()
  ctx.moveTo(toCX(hexVertices[0].x), toCY(hexVertices[0].y))
  for (let i = 1; i < 6; i++) ctx.lineTo(toCX(hexVertices[i].x), toCY(hexVertices[i].y))
  ctx.closePath(); ctx.stroke()

  const axisLen = hexR + 24
  ctx.strokeStyle = COL_LINE2; ctx.lineWidth = 1
  ctx.beginPath(); ctx.moveTo(toCX(-axisLen), toCY(0)); ctx.lineTo(toCX(axisLen), toCY(0)); ctx.stroke()
  ctx.beginPath(); ctx.moveTo(toCX(0), toCY(-axisLen)); ctx.lineTo(toCX(0), toCY(axisLen)); ctx.stroke()
  ctx.fillStyle = COL_LINE2; ctx.font = 'bold 13px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('α', toCX(axisLen + 7), toCY(0) + 5)
  ctx.fillText('β', toCX(0) + 5, toCY(axisLen + 7) + 4)

  hexVertices.forEach((vert, i) => {
    const v = basicVectors[i]
    ctx.strokeStyle = COL_VECLINE; ctx.lineWidth = 1.2
    ctx.beginPath(); ctx.moveTo(toCX(0), toCY(0)); ctx.lineTo(toCX(vert.x), toCY(vert.y)); ctx.stroke()
    ctx.fillStyle = primaryColor; ctx.beginPath(); ctx.arc(toCX(vert.x), toCY(vert.y), 4.5, 0, 2 * Math.PI); ctx.fill()
    ctx.fillStyle = textColor; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
    ctx.fillText(`${v.name}`, toCX((hexR + 18) * Math.cos(toRad(v.angleDeg))), toCY((hexR + 18) * Math.sin(toRad(v.angleDeg))))
  })

  ctx.fillStyle = '#ef4444'; ctx.beginPath(); ctx.arc(toCX(0), toCY(0), 4.5, 0, 2 * Math.PI); ctx.fill()
  ctx.fillStyle = textColor; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'right'
  ctx.fillText('V0', toCX(-10), toCY(-10))
  ctx.fillStyle = '#f59e0b'; ctx.beginPath(); ctx.arc(toCX(5), toCY(-8), 3.5, 0, 2 * Math.PI); ctx.fill()
  ctx.fillText('V7', toCX(17), toCY(-8))

  const ppm = hexR / (2 * Vdc.value / 3)
  const vrefPx = vrefMag.value * ppm

  // Vref 轨迹拖尾 — 扇区切换视觉平滑
  const trail = trailAngles.value
  for (let ti = trail.length - 1; ti >= 0; ti--) {
    const tAngle = trail[ti]
    const tRad = toRad(tAngle)
    const tPx = vrefPx
    const tX = tPx * Math.cos(tRad), tY = tPx * Math.sin(tRad)
    const alpha = (1 - ti / TRAIL_LEN) * 0.25
    ctx.strokeStyle = `rgba(239, 68, 68, ${alpha})`; ctx.lineWidth = 2 * (1 - ti / TRAIL_LEN)
    ctx.beginPath(); ctx.moveTo(toCX(0), toCY(0)); ctx.lineTo(toCX(tX), toCY(tY)); ctx.stroke()
  }

  const vrefX = vrefPx * Math.cos(thetaRad), vrefY = vrefPx * Math.sin(thetaRad)
  ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 2.5
  ctx.beginPath(); ctx.moveTo(toCX(0), toCY(0)); ctx.lineTo(toCX(vrefX), toCY(vrefY)); ctx.stroke()
  const aLen = 12, aR1 = thetaRad + Math.PI * 0.82, aR2 = thetaRad - Math.PI * 0.82
  ctx.fillStyle = '#ef4444'; ctx.beginPath()
  ctx.moveTo(toCX(vrefX), toCY(vrefY))
  ctx.lineTo(toCX(vrefX) - aLen * Math.cos(aR1), toCY(vrefY) + aLen * Math.sin(aR1))
  ctx.lineTo(toCX(vrefX) - aLen * Math.cos(aR2), toCY(vrefY) + aLen * Math.sin(aR2))
  ctx.closePath(); ctx.fill()
  ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText('Vref', toCX(vrefX) + 14 * Math.cos(thetaRad), toCY(vrefY) - 14 * Math.sin(thetaRad))

  const r1 = toRad(av1.angleDeg), r2 = toRad(av2.angleDeg)
  const c1X = times.T1 * hexR * Math.cos(r1), c1Y = times.T1 * hexR * Math.sin(r1)
  const c2X = times.T2 * hexR * Math.cos(r2), c2Y = times.T2 * hexR * Math.sin(r2)
  ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 1.3; ctx.setLineDash([4, 4])
  ctx.beginPath(); ctx.moveTo(toCX(vrefX), toCY(vrefY)); ctx.lineTo(toCX(c1X), toCY(c1Y)); ctx.stroke()
  ctx.strokeStyle = '#8b5cf6'
  ctx.beginPath(); ctx.moveTo(toCX(vrefX), toCY(vrefY)); ctx.lineTo(toCX(c2X), toCY(c2Y)); ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#f59e0b'; ctx.beginPath(); ctx.arc(toCX(c1X), toCY(c1Y), 3.5, 0, 2 * Math.PI); ctx.fill()
  ctx.fillStyle = '#8b5cf6'; ctx.beginPath(); ctx.arc(toCX(c2X), toCY(c2Y), 3.5, 0, 2 * Math.PI); ctx.fill()

  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
  const roman = ['I','II','III','IV','V','VI']
  for (let s = 1; s <= 6; s++) {
    const rad = toRad((s - 1) * 60 + 30)
    ctx.fillText(roman[s-1], toCX(hexR * 0.52 * Math.cos(rad)), toCY(hexR * 0.52 * Math.sin(rad)))
  }

  ctx.fillStyle = textColor; ctx.font = 'bold 10px monospace'; ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic'
  ctx.fillText(`图5/图12: N=4C+2B+A=${nValue.value.N} → 扇区${sector}`, 14, 22)
  ctx.font = '9px monospace'; ctx.fillStyle = COL_TEXT2
  ctx.fillText(`A=${nValue.value.A} B=${nValue.value.B} C=${nValue.value.C}`, 14, 36)

  // ══════════════════════════════════════════════════
  //  右上: 图8 — 三相正弦电压 ua/ub/uc
  // ══════════════════════════════════════════════════
  const R8L = 490, R8T = 16, R8W = 418, R8H = 360
  ctx.fillStyle = textColor; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('图8/9: 三相正弦电压 uₐ, u_b, u_c (幅值uₘ) — 合成空间矢量Vδ', R8L + R8W/2, R8T + 14)

  const sinMidY = R8T + R8H/2 + 20
  const sinAmp = R8H * 0.36
  ctx.strokeStyle = COL_LINE2; ctx.lineWidth = 0.5
  ctx.beginPath(); ctx.moveTo(R8L + 35, sinMidY); ctx.lineTo(R8L + R8W - 8, sinMidY); ctx.stroke()

  const sinColors = ['#ef4444','#10b981','#3b82f6']
  const sinNames = ['uₐ','u_b','u_c']
  const N_SIN = 720
  for (let ph = 0; ph < 3; ph++) {
    ctx.strokeStyle = sinColors[ph]; ctx.lineWidth = 1.3
    ctx.beginPath()
    for (let i = 0; i <= N_SIN; i++) {
      const th = (i / N_SIN) * 720 * Math.PI / 180
      const val = vrefMag.value * Math.sin(th - ph * 2 * Math.PI / 3)
      const x = R8L + 35 + (i / N_SIN) * (R8W - 45)
      const y = sinMidY - val * sinAmp / vrefMagMax.value
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
    }
    ctx.stroke()
    ctx.fillStyle = sinColors[ph]; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'left'
    const lblIdx = Math.floor(N_SIN * 0.04 + ph * N_SIN * 0.14)
    const lblTh = (lblIdx / N_SIN) * 720 * Math.PI / 180
    const lblVal = vrefMag.value * Math.sin(lblTh - ph * 2 * Math.PI / 3)
    ctx.fillText(sinNames[ph], R8L + 35 + (lblIdx/N_SIN)*(R8W-45) + 3, sinMidY - lblVal*sinAmp/vrefMagMax.value - 4)
  }

  const cursorFrac360 = ((vrefAngle.value % 360) + 360) % 360 / 360
  const cursorX8 = R8L + 35 + cursorFrac360 * (R8W - 45)
  ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 1.5; ctx.setLineDash([4, 3])
  ctx.beginPath(); ctx.moveTo(cursorX8, R8T + 26); ctx.lineTo(cursorX8, R8T + R8H - 8); ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#f59e0b'; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText(`${vrefAngle.value.toFixed(1)}°`, cursorX8, R8T + R8H - 2)

  ctx.fillStyle = COL_TEXT2; ctx.font = '9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('0°', R8L + 35, R8T + R8H + 14)
  ctx.fillText('360°', R8L + R8W - 8, R8T + R8H + 14)
  ctx.fillText('图9: Vδ=1.5·uₘ·e^(jωₑt) 合成空间矢量', R8L + R8W/2, R8T + R8H + 28)

  // ══════════════════════════════════════════════════
  //  ★ 中部: 图22/25 三角波载波 + Tcmp + PWM + 完整信息层
  // ══════════════════════════════════════════════════
  const PWML = 40, PWMR = W - 40, PWMW = PWMR - PWML
  const PWMT = 480
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText(`图22/25 三角波载波比较法 — 扇区${sector}: ${av1.name}+${av2.name}`, W/2, PWMT + 14)

  const triTop = PWMT + 34, triH = 125
  const triBot = triTop + triH
  const pwmRowH = 46, pwmLabelW = 60
  const DL = PWML + pwmLabelW + 4, DW = PWMW - pwmLabelW - 10

  // 三角波载波
  const triPeak = 0.5
  ctx.strokeStyle = COL_VECLINE; ctx.lineWidth = 1.5
  ctx.beginPath()
  const N_TRI = 400
  for (let i = 0; i <= N_TRI; i++) {
    const frac = i / N_TRI
    const tv = frac < 0.5 ? frac * triPeak * 2 : (1 - frac) * triPeak * 2
    const px = DL + frac * DW
    const py = triBot - tv / triPeak * triH
    if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py)
  }
  ctx.stroke()
  ctx.fillStyle = COL_TEXT2; ctx.font = '11px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText('Ts/2', DL + DW/2 + 5, triTop - 2)
  ctx.fillText('0', DL - 4, triBot + 14)
  ctx.fillText('Ts', DL + DW, triBot + 14)

  // Tcmp 阈值线 + 标注点
  const tcm = tcmTimes.value
  const sortedTcms = [
    { val: tcm.Tcm1, color: '#ef4444', name: 'Tcmp₁', phase: 'A' },
    { val: tcm.Tcm2, color: '#10b981', name: 'Tcmp₂', phase: 'B' },
    { val: tcm.Tcm3, color: '#3b82f6', name: 'Tcmp₃', phase: 'C' },
  ].sort((a, b) => a.val - b.val)

  sortedTcms.forEach(tc => {
    const ty = triBot - (tc.val / 0.5) * triH
    const riseFrac = tc.val / 0.5 * 0.5
    const fallFrac = 1 - riseFrac
    const riseX = DL + riseFrac * DW
    const fallX = DL + fallFrac * DW
    ctx.strokeStyle = tc.color; ctx.lineWidth = 1; ctx.setLineDash([4, 3])
    ctx.beginPath(); ctx.moveTo(DL, ty); ctx.lineTo(DL + DW, ty); ctx.stroke()
    ctx.setLineDash([])
    ctx.fillStyle = tc.color; ctx.beginPath(); ctx.arc(riseX, ty, 4.5, 0, 2*Math.PI); ctx.fill()
    ctx.fillStyle = tc.color; ctx.beginPath(); ctx.arc(fallX, ty, 4.5, 0, 2*Math.PI); ctx.fill()
    ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'left'
    ctx.fillText(tc.name, riseX + 7, ty - 5)
    ctx.fillStyle = '#f59e0b'; ctx.font = 'bold 8px sans-serif'; ctx.textAlign = 'center'
    ctx.fillText('↑开通', riseX, ty - 14)
    ctx.fillText('↓关断', fallX, ty - 14)
  })

  const deadFrac = deadTimeNs.value * 1e-9 * pwmFreq.value * 1000
  const deadPx = deadFrac / 1000 * DW
  const hasDeadTime = deadTimeNs.value > 0

  // PWMA / PWMB / PWMC 输出波形
  const pwmPhases = [
    { name: 'PWMA', color: '#ef4444' }, { name: 'PWMB', color: '#10b981' }, { name: 'PWMC', color: '#3b82f6' },
  ]
  pwmPhases.forEach((pp, pi) => {
    const rowTop = triBot + 14 + pi * pwmRowH
    const rowMid = rowTop + pwmRowH / 2
    ctx.fillStyle = pp.color; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'right'
    ctx.fillText(pp.name, DL - 10, rowMid + 4)
    ctx.strokeStyle = borderColor; ctx.lineWidth = 0.5
    ctx.strokeRect(DL, rowTop, DW, pwmRowH)
    const tcmVal = [tcm.Tcm1, tcm.Tcm2, tcm.Tcm3][pi]
    ctx.fillStyle = pp.color + '50'
    for (let i = 0; i < N_TRI; i++) {
      const frac = i / N_TRI
      const tv = frac < 0.5 ? frac * triPeak * 2 : (1 - frac) * triPeak * 2
      if (tv > tcmVal) {
        const px = DL + frac * DW
        const pxNext = DL + ((i+1)/N_TRI) * DW
        ctx.fillRect(px, rowTop + 5, Math.max(1, pxNext-px), pwmRowH - 10)
      }
    }
    if (hasDeadTime) {
      const riseFrac = tcmVal / 0.5 * 0.5
      const riseX1 = DL + riseFrac * DW
      const gapW = Math.max(1, deadPx)
      ctx.fillStyle = 'rgba(239,68,68,0.25)'; ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 0.8
      ctx.fillRect(riseX1, rowTop + 5, gapW, pwmRowH - 10)
      ctx.strokeRect(riseX1, rowTop + 5, gapW, pwmRowH - 10)
      ctx.setLineDash([2, 2]); ctx.lineWidth = 0.6
      ctx.beginPath(); ctx.moveTo(riseX1 + gapW, rowTop + 5); ctx.lineTo(riseX1 + gapW, rowTop + pwmRowH - 5); ctx.stroke()
      ctx.setLineDash([])
    }
  })

  if (hasDeadTime) {
    ctx.fillStyle = '#ef4444'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'center'
    const deadNoteY = triBot + 14 + 3 * pwmRowH + 6
    ctx.fillText(`死区 Tdead = ${deadTimeNs}ns  (PWM ${pwmFreq}kHz → Ts=${(1000/pwmFreq).toFixed(0)}μs, 占比 ${(deadFrac).toFixed(1)}‰)`, DL + DW/2, deadNoteY)
    ctx.fillText('↑ 红色阴影区 = 上升沿延迟开通，防止上下管直通 (shoot-through)', DL + DW/2, deadNoteY + 14)
  }

  // ★ 完整信息层 (统一用 DL/DW 坐标系)
  const infoTop = triBot + 14 + 3 * pwmRowH + (hasDeadTime ? 46 : 18)
  const infoRowH = 30
  const segs = segmentDetail.value

  // 第1行: 时间标签
  const timeLabels = ['T₀/4','T₁/2','T₂/2','T₀/2','T₀/2','T₂/2','T₁/2','T₀/4']
  let sc = 0
  timeLabels.forEach((tl, ti) => {
    if (ti >= segs.length) return
    const sx = DL + sc * DW
    const sw = Math.max(1, segs[ti].dur * DW)
    ctx.fillStyle = COL_TEXT2; ctx.font = '11px monospace'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
    if (sw > 28) ctx.fillText(tl, sx + sw/2, infoTop + 14)
    sc += segs[ti].dur
  })
  ctx.textBaseline = 'alphabetic'

  // 第2行: 矢量名
  const vecNameMap: Record<string, string> = {
    '000':'Ũ₀', '100':'Ũ₄', '110':'Ũ₆', '010':'Ũ₂',
    '011':'Ũ₃', '001':'Ũ₁', '101':'Ũ₅', '111':'Ũ₇',
  }
  sc = 0
  segs.forEach((seg) => {
    const sx = DL + sc * DW
    const sw = Math.max(1, seg.dur * DW)
    if (sw > 28) {
      ctx.fillStyle = textColor; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
      ctx.fillText(vecNameMap[seg.state] || seg.label, sx + sw/2, infoTop + 14 + infoRowH)
    }
    sc += seg.dur
  })
  ctx.textBaseline = 'alphabetic'

  // 第3行: 开关状态编码 + 颜色背景
  sc = 0
  segs.forEach((seg) => {
    const sx = DL + sc * DW
    const sw = Math.max(1, seg.dur * DW)
    if (seg.state === '000') ctx.fillStyle = 'rgba(239,68,68,0.15)'
    else if (seg.state === '111') ctx.fillStyle = 'rgba(245,158,11,0.15)'
    else ctx.fillStyle = 'rgba(59,130,246,0.12)'
    ctx.fillRect(sx, infoTop + infoRowH*2, sw, infoRowH)
    ctx.strokeStyle = borderColor; ctx.lineWidth = 0.5
    ctx.strokeRect(sx, infoTop + infoRowH*2, sw, infoRowH)
    if (sw > 30) {
      ctx.fillStyle = textColor; ctx.font = 'bold 12px monospace'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
      ctx.fillText(seg.state, sx + sw/2, infoTop + infoRowH*2 + infoRowH/2)
    }
    ctx.textBaseline = 'alphabetic'
    sc += seg.dur
  })

  // ★ 竖虚线: 统一用 DL/DW，从三角波顶部贯穿到信息层底部
  const infoBottomY = infoTop + infoRowH * 3
  ctx.setLineDash([3, 4]); ctx.strokeStyle = COL_LINE2; ctx.lineWidth = 0.8
  sc = 0
  segs.forEach((seg, si) => {
    if (si > 0) {
      const vx = DL + sc * DW
      ctx.beginPath(); ctx.moveTo(vx, triTop - 8); ctx.lineTo(vx, infoBottomY + 8); ctx.stroke()
    }
    sc += seg.dur
  })
  ctx.setLineDash([])

  // ★ 双向箭头 ↔ 标注各段时间
  const arrowY = infoTop - 14
  ;[
    { x: DL, label: '', dur: segs[0]?.dur },
    { x: DL + (segs[0]?.dur||0) * DW, label: 'Tₐ', dur: segs[1]?.dur },
    { x: DL + ((segs[0]?.dur||0)+(segs[1]?.dur||0)) * DW, label: 'T_b', dur: segs[2]?.dur },
    { x: DL + ((segs[0]?.dur||0)+(segs[1]?.dur||0)+(segs[2]?.dur||0)) * DW, label: 'T_c', dur: segs[3]?.dur },
  ].forEach(pt => {
    if (!pt.label || pt.dur === undefined) return
    const sx = pt.x, sw = pt.dur * DW
    if (sw < 24) return
    const midX = sx + sw / 2
    ctx.strokeStyle = '#8b5cf6'; ctx.lineWidth = 1.2
    ctx.beginPath(); ctx.moveTo(sx + 2, arrowY); ctx.lineTo(midX - 6, arrowY); ctx.stroke()
    ctx.beginPath(); ctx.moveTo(midX + 6, arrowY); ctx.lineTo(sx + sw - 2, arrowY); ctx.stroke()
    ctx.fillStyle = '#8b5cf6'
    ctx.beginPath(); ctx.moveTo(sx + 2, arrowY); ctx.lineTo(sx + 7, arrowY - 3); ctx.lineTo(sx + 7, arrowY + 3); ctx.closePath(); ctx.fill()
    ctx.beginPath(); ctx.moveTo(sx + sw - 2, arrowY); ctx.lineTo(sx + sw - 7, arrowY - 3); ctx.lineTo(sx + sw - 7, arrowY + 3); ctx.closePath(); ctx.fill()
    ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
    ctx.fillText(pt.label, midX, arrowY - 6)
  })

  // Ta/Tb/Tc 向下箭头
  const downArrowY = infoBottomY + 18
  const Ta_x = DL + segs[0].dur * DW
  const Tb_x = DL + (segs[0].dur + segs[1].dur) * DW
  const Tc_x = DL + (segs[0].dur + segs[1].dur + segs[2].dur) * DW
  ;[{ x: Ta_x, label: 'Tₐ' }, { x: Tb_x, label: 'T_b' }, { x: Tc_x, label: 'T_c' }].forEach(pt => {
    ctx.fillStyle = '#8b5cf6'; ctx.beginPath()
    ctx.moveTo(pt.x, downArrowY - 2); ctx.lineTo(pt.x - 5, downArrowY - 9); ctx.lineTo(pt.x + 5, downArrowY - 9); ctx.closePath(); ctx.fill()
    ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
    ctx.fillText(pt.label, pt.x, downArrowY + 14)
  })

  // 右侧公式框
  const formulaX = PWMR - 178, formulaY = PWMT + 36
  ctx.fillStyle = 'rgba(248,250,252,0.93)'; ctx.fillRect(formulaX, formulaY, 172, 82)
  ctx.strokeStyle = borderColor; ctx.lineWidth = 0.8; ctx.strokeRect(formulaX, formulaY, 172, 82)
  ctx.fillStyle = textColor; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('切换点的时间计算', formulaX + 86, formulaY + 16)
  ctx.font = '11px monospace'; ctx.textAlign = 'left'
  ctx.fillText('Tₐ=(Ts−T₁−T₂)/4', formulaX + 8, formulaY + 36)
  ctx.fillText('T_b=Tₐ+T₁/2', formulaX + 8, formulaY + 54)
  ctx.fillText('T_c=T_b+T₂/2', formulaX + 8, formulaY + 72)

  // 右下角扇区编号
  const romanSect = ['I','II','III','IV','V','VI']
  ctx.fillStyle = textColor; ctx.font = 'bold 14px sans-serif'; ctx.textAlign = 'right'
  ctx.fillText(`第 ${romanSect[sector-1]} 扇区`, PWMR, downArrowY + 14)
  ctx.fillStyle = COL_TEXT2; ctx.font = '11px sans-serif'; ctx.textAlign = 'right'
  ctx.fillText(segs.map(s => s.state).join('-'), PWMR, downArrowY + 30)

  // ══════════════════════════════════════════════════
  //  下中: 三相马鞍波
  // ══════════════════════════════════════════════════
  const SADL = 42, SADR = W - 18, SADW = SADR - SADL, SADT = 920, SADH = 160
  ctx.fillStyle = textColor; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('三相马鞍波（调制波 = 正弦 + 零序注入）', W/2, SADT - 8)

  const sadMidY = SADT + SADH/2
  ctx.strokeStyle = COL_LINE2; ctx.lineWidth = 0.5
  ctx.beginPath(); ctx.moveTo(SADL, sadMidY); ctx.lineTo(SADR, sadMidY); ctx.stroke()
  ctx.setLineDash([3, 5]); ctx.strokeStyle = COL_LINE2; ctx.lineWidth = 0.5
  const sadScale = SADH / (Vdc.value * 1.2)
  ctx.beginPath(); ctx.moveTo(SADL, sadMidY - (Vdc.value/2)*sadScale); ctx.lineTo(SADR, sadMidY - (Vdc.value/2)*sadScale); ctx.stroke()
  ctx.beginPath(); ctx.moveTo(SADL, sadMidY + (Vdc.value/2)*sadScale); ctx.lineTo(SADR, sadMidY + (Vdc.value/2)*sadScale); ctx.stroke()
  ctx.setLineDash([])

  const colors = ['#ef4444', '#10b981', '#3b82f6']
  const N_SADDLE = 720
  const phaseA: number[] = [], phaseB: number[] = [], phaseC: number[] = []
  for (let i = 0; i <= N_SADDLE; i++) {
    const theta = (i / N_SADDLE) * 360 * Math.PI / 180
    const vas = vrefMag.value * Math.sin(theta)
    const vbs = vrefMag.value * Math.sin(theta - 2*Math.PI/3)
    const vcs = vrefMag.value * Math.sin(theta + 2*Math.PI/3)
    const vmax = Math.max(vas, vbs, vcs), vmin = Math.min(vas, vbs, vcs)
    const vo = -(vmax + vmin) / 2
    phaseA.push(vas + vo); phaseB.push(vbs + vo); phaseC.push(vcs + vo)
  }

  const sadPhases = [
    { name: 'Va*', data: phaseA, color: colors[0] },
    { name: 'Vb*', data: phaseB, color: colors[1] },
    { name: 'Vc*', data: phaseC, color: colors[2] },
  ]
  sadPhases.forEach(ph => {
    ctx.strokeStyle = ph.color; ctx.lineWidth = 1.3
    ctx.beginPath()
    for (let i = 0; i <= N_SADDLE; i++) {
      const x = SADL + (i/N_SADDLE)*SADW, y = sadMidY - ph.data[i]*sadScale
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
    }
    ctx.stroke()
    ctx.fillStyle = ph.color; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'left'
    const li = Math.floor(N_SADDLE * 0.07)
    ctx.fillText(ph.name, SADL + (li/N_SADDLE)*SADW + 4, sadMidY - ph.data[li]*sadScale - 4)
  })

  const cursorX = SADL + cursorFrac360 * SADW
  ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 1.5; ctx.setLineDash([4, 3])
  ctx.beginPath(); ctx.moveTo(cursorX, SADT); ctx.lineTo(cursorX, SADT + SADH); ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#f59e0b'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText(`θ=${vrefAngle.value.toFixed(1)}°`, cursorX, SADT - 4)

  const pvNow = phaseVoltages.value
  ;[
    { v: pvNow.vasInj, c: colors[0] }, { v: pvNow.vbsInj, c: colors[1] }, { v: pvNow.vcsInj, c: colors[2] },
  ].forEach(iv => {
    ctx.fillStyle = iv.c; ctx.beginPath(); ctx.arc(cursorX, sadMidY - iv.v*sadScale, 3.5, 0, 2*Math.PI); ctx.fill()
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 0.8
    ctx.beginPath(); ctx.arc(cursorX, sadMidY - iv.v*sadScale, 3.5, 0, 2*Math.PI); ctx.stroke()
  })

  ctx.fillStyle = COL_TEXT2; ctx.font = '9px sans-serif'; ctx.textAlign = 'center'
  for (let d = 0; d <= 360; d += 90) ctx.fillText(`${d}°`, SADL + (d/360)*SADW, SADT + SADH + 12)
  ctx.fillText('基波周期（电气角度）', W/2, SADT + SADH + 24)

  // ══════════════════════════════════════════════════
  //  底部: 表2 6扇区×8段开关顺序
  // ══════════════════════════════════════════════════
  const TBL = 24, TBT = 1130, TBL_CELL_W = 68, TBL_CELL_H = 26
  const tblHeaders = ['', 'I(1)', 'II(2)', 'III(3)', 'IV(4)', 'V(5)', 'VI(6)']

  ctx.fillStyle = textColor; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('表2 / 图14: 各扇区开关切换顺序 (知乎 §2) — 当前扇区高亮', W/2, TBT + 11)

  ctx.fillStyle = '#f1f5f9'; ctx.fillRect(TBL, TBT + 28, TBL_CELL_W * 7, TBL_CELL_H)
  tblHeaders.forEach((h, hi) => {
    ctx.strokeStyle = borderColor; ctx.lineWidth = 0.5
    ctx.strokeRect(TBL + hi*TBL_CELL_W, TBT + 28, TBL_CELL_W, TBL_CELL_H)
    ctx.fillStyle = textColor; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
    ctx.fillText(h, TBL + hi*TBL_CELL_W + TBL_CELL_W/2, TBT + 28 + TBL_CELL_H/2)
  })
  ctx.textBaseline = 'alphabetic'

  const tblTimeLabels = ['T0/4','T1/2','T2/2','T0/2','T0/2','T2/2','T1/2','T0/4']
  const allSeqs: Record<number, string[]> = {
    1:['V0','V4','V6','V7','V7','V6','V4','V0'],
    2:['V0','V6','V2','V7','V7','V2','V6','V0'],
    3:['V0','V2','V3','V7','V7','V3','V2','V0'],
    4:['V0','V3','V1','V7','V7','V1','V3','V0'],
    5:['V0','V1','V5','V7','V7','V5','V1','V0'],
    6:['V0','V5','V4','V7','V7','V4','V5','V0'],
  }

  const TBL_ROW_START = TBT + 56
  tblTimeLabels.forEach((tl, row) => {
    ctx.fillStyle = '#f8fafc'; ctx.fillRect(TBL, TBL_ROW_START + row*TBL_CELL_H, TBL_CELL_W, TBL_CELL_H)
    ctx.strokeStyle = borderColor; ctx.lineWidth = 0.5
    ctx.strokeRect(TBL, TBL_ROW_START + row*TBL_CELL_H, TBL_CELL_W, TBL_CELL_H)
    ctx.fillStyle = COL_TEXT2; ctx.font = '9px monospace'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
    ctx.fillText(tl, TBL + TBL_CELL_W/2, TBL_ROW_START + row*TBL_CELL_H + TBL_CELL_H/2)

    for (let col = 1; col <= 6; col++) {
      const vecName = allSeqs[col][row]
      const isCurSector = col === sector
      ctx.fillStyle = isCurSector ? 'rgba(59,130,246,0.15)' : '#fff'
      ctx.fillRect(TBL + col*TBL_CELL_W, TBL_ROW_START + row*TBL_CELL_H, TBL_CELL_W, TBL_CELL_H)
      ctx.strokeStyle = borderColor; ctx.lineWidth = isCurSector ? 1 : 0.5
      ctx.strokeRect(TBL + col*TBL_CELL_W, TBL_ROW_START + row*TBL_CELL_H, TBL_CELL_W, TBL_CELL_H)

      let txtColor = textColor
      if (vecName === 'V0') txtColor = '#ef4444'
      else if (vecName === 'V7') txtColor = '#f59e0b'
      else if (isCurSector) txtColor = primaryColor

      ctx.fillStyle = txtColor; ctx.font = `${isCurSector ? 'bold' : 'normal'} 9px sans-serif`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
      ctx.fillText(vecName, TBL + col*TBL_CELL_W + TBL_CELL_W/2, TBL_ROW_START + row*TBL_CELL_H + TBL_CELL_H/2)
    }
  })
  ctx.textBaseline = 'alphabetic'

  const seqInfo = switchSequenceTable.value
  ctx.fillStyle = primaryColor; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText(`当前扇区 ${sector}: ${seqInfo.firstVec}(Tfirst) + ${seqInfo.secondVec}(Tsecond)  →  ${seqInfo.order.join(' → ')}`, TBL, TBL_ROW_START + 8*TBL_CELL_H + 14)
  ctx.fillStyle = '#f59e0b'; ctx.font = 'bold 9px sans-serif'
  ctx.fillText('★ 图16: 七段式中心对称PWM → 电流纹波Δi=V·Δt/L，Δt减半→纹波≈½ (对称PWM纹波仅为非对称的~50%，转矩脉动和噪音显著减小)', TBL, TBL_ROW_START + 8*TBL_CELL_H + 30)
}

let cumAngle = vrefAngle.value

const animate = (ts: number) => {
  if (autoRotate.value && lastTs > 0) {
    const dt = Math.min((ts - lastTs) / 1000, 0.1)
    cumAngle += rotateSpeed.value * dt
    vrefAngle.value = ((cumAngle % 360) + 360) % 360
    trailAngles.value = [cumAngle, ...trailAngles.value.slice(0, TRAIL_LEN - 1)]
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
      <h4>SVPWM 矢量合成仿真（参考知乎：SVPWM理论与实践）</h4>
      <div class="header-controls">
        <button class="auto-btn" :class="{ active: autoRotate }" @click="autoRotate = !autoRotate">
          {{ autoRotate ? '⏸ 暂停旋转' : '▶ 自动旋转' }}
        </button>
        <div class="mode-selector">
          <button :class="{ active: svmMode === 'seven' }" @click="svmMode = 'seven'">七段式</button>
          <button :class="{ active: svmMode === 'five' }" @click="svmMode = 'five'">五段式</button>
        </div>
      </div>
    </div>

    <canvas ref="canvasRef" class="sim-canvas"></canvas>

    <div class="sim-params">
      <div class="param-group">
        <label>母线电压 Vdc: <span class="param-val">{{ Vdc }} V</span></label>
        <input type="range" v-model.number="Vdc" :min="24" :max="400" :step="1" />
        <span class="param-range">24 ~ 400 V</span>
      </div>
      <div class="param-group">
        <label>极对数 p: <span class="param-val">{{ polePairs }}</span></label>
        <input type="range" v-model.number="polePairs" :min="1" :max="8" :step="1" />
        <span class="param-range">1 ~ 8</span>
      </div>
      <div class="param-group">
        <label>Vref 幅值: <span class="param-val">{{ vrefMag.toFixed(1) }} V</span></label>
        <input type="range" v-model.number="vrefMag" :min="0" :max="vrefMagMax" :step="0.5" />
        <span class="param-range">0 ~ {{ vrefMagMax.toFixed(1) }} V</span>
      </div>
      <div class="param-group">
        <label>电气角度 θe: <span class="param-val">{{ vrefAngle.toFixed(1) }}°</span></label>
        <input type="range" v-model.number="vrefAngle" :min="0" :max="360" :step="0.5" />
        <span class="param-range">0° ~ 360° (机械: {{ (vrefAngle / polePairs).toFixed(1) }}°)</span>
      </div>
      <div class="param-group" v-if="autoRotate">
        <label>旋转速度: <span class="param-val">{{ rotateSpeed }}°/s</span></label>
        <input type="range" v-model.number="rotateSpeed" :min="1" :max="720" :step="1" />
        <span class="param-range">1 ~ 720 °/s</span>
      </div>
      <div class="param-group">
        <label>PWM频率: <span class="param-val">{{ pwmFreq }} kHz</span></label>
        <input type="range" v-model.number="pwmFreq" :min="1" :max="20" :step="1" />
        <span class="param-range">1 ~ 20 kHz (Ts={{ (1000/pwmFreq).toFixed(0) }}μs)</span>
      </div>
      <div class="param-group">
        <label>死区时间 Tdead: <span class="param-val">{{ deadTimeNs >= 1000 ? (deadTimeNs/1000).toFixed(1)+'μs' : deadTimeNs+'ns' }}</span></label>
        <input type="range" v-model.number="deadTimeNs" :min="0" :max="3000" :step="50" />
        <span class="param-range">0 ~ 3000 ns</span>
      </div>
    </div>

    <div class="sim-info">
      <div class="info-block">
        <div class="info-block-title">扇区判定 N = 4C + 2B + A</div>
        <div class="info-row"><span class="info-label">Vref1=uβ:</span><span class="info-value" :class="{ pos: nValue.A === 1 }">{{ nValue.A === 1 ? '>0' : '≤0' }}</span><span class="info-label ml">A=</span><span class="info-value highlight">{{ nValue.A }}</span></div>
        <div class="info-row"><span class="info-label">Vref2:</span><span class="info-value" :class="{ pos: nValue.B === 1 }">{{ nValue.B === 1 ? '>0' : '≤0' }}</span><span class="info-label ml">B=</span><span class="info-value highlight">{{ nValue.B }}</span></div>
        <div class="info-row"><span class="info-label">Vref3:</span><span class="info-value" :class="{ pos: nValue.C === 1 }">{{ nValue.C === 1 ? '>0' : '≤0' }}</span><span class="info-label ml">C=</span><span class="info-value highlight">{{ nValue.C }}</span></div>
        <div class="info-row"><span class="info-label">N:</span><span class="info-value highlight-big">{{ nValue.N }}</span><span class="info-label">→扇区:</span><span class="info-value highlight-big">{{ getSector(vrefAngle) }}</span></div>
      </div>
      <div class="info-block">
        <div class="info-block-title">X/Y/Z &amp; Tcm 切换时间</div>
        <div class="info-row"><span class="info-label">X:</span><span class="info-value">{{ (xyzVars.X * 100).toFixed(1) }}%</span></div>
        <div class="info-row"><span class="info-label">Y:</span><span class="info-value">{{ (xyzVars.Y * 100).toFixed(1) }}%</span></div>
        <div class="info-row"><span class="info-label">Z:</span><span class="info-value">{{ (xyzVars.Z * 100).toFixed(1) }}%</span></div>
        <div class="info-row"><span class="info-label">Tcm1:</span><span class="info-value highlight">{{ (tcmTimes.Tcm1 * 200).toFixed(1) }}%Ts</span></div>
        <div class="info-row"><span class="info-label">Tcm2:</span><span class="info-value highlight">{{ (tcmTimes.Tcm2 * 200).toFixed(1) }}%Ts</span></div>
        <div class="info-row"><span class="info-label">Tcm3:</span><span class="info-value highlight">{{ (tcmTimes.Tcm3 * 200).toFixed(1) }}%Ts</span></div>
      </div>
      <div class="info-block">
        <div class="info-block-title">基本参数 &amp; 调制比</div>
        <div class="info-row"><span class="info-label">Vdc:</span><span class="info-value">{{ Vdc }} V</span></div>
        <div class="info-row"><span class="info-label">极对数 p:</span><span class="info-value">{{ polePairs }}</span></div>
        <div class="info-row"><span class="info-label">机械角度:</span><span class="info-value highlight">{{ (vrefAngle / polePairs).toFixed(1) }}°</span></div>
        <div class="info-row"><span class="info-label">活跃矢量:</span><span class="info-value">{{ getActiveVectors(vrefAngle).v1 }}+{{ getActiveVectors(vrefAngle).v2 }}</span></div>
        <div class="info-row"><span class="info-label">m(SVPWM):</span><span class="info-value highlight">{{ calcTimes(vrefMag, vrefAngle).m.toFixed(3) }}</span><span class="info-note">≤1线性</span></div>
        <div class="info-row"><span class="info-label">ma(SPWM):</span><span class="info-value">{{ phaseVoltages.ma.toFixed(3) }}</span><span class="info-note">=m×2/√3</span></div>
        <div class="info-row"><span class="info-label">T1/T2/T0:</span><span class="info-value">{{ (calcTimes(vrefMag,vrefAngle).T1*100).toFixed(0) }}/{{ (calcTimes(vrefMag,vrefAngle).T2*100).toFixed(0) }}/{{ (calcTimes(vrefMag,vrefAngle).T0*100).toFixed(0) }}%</span></div>
      </div>
    </div>

    <div class="explain-panel">
      <details open>
        <summary>SVPWM 完整算法流程 &amp; 关键图表说明</summary>
        <div class="explain-content">
          <table class="compare-table">
            <thead><tr><th>特性</th><th>七段式</th><th>五段式</th></tr></thead>
            <tbody>
              <tr><td>每周期段数</td><td>7 (V0→V1→V2→V7→V2→V1→V0)</td><td>5</td></tr>
              <tr><td>零矢量分布</td><td>V0+V7 对称(图13)</td><td>交替舍弃</td></tr>
              <tr><td>PWM对称性</td><td>完全中心对称(图15)</td><td>中心对称</td></tr>
              <tr><td>电流纹波(图16)</td><td>最小(Δt减半)</td><td>较大(~1.5×)</td></tr>
              <tr><td>开关损耗</td><td>较高(每相2次)</td><td>降低~33%</td></tr>
            </tbody>
          </table>
          <p><strong>图5/12 矢量合成区：</strong>六边形顶点为6个非零基本矢量(V₁~V₆)，中心为零矢量(V₀/V₇)。绿色虚线=内切圆(m≤1线性区)，橙色虚线=外接圆(m≤1.155过调制区)。Vref为待合成的参考矢量，通过相邻两矢量投影分解得到作用时间。</p>
          <p><strong>图8/9 三相正弦：</strong>三相电压uₐ/u_b/u_c互差120°，合成空间矢量Vδ幅值为相电压1.5倍，以电角速度ωₑ旋转。</p>
          <p><strong>图22/25 三角波载波比较法：</strong>三角波峰值=Ts/2，Tcmp1/Tcmp2/Tcmp3为三相开关切换时间。当三角波值 > TcmpX时对应相PWM输出高电平(上管导通)，否则低电平。下方完整显示各时段的时间标签、矢量名和开关状态。</p>
          <p><strong>表2/图14 开关顺序：</strong>6扇区各有独立切换顺序(0-4-6-7-6-4-0等)，原则是每次只变一相开关状态。当前扇区高亮显示完整 8 步序列。</p>
          <p><strong>图16 对称性优势：</strong>七段式将零矢量V₇置于中间→相电压关于Ts/2对称→电流纹波Δi∝V·Δt/L中Δt减半→纹波≈½→转矩脉动和噪音显著减小。</p>
        <p><strong>死区时间 (Dead-Time)：</strong>为防止上下桥臂直通(shoot-through)，上管关断后需延时Tdead才开通下管(反之亦然)。导致实际输出电压损失 = Vdc × (Tdead/Ts)，低速时畸变显著需补偿。图中红色影区=上升沿被延迟的份额。</p>
        </div>
      </details>
    </div>
    <div class="explain-panel">
      <details>
        <summary>📋 电控知识体系中缺失的关键内容（建议后续补充）</summary>
        <div class="explain-content">
          <table class="compare-table">
            <thead><tr><th>#</th><th>缺失内容</th><th>为何不可或缺</th></tr></thead>
            <tbody>
              <tr><td>1</td><td><strong>Clarke/Park 坐标变换</strong> abc→αβ→dq</td><td>FOC 核心：将三相交流量变换为旋转坐标系直流分量，是电流环 PI 控制的前提</td></tr>
              <tr><td>2</td><td><strong>电流采样时序</strong></td><td>ADC 在 PWM 周期中心(V0段)采样可避开开关噪声，采样误差直接影响控制精度</td></tr>
              <tr><td>3</td><td><strong>过调制策略</strong> (m&gt;1.155)</td><td>六边形顶点以外区域需电压钳位/角度跟随，影响弱磁区最大输出转矩</td></tr>
              <tr><td>4</td><td><strong>PI 电流调节器</strong> dq电流环设计</td><td>解耦控制 + 前馈补偿 + anti-windup，是 FOC 内环核心</td></tr>
              <tr><td>5</td><td><strong>位置/速度观测器</strong> (SMO/Luenberger)</td><td>无传感器 FOC 必需：从电流/电压估算反电势→转子位置→速度</td></tr>
              <tr><td>6</td><td><strong>弱磁控制 + MTPA/MTPV</strong></td><td>高速区利用负Id弱磁扩展转速范围，MTPA最大化转矩/电流比</td></tr>
              <tr><td>7</td><td><strong>THD 谐波分析</strong> SVPWM vs SPWM</td><td>量化调制策略的电流波形质量，影响铁损和转矩脉动评估</td></tr>
              <tr><td>8</td><td><strong>启动定位/预定位</strong></td><td>初始转子位置检测(高频注入/IPD)，影响启动转矩和反转风险</td></tr>
              <tr><td>9</td><td><strong>速度环 + 负载转矩观测器</strong></td><td>外环速度控制 + 负载扰动前馈补偿，提升动态刚度</td></tr>
              <tr><td>10</td><td><strong>死区补偿策略</strong></td><td>基于电流极性判断的电压前馈补偿，消除低速电流钳位和转矩脉动</td></tr>
            </tbody>
          </table>
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
.info-block { display: flex; flex-direction: column; gap: 0.15rem; min-width: 185px; max-width: 300px; }
.info-block-title { font-weight: 700; color: var(--primary-color); margin-bottom: 0.12rem; border-bottom: 1px solid var(--border-color); padding-bottom: 0.12rem; font-size: 0.8rem; }
.info-row { display: flex; gap: 0.2rem; align-items: center; flex-wrap: wrap; }
.info-label { font-weight: 600; color: #0ea5e9; white-space: nowrap; font-size: 0.75rem; }
.info-label.ml { margin-left: 0.25rem; }
.info-value { color: var(--text-color); font-family: 'Consolas','Monaco',monospace; font-size: 0.72rem; }
.info-value.highlight { color: var(--primary-color); font-weight: 700; }
.info-value.highlight-big { color: var(--primary-color); font-weight: 700; font-size: 0.88rem; }
.info-value.pos { color: #10b981; font-weight: 600; }
.info-note { font-size: 0.62rem; color: #0ea5e9; font-style: italic; }
.explain-panel { margin-top: 0.6rem; padding: 0.55rem 0.7rem; background: var(--content-bg); border: 1px solid var(--border-color); border-radius: 6px; font-size: 0.76rem; }
.explain-panel details summary { font-weight: 600; color: var(--primary-color); cursor: pointer; font-size: 0.82rem; }
.explain-panel .explain-content { margin-top: 0.45rem; }
.compare-table { width: 100%; border-collapse: collapse; margin: 0.4rem 0; font-size: 0.74rem; }
.compare-table th, .compare-table td { border: 1px solid var(--border-color); padding: 0.25rem 0.55rem; text-align: left; }
.compare-table th { background: var(--code-bg, #f1f5f9); }
.compare-table td:first-child { font-weight: 500; }
.explain-content p { margin: 0.35rem 0; color: var(--text-secondary); line-height: 1.45; }
</style>
