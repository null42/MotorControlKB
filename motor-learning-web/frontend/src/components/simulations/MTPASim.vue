<script setup lang="ts">
import { ref, computed, watch, onMounted, onUnmounted } from 'vue'

/**
 * MTPA 与弱磁控制仿真
 * 展示 Id-Iq 电流平面上的 MTPA 轨迹、电流/电压约束、弱磁区间
 *
 * Canvas 920×520，左右两栏:
 *   左栏 (0~445): Id-Iq 电流平面 — 恒转矩曲线, 电流极限圆, 电压极限椭圆, MTPA/MTPV 轨迹
 *   右栏 (450~910): 转矩-速度包络线 — 恒转矩 → 恒功率(FW) → MTPV 三区
 */

const canvasRef = ref<HTMLCanvasElement | null>(null)
let animFrameId = 0

// --- 电机参数 ---
const ld = ref(5)           // d 轴电感 mH
const lq = ref(8)           // q 轴电感 mH
const fluxLinkage = ref(0.08) // 永磁磁链 ψf Wb
const ratedCurrent = ref(10)  // 额定电流 A
const vdc = ref(100)         // 母线电压 V

// --- 工作点 ---
const speedRatio = ref(1.0)  // 转速/基速比 (0~2)
const torqueCmd = ref(5.0)   // 转矩指令 Nm

// --- 仿真数据 ---
const showMTPA = ref(true)
const showFW = ref(true)

// 计算基速
const baseSpeed = computed(() => {
  // ω_base = Vmax / sqrt((ψf + Ld*Id)^2 + (Lq*Iq)^2)
  // 简化: ω_base = Vmax / ψf (近似)
  const vmax = vdc.value / Math.sqrt(3)
  return (vmax / fluxLinkage.value) * (60 / (2 * Math.PI))
})

// 当前转速
const currentSpeed = computed(() => speedRatio.value * baseSpeed.value)

// 电流极限圆半径
const IsMax = computed(() => ratedCurrent.value)

// 电压极限椭圆参数 (随转速收缩)
const voltageEllipseData = computed(() => {
  const omegaE = speedRatio.value * (vdc.value / Math.sqrt(3)) / fluxLinkage.value
  const vmax = vdc.value / Math.sqrt(3)
  // 电压极限方程: (ψf + Ld·id)^2 + (Lq·iq)^2 = (Vmax/ωe)^2
  const a = vmax / (omegaE * ld.value * 1e-3 + 1e-9)  // d 轴截距
  const b = vmax / (omegaE * lq.value * 1e-3 + 1e-9)  // q 轴截距
  return { a, b, centerX: -fluxLinkage.value / (ld.value * 1e-3 + 1e-9), centerY: 0 }
})

// MTPA 轨迹点计算 (拉格朗日乘子法)
const mtpaPoints = computed(() => {
  const pts: { id: number; iq: number }[] = []
  const Ld = ld.value * 1e-3, Lq = lq.value * 1e-3
  const psiF = fluxLinkage.value
  const deltaL = Lq - Ld  // IPM: Lq > Ld

  for (let iq = 0; iq <= IsMax.value; iq += 0.2) {
    let id: number
    if (deltaL < 0.001) {
      // SPM: MTPA = Id=0
      id = 0
    } else {
      // MTPA: Id = ψf/(2(Lq-Ld)) - sqrt(ψf²/(4(Lq-Ld)²) + Iq²)
      const a = psiF / (2 * deltaL)
      id = a - Math.sqrt(a * a + iq * iq)
    }
    // 电流约束
    if (id * id + iq * iq > IsMax.value * IsMax.value) break
    pts.push({ id, iq })
  }
  return pts
})

// 转矩计算
function calcTorque(id: number, iq: number): number {
  const Ld = ld.value * 1e-3, Lq = lq.value * 1e-3
  const psiF = fluxLinkage.value
  // Te = 3/2 * p * (ψf·iq + (Ld-Lq)·id·iq)
  return (3 / 2) * 1 * (psiF * iq + (Ld - Lq) * id * iq)
}

// 恒转矩等值线
const torqueContours = computed(() => {
  const levels = [2, 4, 6, 8, 10, 12, 15]
  const Ld = ld.value * 1e-3, Lq = lq.value * 1e-3
  const psiF = fluxLinkage.value
  const dL = Ld - Lq
  const isSPM = Math.abs(dL) < 1e-7
  return levels.map(tLevel => {
    const pts: { id: number; iq: number }[] = []
    for (let iq = 0; iq <= IsMax.value + 2; iq += 0.15) {
      if (iq < 0.01) continue
      const id = isSPM ? 0 : (tLevel / 1.5 - psiF * iq) / (dL * iq)
      if (id > 0) continue
      if (id * id + iq * iq > (IsMax.value + 4) * (IsMax.value + 4)) continue
      if (id < -IsMax.value * 1.5) continue
      pts.push({ id, iq })
    }
    return { level: tLevel, pts }
  })
})

// T-ω 包络线
const torqueSpeedEnvelope = computed(() => {
  const pts: { speed: number; torque: number }[] = []
  const baseRPM = baseSpeed.value

  // 恒转矩区 (speed 0→base)
  const maxTorque = calcTorque(
    mtpaPoints.value.length > 0 ? mtpaPoints.value[mtpaPoints.value.length - 1].id : 0,
    mtpaPoints.value.length > 0 ? mtpaPoints.value[mtpaPoints.value.length - 1].iq : IsMax.value
  )

  for (let i = 0; i <= 30; i++) {
    const ratio = i / 30
    pts.push({ speed: ratio * baseRPM * 0.95, torque: maxTorque })
  }

  // 恒功率区 (弱磁: base→2*base)
  for (let i = 1; i <= 30; i++) {
    const ratio = 1 + i / 30
    const speed = ratio * baseRPM * 0.95
    // 功率恒定: T ∝ 1/ω
    const torque = maxTorque * baseRPM * 0.95 / speed
    pts.push({ speed, torque: Math.max(torque, maxTorque * 0.15) })
  }

  return pts
})

const operatingMode = computed(() => {
  if (speedRatio.value <= 0.98) return 'MTPA (恒转矩区)'
  if (speedRatio.value <= 1.5) return 'FW (弱磁 / 恒功率)'
  return 'MTPV (最大转矩/电压)'
})

const voltageMargin = computed(() => {
  const omegaE = speedRatio.value * (vdc.value / Math.sqrt(3)) / (fluxLinkage.value + 1e-9)
  const vmax = vdc.value / Math.sqrt(3)
  // 简化: margin = 1 - |v|/vmax
  const margin = Math.max(0, (1 - speedRatio.value) * 100)
  return margin
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

  // 垂直分隔线 x=445
  const VSEP = 445
  ctx.strokeStyle = borderColor; ctx.lineWidth = 1; ctx.setLineDash([6, 4])
  ctx.beginPath(); ctx.moveTo(VSEP, 10); ctx.lineTo(VSEP, H - 10); ctx.stroke()
  ctx.setLineDash([])

  // ═════════════════════ 左栏: Id-Iq 电流平面 ═════════════════════
  const L_CX = 222, L_CY = 270, L_SCALE = 14
  const toX = (id: number) => L_CX + id * L_SCALE
  const toY = (iq: number) => L_CY - iq * L_SCALE

  // 标题
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('Id-Iq 电流平面 (MTPA & 弱磁)', L_CX, 22)

  // 坐标轴
  ctx.strokeStyle = textSec; ctx.lineWidth = 0.8
  ctx.beginPath(); ctx.moveTo(L_CX - 170, L_CY); ctx.lineTo(L_CX + 170, L_CY); ctx.stroke()
  ctx.beginPath(); ctx.moveTo(L_CX, L_CY - 170); ctx.lineTo(L_CX, L_CY + 170); ctx.stroke()
  ctx.fillStyle = textSec; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('Id (← 负向励磁)', L_CX - 80, L_CY - 8)
  ctx.fillText('Iq (↑ 转矩)', L_CX + 6, L_CY - 155)

  // 电流极限圆
  ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 1.8; ctx.setLineDash([2, 3])
  ctx.beginPath(); ctx.arc(L_CX, L_CY, IsMax.value * L_SCALE, 0, 2 * Math.PI); ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#ef4444'; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText(`Is,max=${IsMax.value}A`, L_CX + IsMax.value * L_SCALE + 4, L_CY - 4)

  // 电压极限椭圆 (随转速收缩)
  const vel = voltageEllipseData.value
  if (speedRatio.value > 0.3) {
    const eCX = L_CX + vel.centerX * L_SCALE, eCY = L_CY
    const eRX = vel.a * L_SCALE, eRY = vel.b * L_SCALE
    ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 1.5; ctx.setLineDash([5, 3])
    ctx.beginPath()
    for (let ang = 0; ang <= 2 * Math.PI; ang += 0.05) {
      const ex = eCX + eRX * Math.cos(ang)
      const ey = eCY - eRY * Math.sin(ang)
      if (ang === 0) ctx.moveTo(ex, ey); else ctx.lineTo(ex, ey)
    }
    ctx.closePath(); ctx.stroke()
    ctx.setLineDash([])
    ctx.fillStyle = '#f59e0b'; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'left'
    ctx.fillText(`Vlim @ ${speedRatio.value.toFixed(1)}×base`, eCX + eRX + 2, eCY - eRY * 0.5)
  }

  // 恒转矩曲线
  torqueContours.value.forEach(tc => {
    if (tc.pts.length < 2) return
    ctx.strokeStyle = '#e0e7ff'; ctx.lineWidth = 0.7
    ctx.beginPath()
    tc.pts.forEach((p, i) => {
      const x = toX(p.id), y = toY(p.iq)
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
    })
    ctx.stroke()
    // 标注
    const mid = tc.pts[Math.floor(tc.pts.length / 2)]
    const mx = toX(mid.id), my = toY(mid.iq)
    ctx.fillStyle = textSec; ctx.font = 'bold 8px sans-serif'; ctx.textAlign = 'center'
    ctx.fillText(`${tc.level}Nm`, mx - 14, my + 4)
  })

  // MTPA 轨迹
  const mtpa = mtpaPoints.value
  if (mtpa.length > 1 && showMTPA.value) {
    ctx.strokeStyle = '#10b981'; ctx.lineWidth = 2.2
    ctx.beginPath()
    mtpa.forEach((p, i) => {
      const x = toX(p.id), y = toY(p.iq)
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
    })
    ctx.stroke()
    ctx.fillStyle = '#10b981'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'left'
    ctx.fillText('MTPA', toX(mtpa[Math.floor(mtpa.length / 2)].id) - 42, toY(mtpa[Math.floor(mtpa.length / 2)].iq))
  }

  // 当前工作点
  let opIq = Math.min(torqueCmd.value, IsMax.value * 0.9)
  let opId = 0
  if (mtpa.length > 1) {
    let bestDist = Infinity
    for (const p of mtpa) {
      const t = calcTorque(p.id, p.iq)
      const dist = Math.abs(t - torqueCmd.value)
      if (dist < bestDist) { bestDist = dist; opId = p.id; opIq = p.iq }
    }
  }

  const opX = toX(opId), opY = toY(opIq)
  ctx.fillStyle = '#ef4444'; ctx.beginPath(); ctx.arc(opX, opY, 5, 0, 2 * Math.PI); ctx.fill()
  ctx.strokeStyle = '#fff'; ctx.lineWidth = 1
  ctx.beginPath(); ctx.arc(opX, opY, 5, 0, 2 * Math.PI); ctx.stroke()
  ctx.fillStyle = '#ef4444'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText(`工作点 (${opId.toFixed(1)},${opIq.toFixed(1)})`, opX + 10, opY - 6)

  // 原点
  ctx.fillStyle = textColor; ctx.beginPath(); ctx.arc(L_CX, L_CY, 3, 0, 2 * Math.PI); ctx.fill()

  // 图例框
  ctx.fillStyle = contentBg; ctx.fillRect(12, H - 52, 195, 42)
  ctx.strokeStyle = borderColor; ctx.lineWidth = 0.6; ctx.strokeRect(12, H - 52, 195, 42)
  ctx.font = 'bold 8px sans-serif'; ctx.textAlign = 'left'
  ctx.fillStyle = '#ef4444'; ctx.fillText('── 电流极限圆', 18, H - 34)
  ctx.fillStyle = '#f59e0b'; ctx.fillText('--- 电压极限椭圆', 18, H - 22)
  ctx.fillStyle = '#10b981'; ctx.fillText('── MTPA 轨迹', 120, H - 34)
  ctx.fillText('Lq/Ld = ' + (lq.value / ld.value).toFixed(1), 120, H - 22)

  // ═════════════════════ 右栏: 转矩-速度包络 ═════════════════════
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('转矩-速度包络线 (T-ω)', (VSEP + W) / 2, 22)

  const R_L = 462, R_T = 38, R_W = 440, R_H = 270
  const R_B = R_T + R_H

  ctx.strokeStyle = borderColor; ctx.lineWidth = 0.6
  ctx.strokeRect(R_L, R_T, R_W, R_H)

  const env = torqueSpeedEnvelope.value
  const maxSpeed = baseSpeed.value * 1.9
  const maxTorque = env.length > 0 ? env[0].torque : 10

  const tToX = (sp: number) => R_L + 10 + (sp / maxSpeed) * (R_W - 20)
  const tToY = (trq: number) => R_B - 10 - (trq / maxTorque) * (R_H - 30)

  // 包络线
  ctx.strokeStyle = primaryColor; ctx.lineWidth = 2.5
  ctx.beginPath()
  env.forEach((p, i) => {
    const x = tToX(p.speed), y = tToY(p.torque)
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
  })
  ctx.stroke()

  // 填充区域
  ctx.fillStyle = 'rgba(59, 130, 246, 0.08)'
  ctx.beginPath()
  env.forEach((p, i) => {
    const x = tToX(p.speed), y = tToY(p.torque)
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
  })
  ctx.lineTo(tToX(maxSpeed), tToY(0))
  ctx.lineTo(tToX(0), tToY(0))
  ctx.closePath(); ctx.fill()

  // 分区标注
  const zone1End = R_L + 10 + (0.95 * baseSpeed.value / maxSpeed) * (R_W - 20)
  const zone2End = R_L + 10 + (1.8 * baseSpeed.value / maxSpeed) * (R_W - 20)

  ctx.fillStyle = 'rgba(16, 185, 129, 0.12)'
  ctx.fillRect(R_L + 10, R_T + 8, zone1End - R_L - 10, 18)
  ctx.fillStyle = '#10b981'; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('I: 恒转矩区', (R_L + 10 + zone1End) / 2, R_T + 22)

  ctx.fillStyle = 'rgba(245, 158, 11, 0.12)'
  ctx.fillRect(zone1End, R_T + 8, zone2End - zone1End, 18)
  ctx.fillStyle = '#f59e0b'; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('II: 恒功率 (弱磁)', (zone1End + zone2End) / 2, R_T + 22)

  ctx.fillStyle = 'rgba(239, 68, 68, 0.12)'
  ctx.fillRect(zone2End, R_T + 8, R_L + R_W - 10 - zone2End, 18)
  ctx.fillStyle = '#ef4444'; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('III: MTPV', (zone2End + R_L + R_W - 10) / 2, R_T + 22)

  // 当前工作点
  const wpX = tToX(currentSpeed.value)
  const wpY = tToY(torqueCmd.value)
  ctx.fillStyle = '#ef4444'; ctx.beginPath(); ctx.arc(wpX, Math.max(R_T + 10, Math.min(wpY, R_B - 10)), 5, 0, 2 * Math.PI); ctx.fill()
  ctx.strokeStyle = '#fff'; ctx.lineWidth = 1
  ctx.beginPath(); ctx.arc(wpX, Math.max(R_T + 10, Math.min(wpY, R_B - 10)), 5, 0, 2 * Math.PI); ctx.stroke()

  // 基速标记
  ctx.strokeStyle = '#94a3b8'; ctx.lineWidth = 0.8; ctx.setLineDash([3, 4])
  ctx.beginPath()
  ctx.moveTo(tToX(baseSpeed.value), R_T + 28)
  ctx.lineTo(tToX(baseSpeed.value), R_B - 10)
  ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = textSec; ctx.font = 'bold 8px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('基速', tToX(baseSpeed.value), R_T + 40)

  // 坐标轴标注
  ctx.fillStyle = textSec; ctx.font = '9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('0', R_L + 4, R_B - 6)
  ctx.fillText(`${(baseSpeed.value * 2).toFixed(0)} RPM`, R_L + R_W - 8, R_B - 6)
  ctx.fillText('转速 (RPM)', R_L + R_W / 2, R_B + 16)

  // 下方指标
  const infoY = R_B + 32
  const mItems = [
    { label: 'Id_ref', val: `${opId.toFixed(2)} A`, c: '#ec4899' },
    { label: 'Iq_ref', val: `${opIq.toFixed(2)} A`, c: '#06b6d4' },
    { label: '运行模式', val: operatingMode.value, c: primaryColor },
    { label: '电压裕量', val: `${voltageMargin.value.toFixed(0)}%`, c: '#10b981' },
  ]

  const cardW2 = 200, cardH2 = 46, cardGap2 = 8
  const csX = R_L + (R_W - mItems.length * (cardW2 + cardGap2) + cardGap2) / 2

  mItems.forEach((mi, i) => {
    const ix = csX + i * (cardW2 + cardGap2)
    ctx.fillStyle = contentBg; ctx.fillRect(ix, infoY, cardW2, cardH2)
    ctx.strokeStyle = borderColor; ctx.lineWidth = 0.6; ctx.strokeRect(ix, infoY, cardW2, cardH2)
    ctx.fillStyle = textSec; ctx.font = '9px sans-serif'; ctx.textAlign = 'center'
    ctx.fillText(mi.label, ix + cardW2 / 2, infoY + 14)
    ctx.fillStyle = mi.c; ctx.font = 'bold 13px monospace'
    ctx.fillText(mi.val, ix + cardW2 / 2, infoY + 36)
  })
}

// ═════════════════════════ 动画循环 ═════════════════════════
const animate = (ts: number) => {
  try { draw() } catch { /* keep loop alive */ }
  animFrameId = requestAnimationFrame(animate)
}

onMounted(() => { animFrameId = requestAnimationFrame(animate) })
onUnmounted(() => cancelAnimationFrame(animFrameId))
</script>

<template>
  <div class="sim-container">
    <div class="sim-header">
      <h4>MTPA &amp; 弱磁控制仿真 -- Id-Iq 电流平面与转矩-速度包络</h4>
    </div>

    <canvas ref="canvasRef" class="sim-canvas"></canvas>

    <div class="sim-params">
      <div class="param-group">
        <label>Ld (d轴电感): <span class="param-val">{{ ld }} mH</span></label>
        <input type="range" v-model.number="ld" :min="1" :max="15" :step="0.5" />
        <span class="param-range">1 ~ 15 mH</span>
      </div>
      <div class="param-group">
        <label>Lq (q轴电感): <span class="param-val">{{ lq }} mH</span></label>
        <input type="range" v-model.number="lq" :min="1" :max="20" :step="0.5" />
        <span class="param-range">1 ~ 20 mH</span>
      </div>
      <div class="param-group">
        <label>凸极比 Lq/Ld: <span class="param-val">{{ (lq / ld).toFixed(2) }}</span></label>
        <input type="range" v-model.number="lq" :min="ld" :max="20" :step="0.5" />
        <span class="param-range">1.0 (SPM) ~ >1 (IPM)</span>
      </div>
      <div class="param-group">
        <label>&#968;f 磁链: <span class="param-val">{{ fluxLinkage }} Wb</span></label>
        <input type="range" v-model.number="fluxLinkage" :min="0.02" :max="0.2" :step="0.005" />
        <span class="param-range">0.02 ~ 0.2 Wb</span>
      </div>
      <div class="param-group">
        <label>转速/基速比: <span class="param-val">{{ speedRatio.toFixed(2) }}×</span></label>
        <input type="range" v-model.number="speedRatio" :min="0.1" :max="2.0" :step="0.02" />
        <span class="param-range">0.1 ~ 2.0 (基速={{ baseSpeed.toFixed(0) }} RPM)</span>
      </div>
      <div class="param-group">
        <label>转矩指令: <span class="param-val">{{ torqueCmd }} Nm</span></label>
        <input type="range" v-model.number="torqueCmd" :min="0.5" :max="15" :step="0.5" />
        <span class="param-range">0.5 ~ 15 Nm</span>
      </div>
    </div>

    <div class="sim-info">
      <div class="info-block">
        <div class="info-block-title">电机参数</div>
        <div class="info-row"><span class="info-label">Ld:</span><span class="info-value">{{ ld }} mH</span></div>
        <div class="info-row"><span class="info-label">Lq:</span><span class="info-value">{{ lq }} mH</span></div>
        <div class="info-row"><span class="info-label">凸极率:</span><span class="info-value highlight">{{ (lq / ld).toFixed(2) }}</span></div>
        <div class="info-row"><span class="info-label">&#968;f:</span><span class="info-value">{{ fluxLinkage }} Wb</span></div>
      </div>
      <div class="info-block">
        <div class="info-block-title">运行状态</div>
        <div class="info-row"><span class="info-label">转速:</span><span class="info-value highlight">{{ currentSpeed.toFixed(0) }} RPM</span></div>
        <div class="info-row"><span class="info-label">基速:</span><span class="info-value">{{ baseSpeed.toFixed(0) }} RPM</span></div>
        <div class="info-row"><span class="info-label">模式:</span><span class="info-value highlight">{{ operatingMode }}</span></div>
        <div class="info-row"><span class="info-label">电压裕量:</span><span class="info-value" :style="{ color: voltageMargin > 10 ? '#10b981' : '#ef4444' }">{{ voltageMargin.toFixed(0) }}%</span></div>
      </div>
      <div class="info-block">
        <div class="info-block-title">弱磁原理</div>
        <div class="info-row"><span class="info-label">Vmax:</span><span class="info-value">{{ (vdc / Math.sqrt(3)).toFixed(0) }} V</span></div>
        <div class="info-row"><span class="info-label">Is,max:</span><span class="info-value">{{ ratedCurrent }} A</span></div>
        <div class="info-row"><span class="info-label">Vlim 椭圆:</span><span class="info-value" style="color:#f59e0b">随 &#969; 收缩</span></div>
      </div>
    </div>

    <div class="explain-panel">
      <details open>
        <summary>MTPA / 弱磁 / MTPV 控制原理</summary>
        <div class="explain-content">
          <p><strong>MTPA (最大转矩/电流比)：</strong>在电流极限圆内，选择 Id-Iq 组合使单位电流产生最大转矩。对于 IPM (Lq &gt; Ld)，Id 需取负值以利用磁阻转矩。MTPA 轨迹是恒转矩双曲线与电流极限圆的切点连线。</p>
          <p><strong>弱磁 (Flux Weakening)：</strong>当反电动势达到电压极限后，通过施加负 Id 削弱气隙磁通，使转速继续升高。电压极限椭圆随转速增大向 Id 负方向收缩。</p>
          <p><strong>MTPV (最大转矩/电压比)：</strong>深度弱磁区，工作点沿电压极限椭圆移动，在给定电压下最大化输出转矩。</p>
          <p><strong>SPM vs IPM：</strong>SPM (Ld=Lq) 无磁阻转矩，MTPA 即 Id=0。IPM (Lq&gt;Ld) 的凸极率越大，MTPA 轨迹越偏离 Id=0 轴，弱磁调速范围越宽。</p>
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