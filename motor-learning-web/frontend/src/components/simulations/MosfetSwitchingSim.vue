<script setup lang="ts">
import { ref, computed, watch, onMounted, nextTick } from 'vue'

// ── Canvas ref ────────────────────────────────────────────────────
const canvasRef = ref<HTMLCanvasElement | null>(null)

// ── Adjustable parameters (sliders) ───────────────────────────────
const Rg    = ref(10)    // Gate resistance, Ω
const Vdrive = ref(12)   // Gate drive voltage, V
const Id    = ref(10)    // Drain current, A
const Ciss  = ref(2)     // Input capacitance, nF

// ── Fixed MOSFET parameters ───────────────────────────────────────
const Vth     = 3       // Threshold voltage, V
const gm      = 20      // Transconductance, S
const Qgd     = 20e-9   // Gate-drain charge, C
const Rdson   = 0.01    // On-resistance, Ω
const Vbus    = 48      // Bus voltage, V

// ── Derived quantities ────────────────────────────────────────────
const tau      = computed(() => Rg.value * Ciss.value * 1e-9)        // Rg*Ciss, seconds
const Vplateau = computed(() => Vth + Id.value / gm)                  // Miller plateau voltage, V

// ── Switching times (all in seconds → displayed as ns) ────────────
const td_on  = computed(() => tau.value * Math.log(Vdrive.value / (Vdrive.value - Vth)))
const tr     = computed(() => Rg.value * Qgd / (Vdrive.value - Vplateau.value))
// Vgs continues charging from Vplateau to Vdrive after Miller plateau
//   t_rise_tail = tau * ln((Vdrive-Vplateau) / 0.05*(Vdrive-Vplateau))
// ≈ tau * 3  (to 95% of final value)
const t_rise_tail = computed(() => tau.value * 2.5)

const td_off = computed(() => tau.value * Math.log(Vdrive.value / Vplateau.value))
const tf     = computed(() => Rg.value * Qgd / Vplateau.value)
const t_fall_tail = computed(() => tau.value * 2.5)  // Vgs tail from Vplateau to 0

// Hold time between turn-on and turn-off (fixed padding for visual clarity)
const t_hold = 30e-9  // 30 ns

const totalTime = computed(() =>
  td_on.value + tr.value + t_rise_tail.value + t_hold +
  td_off.value + tf.value + t_fall_tail.value
)

// ── Canvas geometry constants ─────────────────────────────────────
const CANVAS_W = 600
const CANVAS_H = 450
const MARGIN_L = 65
const MARGIN_R = 20
const MARGIN_T_VGS = 30
const MARGIN_B_VGS = 3
const GAP_Y      = 30
const MARGIN_T_VDS = 8
const MARGIN_B   = 38

const PLOT_W    = CANVAS_W - MARGIN_L - MARGIN_R
const VGS_Y0    = MARGIN_T_VGS
const VGS_H     = (CANVAS_H - GAP_Y - MARGIN_B) / 2 - MARGIN_T_VGS
const VGS_Y1    = VGS_Y0 + VGS_H
const VDS_Y0    = VGS_Y1 + GAP_Y
const VDS_H     = CANVAS_H - VDS_Y0 - MARGIN_B

// Y-axis scales
const VGS_Y_MAX = 15    // V
const VDS_Y_MAX = Vbus  // V (48)

// Helper: map time (seconds) to canvas X
function timeToX(t: number, totalS: number): number {
  return MARGIN_L + (t / totalS) * PLOT_W
}

// Helper: draw centered text
function drawCenteredText(
  ctx: CanvasRenderingContext2D,
  text: string,
  x: number,
  y: number
) {
  ctx.textAlign = 'center'
  ctx.textBaseline = 'middle'
  ctx.fillText(text, x, y)
}

// ── Drawing ───────────────────────────────────────────────────────
const draw = () => {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  const w = canvas.width
  const h = canvas.height

  // Resolve CSS variable colors
  const style = getComputedStyle(document.documentElement)
  const bgColor   = style.getPropertyValue('--bg-color').trim()    || '#f8fafc'
  const textColor = style.getPropertyValue('--text-color').trim()   || '#1e293b'
  const textSec   = style.getPropertyValue('--text-secondary').trim() || '#64748b'
  const borderC   = style.getPropertyValue('--border-color').trim()  || '#e2e8f0'
  const primaryC  = style.getPropertyValue('--primary-color').trim() || '#3b82f6'

  ctx.clearRect(0, 0, w, h)
  ctx.fillStyle = bgColor
  ctx.fillRect(0, 0, w, h)

  // Snapshot all computed values (ensure consistency during draw)
  const _td_on  = td_on.value
  const _tr     = tr.value
  const _t_tail = t_rise_tail.value
  const _td_off = td_off.value
  const _tf     = tf.value
  const _t_fall = t_fall_tail.value
  const _total  = totalTime.value
  const _Vp     = Vplateau.value
  const _Vd     = Vdrive.value
  const _tau    = tau.value

  // Build time axis segments for the X-axis labels (builders)
  //  segs are [start, end] times in seconds for each labeled phase
  const segs: { label: string; t0: number; t1: number; color: string }[] = []
  let cursor = 0
  segs.push({ label: 'td(on)', t0: cursor, t1: cursor + _td_on, color: '#3b82f6' })
  cursor += _td_on
  segs.push({ label: 'tr',     t0: cursor, t1: cursor + _tr,     color: '#f59e0b' })
  cursor += _tr
  segs.push({ label: '',       t0: cursor, t1: cursor + _t_tail, color: '#3b82f6' })
  cursor += _t_tail
  cursor += t_hold
  segs.push({ label: 'td(off)',t0: cursor, t1: cursor + _td_off, color: '#3b82f6' })
  cursor += _td_off
  segs.push({ label: 'tf',     t0: cursor, t1: cursor + _tf,     color: '#f59e0b' })
  cursor += _tf
  segs.push({ label: '',       t0: cursor, t1: cursor + _t_fall, color: '#3b82f6' })

  // ── Helper: draw a grid line ────────────────────────────────────
  const gridLine = (x1: number, y1: number, x2: number, y2: number) => {
    ctx.beginPath()
    ctx.moveTo(x1, y1)
    ctx.lineTo(x2, y2)
    ctx.stroke()
  }

  // ── Draw Vgs channel ────────────────────────────────────────────
  ctx.save()
  // Clip to VGS region
  ctx.beginPath()
  ctx.rect(MARGIN_L, VGS_Y0, PLOT_W, VGS_H)
  ctx.clip()

  // ---- Grid lines ----
  ctx.strokeStyle = borderC
  ctx.lineWidth = 0.5
  ctx.setLineDash([3, 5])
  for (let v = 0; v <= VGS_Y_MAX; v += 3) {
    const y = VGS_Y1 - (v / VGS_Y_MAX) * VGS_H
    gridLine(MARGIN_L, y, MARGIN_L + PLOT_W, y)
    // Label (outside clip, handled below)
  }
  // vertical grid: phase boundaries
  ctx.setLineDash([2, 4])
  // Mark td(on) end, tr end
  const x1 = timeToX(_td_on, _total)
  const x2 = timeToX(_td_on + _tr, _total)
  const x3 = timeToX(_td_on + _tr + _t_tail + t_hold, _total)
  const x4 = timeToX(_td_on + _tr + _t_tail + t_hold + _td_off, _total)
  const x5 = timeToX(_td_on + _tr + _t_tail + t_hold + _td_off + _tf, _total)
  for (const x of [x1, x2, x3, x4, x5]) {
    // Extend through both channels later
    gridLine(x, VGS_Y0, x, VGS_Y1)
  }
  ctx.setLineDash([])

  // ---- Miller plateau highlights (turn-on) ----
  const x_m1_on  = timeToX(_td_on, _total)
  const x_m2_on  = timeToX(_td_on + _tr, _total)
  ctx.fillStyle = 'rgba(245, 158, 11, 0.12)'
  ctx.fillRect(x_m1_on, VGS_Y0, x_m2_on - x_m1_on, VGS_H)

  // ---- Miller plateau highlight (turn-off) ----
  const x_m1_off = timeToX(_td_on + _tr + _t_tail + t_hold + _td_off, _total)
  const x_m2_off = timeToX(_td_on + _tr + _t_tail + t_hold + _td_off + _tf, _total)
  ctx.fillStyle = 'rgba(245, 158, 11, 0.12)'
  ctx.fillRect(x_m1_off, VGS_Y0, x_m2_off - x_m1_off, VGS_H)

  // ---- Vth reference line ----
  ctx.strokeStyle = '#94a3b8'
  ctx.lineWidth = 1
  ctx.setLineDash([4, 3])
  const yVth = VGS_Y1 - (Vth / VGS_Y_MAX) * VGS_H
  gridLine(MARGIN_L, yVth, MARGIN_L + PLOT_W, yVth)
  ctx.setLineDash([])

  // ---- Vplateau reference line ----
  const yVp = VGS_Y1 - (_Vp / VGS_Y_MAX) * VGS_H
  ctx.strokeStyle = '#f59e0b'
  ctx.lineWidth = 0.8
  ctx.setLineDash([3, 4])
  gridLine(MARGIN_L, yVp, MARGIN_L + PLOT_W, yVp)
  ctx.setLineDash([])

  // ---- Vgs waveform ----
  ctx.strokeStyle = '#3b82f6'
  ctx.lineWidth = 2
  ctx.beginPath()

  let t = 0
  const vgsAt = (tSec: number): number => {
    // Turn-on delay phase: RC charging 0 → Vth
    if (tSec <= _td_on) {
      return _Vd * (1 - Math.exp(-tSec / _tau))
    }
    // Miller plateau: Vgs clamped at Vplateau
    if (tSec <= _td_on + _tr) {
      return _Vp
    }
    // Vgs rising tail: Vplateau → Vdrive
    const t2 = tSec - _td_on - _tr
    if (tSec <= _td_on + _tr + _t_tail) {
      return _Vp + (_Vd - _Vp) * (1 - Math.exp(-t2 / _tau))
    }
    // On-state hold
    if (tSec <= _td_on + _tr + _t_tail + t_hold) {
      return _Vd
    }
    // Turn-off delay: Vdrive → Vplateau (exponential decay)
    const t3 = tSec - (_td_on + _tr + _t_tail + t_hold)
    if (tSec <= _td_on + _tr + _t_tail + t_hold + _td_off) {
      return _Vd * Math.exp(-t3 / _tau)
    }
    // Turn-off Miller plateau: Vgs clamped at Vplateau
    if (tSec <= _td_on + _tr + _t_tail + t_hold + _td_off + _tf) {
      return _Vp
    }
    // Vgs falling tail: Vplateau → 0
    const t4 = tSec - (_td_on + _tr + _t_tail + t_hold + _td_off + _tf)
    return _Vp * Math.exp(-t4 / _tau)
  }

  // Sample the Vgs waveform at fine resolution
  const N_SAMPLES = 800
  for (let i = 0; i <= N_SAMPLES; i++) {
    const tSec = (i / N_SAMPLES) * _total
    const vgs = vgsAt(tSec)
    const px = MARGIN_L + (i / N_SAMPLES) * PLOT_W
    const py = VGS_Y1 - (vgs / VGS_Y_MAX) * VGS_H
    if (i === 0) ctx.moveTo(px, py)
    else ctx.lineTo(px, py)
  }
  ctx.stroke()

  // ---- Vgs axis labels (outside clip, so restore) ----
  ctx.restore()

  // Y-axis labels for Vgs
  ctx.fillStyle = textSec
  ctx.font = '10px sans-serif'
  ctx.textAlign = 'right'
  ctx.textBaseline = 'middle'
  for (let v = 0; v <= VGS_Y_MAX; v += 3) {
    const y = VGS_Y1 - (v / VGS_Y_MAX) * VGS_H
    ctx.fillText(`${v}V`, MARGIN_L - 5, y)
  }
  // Vth label
  ctx.fillStyle = '#94a3b8'
  ctx.fillText('Vth', MARGIN_L - 5, yVth)
  // Vplateau label
  ctx.fillStyle = '#f59e0b'
  ctx.fillText('Vp', MARGIN_L - 5, yVp)

  // Channel title
  ctx.fillStyle = primaryC
  ctx.font = 'bold 11px sans-serif'
  ctx.textAlign = 'left'
  ctx.textBaseline = 'top'
  ctx.fillText('Vgs (Gate-Source)', MARGIN_L, VGS_Y0 - 20)

  // ---- Annotations on Vgs waveform ----
  // td(on) label
  const xTdon = timeToX(_td_on, _total)
  ctx.fillStyle = '#3b82f6'
  ctx.font = '9px sans-serif'
  ctx.textAlign = 'center'
  ctx.textBaseline = 'bottom'
  if (xTdon - MARGIN_L > 25) {
    ctx.fillText('td(on)', xTdon, VGS_Y0)
  }

  // tr label
  const xTr = timeToX(_td_on + _tr, _total)
  const xTrMid = (x_m1_on + x_m2_on) / 2
  ctx.fillStyle = '#f59e0b'
  ctx.fillText('tr', xTrMid, VGS_Y0)

  // td(off) label
  const xTdoff = timeToX(_td_on + _tr + _t_tail + t_hold + _td_off, _total)
  ctx.fillStyle = '#3b82f6'
  if (xTdoff - xTr > 30) {
    ctx.fillText('td(off)', xTdoff, VGS_Y0)
  }

  // tf label
  const xTfMid = (x_m1_off + x_m2_off) / 2
  ctx.fillStyle = '#f59e0b'
  ctx.fillText('tf', xTfMid, VGS_Y0)

  // ── Draw Vds channel ────────────────────────────────────────────
  ctx.save()
  ctx.beginPath()
  ctx.rect(MARGIN_L, VDS_Y0, PLOT_W, VDS_H)
  ctx.clip()

  // ---- Grid lines ----
  ctx.strokeStyle = borderC
  ctx.lineWidth = 0.5
  ctx.setLineDash([3, 5])
  const vdsSteps = [0, 12, 24, 36, 48]
  for (const v of vdsSteps) {
    const y = VDS_Y0 + VDS_H - (v / VDS_Y_MAX) * VDS_H // 0 at bottom, Vbus at top
    gridLine(MARGIN_L, y, MARGIN_L + PLOT_W, y)
  }
  // vertical grid at phase boundaries
  ctx.setLineDash([2, 4])
  for (const x of [x1, x2, x3, x4, x5]) {
    gridLine(x, VDS_Y0, x, VDS_Y0 + VDS_H)
  }
  ctx.setLineDash([])

  // ---- Miller plateau highlights (Vds) ----
  ctx.fillStyle = 'rgba(245, 158, 11, 0.12)'
  ctx.fillRect(x_m1_on, VDS_Y0, x_m2_on - x_m1_on, VDS_H)
  ctx.fillRect(x_m1_off, VDS_Y0, x_m2_off - x_m1_off, VDS_H)

  // ---- Vds waveform ----
  // Vds drops from Vbus to Rdson*Id during turn-on Miller plateau,
  // stays low during on-state, then rises back during turn-off Miller plateau.
  const vdsOn  = Rdson * Id.value   // On-state Vds, very small
  const vdsAt = (tSec: number): number => {
    if (tSec <= _td_on) return Vbus
    // During turn-on Miller, Vds drops linearly
    if (tSec <= _td_on + _tr) {
      const frac = (tSec - _td_on) / _tr
      return Vbus - frac * (Vbus - vdsOn)
    }
    // Remainder of turn-on + on-state: Vds stays low
    if (tSec <= _td_on + _tr + _t_tail + t_hold + _td_off) {
      return vdsOn
    }
    // During turn-off Miller, Vds rises linearly
    if (tSec <= _td_on + _tr + _t_tail + t_hold + _td_off + _tf) {
      const frac = (tSec - (_td_on + _tr + _t_tail + t_hold + _td_off)) / _tf
      return vdsOn + frac * (Vbus - vdsOn)
    }
    return Vbus
  }

  ctx.strokeStyle = '#ef4444'
  ctx.lineWidth = 2
  ctx.beginPath()
  for (let i = 0; i <= N_SAMPLES; i++) {
    const tSec = (i / N_SAMPLES) * _total
    const vds = vdsAt(tSec)
    const px = MARGIN_L + (i / N_SAMPLES) * PLOT_W
    // Vds: 0 at bottom, Vbus at top
    const py = VDS_Y0 + VDS_H - (vds / VDS_Y_MAX) * VDS_H
    if (i === 0) ctx.moveTo(px, py)
    else ctx.lineTo(px, py)
  }
  ctx.stroke()

  ctx.restore()

  // Y-axis labels for Vds
  ctx.fillStyle = textSec
  ctx.font = '10px sans-serif'
  ctx.textAlign = 'right'
  ctx.textBaseline = 'middle'
  for (const v of vdsSteps) {
    const y = VDS_Y0 + VDS_H - (v / VDS_Y_MAX) * VDS_H
    ctx.fillText(`${v}V`, MARGIN_L - 5, y)
  }

  // Channel title
  ctx.fillStyle = primaryC
  ctx.font = 'bold 11px sans-serif'
  ctx.textAlign = 'left'
  ctx.textBaseline = 'top'
  ctx.fillText('Vds (Drain-Source)', MARGIN_L, VDS_Y0 - 5)

  // ---- Vds annotations ----
  ctx.fillStyle = textSec
  ctx.font = '9px sans-serif'
  ctx.textAlign = 'center'

  // Vds falling edge label
  const yVdsHigh = VDS_Y0 + VDS_H - (Vbus / VDS_Y_MAX) * VDS_H
  ctx.textBaseline = 'top'
  ctx.fillText('Vbus', MARGIN_L + 2, yVdsHigh + 2)

  // Vds on-state label
  const yVdsLow = VDS_Y0 + VDS_H - (vdsOn / VDS_Y_MAX) * VDS_H
  ctx.textBaseline = 'bottom'
  if (yVdsLow > VDS_Y0 + 15) {
    ctx.fillText(`Rdson*Id≈${(vdsOn*1000).toFixed(0)}mV`, MARGIN_L + PLOT_W / 2, yVdsLow - 2)
  }

  // ── X-axis (time) ───────────────────────────────────────────────
  const xAxisY = VDS_Y0 + VDS_H + 2
  ctx.strokeStyle = textColor
  ctx.lineWidth = 0.8
  gridLine(MARGIN_L, xAxisY, MARGIN_L + PLOT_W, xAxisY)

  // Time ticks
  // Choose a reasonable number of ticks based on total time
  const totalNs = _total * 1e9
  ctx.fillStyle = textSec
  ctx.font = '9px sans-serif'
  ctx.textAlign = 'center'
  ctx.textBaseline = 'top'

  // Draw tick at 0
  ctx.fillText('0', MARGIN_L, xAxisY + 4)

  // Draw tick at each phase boundary
  const tickPositions: { t: number; label: string }[] = [
    { t: _td_on, label: '' },
    { t: _td_on + _tr, label: '' },
    { t: _td_on + _tr + _t_tail + t_hold, label: '' },
    { t: _td_on + _tr + _t_tail + t_hold + _td_off, label: '' },
    { t: _td_on + _tr + _t_tail + t_hold + _td_off + _tf, label: '' },
    { t: _total, label: `${totalNs.toFixed(0)}ns` },
  ]

  for (const tp of tickPositions) {
    const tx = timeToX(tp.t, _total)
    if (tx > MARGIN_L + 5 && tx < MARGIN_L + PLOT_W - 5) {
      gridLine(tx, xAxisY, tx, xAxisY + 5)
      if (tp.label) {
        ctx.fillText(tp.label, tx, xAxisY + 7)
      }
    }
  }

  // X-axis label
  ctx.fillStyle = textSec
  ctx.font = '10px sans-serif'
  ctx.textAlign = 'center'
  ctx.textBaseline = 'top'
  ctx.fillText('时间 (ns)', MARGIN_L + PLOT_W / 2, xAxisY + 20)

  // ── Legend ──────────────────────────────────────────────────────
  const legX = MARGIN_L
  const legY = CANVAS_H - 4
  ctx.font = '9px sans-serif'
  ctx.textAlign = 'left'
  ctx.textBaseline = 'bottom'

  // Vgs legend
  ctx.strokeStyle = '#3b82f6'
  ctx.lineWidth = 2
  ctx.beginPath()
  ctx.moveTo(legX, legY)
  ctx.lineTo(legX + 18, legY)
  ctx.stroke()
  ctx.fillStyle = textSec
  ctx.fillText('Vgs', legX + 22, legY + 1)

  // Vds legend
  ctx.strokeStyle = '#ef4444'
  ctx.lineWidth = 2
  ctx.beginPath()
  ctx.moveTo(legX + 55, legY)
  ctx.lineTo(legX + 73, legY)
  ctx.stroke()
  ctx.fillText('Vds', legX + 77, legY + 1)

  // Miller highlight legend
  ctx.fillStyle = 'rgba(245, 158, 11, 0.25)'
  ctx.fillRect(legX + 115, legY - 8, 16, 10)
  ctx.fillStyle = textSec
  ctx.fillText('Miller plateau', legX + 135, legY + 1)
}

// ── Re-draw when any slider changes ───────────────────────────────
watch([Rg, Vdrive, Id, Ciss], () => {
  nextTick(draw)
})

onMounted(() => {
  nextTick(draw)
})
</script>

<template>
  <div class="sim-container">
    <div class="sim-header">
      <h4>MOSFET 开关波形仿真</h4>
    </div>
    <canvas ref="canvasRef" width="600" height="450" class="sim-canvas"></canvas>
    <div class="sim-params">
      <label class="param-label">
        Rg <span class="param-val">{{ Rg }} Ω</span>
        <input type="range" v-model.number="Rg" min="1" max="100" step="0.1" />
      </label>
      <label class="param-label">
        Vdrive <span class="param-val">{{ Vdrive }} V</span>
        <input type="range" v-model.number="Vdrive" min="8" max="20" step="0.1" />
      </label>
      <label class="param-label">
        Id <span class="param-val">{{ Id }} A</span>
        <input type="range" v-model.number="Id" min="1" max="50" step="0.1" />
      </label>
      <label class="param-label">
        Ciss <span class="param-val">{{ Ciss.toFixed(1) }} nF</span>
        <input type="range" v-model.number="Ciss" min="0.5" max="10" step="0.1" />
      </label>
    </div>
    <div class="sim-info">
      <div class="info-item">
        <span class="info-label">V<sub>plateau</sub></span>
        <span class="info-value">{{ Vplateau.toFixed(2) }} V</span>
      </div>
      <div class="info-item">
        <span class="info-label">t<sub>d(on)</sub></span>
        <span class="info-value">{{ (td_on * 1e9).toFixed(1) }} ns</span>
      </div>
      <div class="info-item">
        <span class="info-label">t<sub>r</sub></span>
        <span class="info-value">{{ (tr * 1e9).toFixed(1) }} ns</span>
      </div>
      <div class="info-item">
        <span class="info-label">t<sub>d(off)</sub></span>
        <span class="info-value">{{ (td_off * 1e9).toFixed(1) }} ns</span>
      </div>
      <div class="info-item">
        <span class="info-label">t<sub>f</sub></span>
        <span class="info-value">{{ (tf * 1e9).toFixed(1) }} ns</span>
      </div>
    </div>
    <div class="explain-panel">
      <details>
        <summary>📖 教学说明 — 理解 MOSFET 开关过程</summary>
        <div class="explain-content">
          <div class="explain-section">
            <h5>开关过程 7 个阶段</h5>
            <table class="phase-table">
              <thead><tr><th>阶段</th><th>名称</th><th>Vgs 变化</th><th>Vds 变化</th><th>物理过程</th></tr></thead>
              <tbody>
                <tr><td>1</td><td>t<sub>d(on)</sub> 开通延迟</td><td>0 → V<sub>th</sub></td><td>= V<sub>bus</sub></td><td>栅极电压通过 Rg 对 Ciss 充电，达到阈值前 MOSFET 不导通</td></tr>
                <tr><td>2</td><td>t<sub>r</sub> 上升时间</td><td>= V<sub>plateau</sub></td><td>V<sub>bus</sub> → R<sub>dson</sub>·I<sub>d</sub></td><td><strong>米勒平台</strong>：Vgs 被钳位，栅极电流全部用于对 Cgd 放电，Vds 快速下降</td></tr>
                <tr><td>3</td><td>尾部充电</td><td>V<sub>plateau</sub> → V<sub>drive</sub></td><td>≈ R<sub>dson</sub>·I<sub>d</sub></td><td>米勒效应结束，Vgs 继续上升至驱动电压，沟道完全增强</td></tr>
                <tr><td>4</td><td>导通保持</td><td>= V<sub>drive</sub></td><td>= R<sub>dson</sub>·I<sub>d</sub></td><td>MOSFET 完全导通，Vds 很小（导通压降）</td></tr>
                <tr><td>5</td><td>t<sub>d(off)</sub> 关断延迟</td><td>V<sub>drive</sub> → V<sub>plateau</sub></td><td>≈ R<sub>dson</sub>·I<sub>d</sub></td><td>Ciss 通过 Rg 放电至米勒平台电压</td></tr>
                <tr><td>6</td><td>t<sub>f</sub> 下降时间</td><td>= V<sub>plateau</sub></td><td>R<sub>dson</sub>·I<sub>d</sub> → V<sub>bus</sub></td><td><strong>米勒平台</strong>：Vgs 再次被钳位，Cgd 充电使 Vds 上升</td></tr>
                <tr><td>7</td><td>尾部放电</td><td>V<sub>plateau</sub> → 0</td><td>= V<sub>bus</sub></td><td>Ciss 完全放电，沟道完全关断</td></tr>
              </tbody>
            </table>
          </div>
          <div class="explain-section">
            <h5>米勒平台原理</h5>
            <p>当 Vgs 达到 <strong>V<sub>plateau</sub> = V<sub>th</sub> + I<sub>d</sub>/g<sub>m</sub></strong> 时，MOSFET 进入饱和区。此时栅极驱动电流不再给 Cgs 充电，而是<strong>全部用于对米勒电容 Cgd 充放电</strong>。Vgs 被"钳位"在 Vplateau，形成波形中的平坦区域（黄色高亮区）。</p>
            <p><strong>关键公式</strong>：米勒平台持续时间 <code>t_plateau = Rg · Qgd / (Vdrive - Vplateau)</code>，即 t<sub>r</sub> 和 t<sub>f</sub> 的长度。</p>
          </div>
          <div class="explain-section">
            <h5>参数影响</h5>
            <ul>
              <li><strong>Rg ↑</strong> → 开关速度 ↓ → tr, tf 增大 → <strong>开关损耗 ↑</strong>，但 EMI ↓（di/dt 和 dv/dt 降低）</li>
              <li><strong>Vdrive ↑</strong> → 开关速度 ↑ → tr, tf 减小 → 开关损耗 ↓，但需要确保 Vgs 不超过 ±20V 额定值</li>
              <li><strong>Ciss ↑</strong>（大功率 MOSFET）→ 充放电时间 ↑ → 需要更大的栅极驱动电流</li>
            </ul>
          </div>
          <div class="explain-section">
            <h5>开关损耗估算</h5>
            <p>单次开关能量（硬开关）：</p>
            <p><code>E<sub>sw</sub> ≈ 0.5 · V<sub>bus</sub> · I<sub>d</sub> · (t<sub>r</sub> + t<sub>f</sub>)</code></p>
            <p>开关损耗功率（PWM 频率 f<sub>sw</sub>）：</p>
            <p><code>P<sub>sw</sub> = E<sub>sw</sub> · f<sub>sw</sub></code></p>
            <p><strong>示例</strong>：Vbus=48V, Id=10A, tr+tf=50ns, fsw=20kHz → Esw≈12μJ, Psw≈0.24W。若 tr+tf 增大到 200ns（Rg 过大），Psw 升至 ≈1W，散热需求显著增加。</p>
          </div>
        </div>
      </details>
    </div>
  </div>
</template>

<style scoped>
.sim-container {
  border: 1px solid var(--border-color);
  border-radius: 8px;
  padding: 1rem;
  margin: 1.5rem 0;
  background: var(--bg-color);
}

.sim-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 0.75rem;
  flex-wrap: wrap;
  gap: 0.5rem;
}

.sim-header h4 {
  margin: 0;
  font-size: 0.95rem;
  color: var(--primary-color);
}

.sim-canvas {
  display: block;
  margin: 0 auto;
  max-width: 100%;
  border-radius: 6px;
  border: 1px solid var(--border-color);
}

.sim-params {
  display: flex;
  gap: 1.25rem;
  margin-top: 0.75rem;
  font-size: 0.8rem;
  justify-content: center;
  flex-wrap: wrap;
}

.param-label {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.2rem;
  color: var(--text-secondary);
  cursor: default;
  user-select: none;
}

.param-val {
  font-weight: 600;
  color: var(--text-color);
  font-size: 0.78rem;
}

.param-label input[type="range"] {
  width: 90px;
  accent-color: var(--primary-color);
  cursor: pointer;
}

.sim-info {
  display: flex;
  gap: 1.25rem;
  margin-top: 0.75rem;
  font-size: 0.8rem;
  justify-content: center;
  flex-wrap: wrap;
  padding: 0.5rem 1rem;
  background: var(--code-bg);
  border-radius: 6px;
  border: 1px solid var(--border-color);
}

.info-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.1rem;
}

.info-label {
  color: var(--text-secondary);
  font-size: 0.72rem;
}

.info-value {
  color: var(--text-color);
  font-weight: 600;
  font-family: 'Courier New', Courier, monospace;
  font-size: 0.82rem;
}

.explain-panel { margin-top: 0.75rem; padding: 0.6rem 0.75rem; background: var(--content-bg); border: 1px solid var(--border-color); border-radius: 6px; font-size: 0.8rem; }
.explain-panel details summary { font-weight: 600; color: var(--primary-color); cursor: pointer; font-size: 0.85rem; }
.explain-content { margin-top: 0.5rem; line-height: 1.6; }
.explain-section { margin-bottom: 0.7rem; }
.explain-section h5 { font-size: 0.82rem; font-weight: 700; margin: 0 0 0.3rem 0; color: var(--text-color); }
.explain-section p { margin: 0.2rem 0; color: var(--text-secondary); }
.explain-section ul { margin: 0.2rem 0; padding-left: 1.2rem; color: var(--text-secondary); }
.explain-section ul li { margin: 0.12rem 0; }
.phase-table { width: 100%; border-collapse: collapse; margin: 0.4rem 0; font-size: 0.73rem; }
.phase-table th, .phase-table td { border: 1px solid var(--border-color); padding: 0.2rem 0.4rem; }
.phase-table th { background: var(--code-bg, #f1f5f9); text-align: center; }
.phase-table td:first-child { text-align: center; font-weight: 600; }
</style>