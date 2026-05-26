<script setup lang="ts">
import { ref, watch, onMounted, nextTick } from 'vue'

// ── Canvas ref ──
const canvasRef = ref<HTMLCanvasElement | null>(null)

// ── Reactive parameters ──
const K = ref(100)    // DC gain
const wz = ref(50)    // zero frequency ωz (rad/s)
const wp1 = ref(10)   // pole 1 frequency ωp1 (rad/s)
const wp2 = ref(100)  // pole 2 frequency ωp2 (rad/s)

// ── Computed results ──
const omegaC = ref<number | null>(null)       // crossover frequency
const phaseMargin = ref<number | null>(null)   // phase margin (degrees)
const gainMargin = ref<number | null>(null)    // gain margin (dB)

// ── Canvas layout constants ──
const CW = 600
const CH = 500
const ML = 72   // left margin (room for y-axis labels)
const MR = 18   // right margin
const MT = 22   // top margin
const MB = 28   // bottom margin

// Two stacked plot areas
const MAG_TOP = MT
const MAG_BOT = 228
const PHASE_TOP = 274
const PHASE_BOT = CH - MB

const FREQ_MIN = 0.1
const FREQ_MAX = 1000
const N_POINTS = 500

const logMin = Math.log10(FREQ_MIN)
const logMax = Math.log10(FREQ_MAX)
const plotW = CW - ML - MR
const magH = MAG_BOT - MAG_TOP
const phaseH = PHASE_BOT - PHASE_TOP
const decades = [0.1, 1, 10, 100, 1000]

// ── Utility helpers ──

/** Generate N_POINTS log-spaced frequencies from FREQ_MIN to FREQ_MAX */
function generateFreqs(): number[] {
  const out: number[] = []
  for (let i = 0; i < N_POINTS; i++) {
    const logF = logMin + (i / (N_POINTS - 1)) * (logMax - logMin)
    out.push(Math.pow(10, logF))
  }
  return out
}

/** Map frequency ω to canvas x pixel coordinate */
function freqToX(w: number): number {
  return ML + ((Math.log10(w) - logMin) / (logMax - logMin)) * plotW
}

/**
 * Compute magnitude (dB) and phase (degrees) at every frequency.
 * G(s) = K * (s/ωz + 1) / ((s/ωp1 + 1)(s/ωp2 + 1))
 */
function computeResponse(freqs: number[]): { mags: number[]; phases: number[] } {
  const mags: number[] = []
  const phases: number[] = []
  const k = K.value
  const z = wz.value
  const p1 = wp1.value
  const p2 = wp2.value
  const r2d = 180 / Math.PI

  for (const w of freqs) {
    // |G(jω)|_dB = 20lg K + 10lg(1+(ω/ωz)²) - 10lg(1+(ω/ωp1)²) - 10lg(1+(ω/ωp2)²)
    let mag = 20 * Math.log10(k)
    mag += 10 * Math.log10(1 + (w / z) * (w / z))
    mag -= 10 * Math.log10(1 + (w / p1) * (w / p1))
    mag -= 10 * Math.log10(1 + (w / p2) * (w / p2))
    mags.push(mag)

    // ∠G(jω) = arctan(ω/ωz) - arctan(ω/ωp1) - arctan(ω/ωp2)  [degrees]
    let phase = Math.atan(w / z) - Math.atan(w / p1) - Math.atan(w / p2)
    phases.push(phase * r2d)
  }

  return { mags, phases }
}

/** Find ωc where magnitude crosses 0 dB (gain crossover, from positive to negative) */
function findCrossover(freqs: number[], mags: number[]): number | null {
  for (let i = 1; i < freqs.length; i++) {
    if (mags[i - 1] >= 0 && mags[i] < 0) {
      // log-space linear interpolation
      const t = mags[i - 1] / (mags[i - 1] - mags[i])
      return freqs[i - 1] * Math.pow(freqs[i] / freqs[i - 1], t)
    }
  }
  return null
}

/** Find ω_180 where phase crosses -180° (phase crossover, from above to below) */
function findPhase180(freqs: number[], phases: number[]): number | null {
  for (let i = 1; i < freqs.length; i++) {
    if (phases[i - 1] >= -180 && phases[i] < -180) {
      const t = (phases[i - 1] + 180) / (phases[i - 1] - phases[i])
      return freqs[i - 1] * Math.pow(freqs[i] / freqs[i - 1], t)
    }
  }
  return null
}

/** Get interpolated phase at a specific frequency */
function getPhaseAt(freqs: number[], phases: number[], wTarget: number): number {
  for (let i = 1; i < freqs.length; i++) {
    if (freqs[i] >= wTarget) {
      const t = Math.log(wTarget / freqs[i - 1]) / Math.log(freqs[i] / freqs[i - 1])
      return phases[i - 1] + t * (phases[i] - phases[i - 1])
    }
  }
  return phases[phases.length - 1]
}

/** Get interpolated magnitude at a specific frequency */
function getMagAt(freqs: number[], mags: number[], wTarget: number): number {
  for (let i = 1; i < freqs.length; i++) {
    if (freqs[i] >= wTarget) {
      const t = Math.log(wTarget / freqs[i - 1]) / Math.log(freqs[i] / freqs[i - 1])
      return mags[i - 1] + t * (mags[i] - mags[i - 1])
    }
  }
  return mags[mags.length - 1]
}

/** Format a numeric value with unit, or 'N/A' / '∞' if null or infinite */
function formatVal(v: number | null, unit: string): string {
  if (v === null) return 'N/A'
  if (!isFinite(v)) return '\u221E ' + unit
  return v.toFixed(1) + ' ' + unit
}

// ── Main draw routine ──
function draw() {
  const canvas = canvasRef.value
  if (!canvas) return

  // ── Compute data ──
  const freqs = generateFreqs()
  const { mags, phases } = computeResponse(freqs)

  // ── Stability metrics ──
  const wc = findCrossover(freqs, mags)
  const w180 = findPhase180(freqs, phases)

  omegaC.value = wc
  if (wc !== null) {
    phaseMargin.value = 180 + getPhaseAt(freqs, phases, wc)
  } else {
    phaseMargin.value = null
  }
  if (w180 !== null) {
    gainMargin.value = -getMagAt(freqs, mags, w180)
  } else {
    gainMargin.value = null
  }

  // ── Compute axis ranges (dynamic, with padding) ──
  let magMin = Infinity, magMax = -Infinity
  let phMin = Infinity, phMax = -Infinity
  for (let i = 0; i < freqs.length; i++) {
    if (mags[i] < magMin) magMin = mags[i]
    if (mags[i] > magMax) magMax = mags[i]
    if (phases[i] < phMin) phMin = phases[i]
    if (phases[i] > phMax) phMax = phases[i]
  }
  const magPad = Math.max((magMax - magMin) * 0.15, 5)
  magMin -= magPad
  magMax += magPad
  magMin = Math.floor(magMin / 20) * 20
  magMax = Math.ceil(magMax / 20) * 20

  const phPad = Math.max((phMax - phMin) * 0.12, 10)
  phMin -= phPad
  phMax += phPad
  phMin = Math.floor(phMin / 45) * 45
  phMax = Math.ceil(phMax / 45) * 45

  // ── Mapping helpers ──
  const magToY = (m: number) => MAG_BOT - ((m - magMin) / (magMax - magMin)) * magH
  const phaseToY = (p: number) => PHASE_BOT - ((p - phMin) / (phMax - phMin)) * phaseH

  // ── Get context ──
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  // ── Read CSS variables for theming ──
  const cs = getComputedStyle(document.documentElement)
  const bgColor = cs.getPropertyValue('--bg-color').trim() || '#f8fafc'
  const textColor = cs.getPropertyValue('--text-color').trim() || '#1e293b'
  const textSec = cs.getPropertyValue('--text-secondary').trim() || '#64748b'
  const borderCol = cs.getPropertyValue('--border-color').trim() || '#e2e8f0'
  const primColor = cs.getPropertyValue('--primary-color').trim() || '#3b82f6'

  // ── Clear canvas ──
  ctx.clearRect(0, 0, CW, CH)
  ctx.fillStyle = bgColor
  ctx.fillRect(0, 0, CW, CH)

  // ═══════════════════════════════════════════
  //  MAGNITUDE PLOT (top)
  // ═══════════════════════════════════════════

  // -- horizontal grid (dB) --
  ctx.strokeStyle = borderCol
  ctx.lineWidth = 0.5
  const magGridStart = Math.ceil(magMin / 20) * 20
  for (let db = magGridStart; db <= magMax + 0.5; db += 20) {
    const y = magToY(db)
    ctx.beginPath()
    ctx.moveTo(ML, y)
    ctx.lineTo(CW - MR, y)
    ctx.stroke()
    // y-axis label
    ctx.fillStyle = textSec
    ctx.font = '10px monospace'
    ctx.textAlign = 'right'
    ctx.fillText(String(db), ML - 5, y + 3)
  }

  // -- vertical grid (decades) --
  for (const d of decades) {
    const x = freqToX(d)
    ctx.beginPath()
    ctx.moveTo(x, MAG_TOP)
    ctx.lineTo(x, MAG_BOT)
    ctx.stroke()
  }

  // -- 0 dB reference line (dashed, if visible) --
  if (magMin <= 0 && magMax >= 0) {
    ctx.strokeStyle = textSec
    ctx.lineWidth = 1
    ctx.setLineDash([5, 4])
    const y0 = magToY(0)
    ctx.beginPath()
    ctx.moveTo(ML, y0)
    ctx.lineTo(CW - MR, y0)
    ctx.stroke()
    ctx.fillStyle = textSec
    ctx.font = '9px monospace'
    ctx.textAlign = 'left'
    ctx.fillText('0 dB', CW - MR - 28, y0 - 3)
    ctx.setLineDash([])
  }

  // -- magnitude curve --
  ctx.strokeStyle = primColor
  ctx.lineWidth = 2
  ctx.beginPath()
  for (let i = 0; i < freqs.length; i++) {
    const x = freqToX(freqs[i])
    const y = magToY(mags[i])
    if (i === 0) ctx.moveTo(x, y)
    else ctx.lineTo(x, y)
  }
  ctx.stroke()

  // -- crossover marker (ωc) --
  if (wc !== null) {
    const xc = freqToX(wc)
    const yc = magToY(0)
    ctx.fillStyle = '#ef4444'
    ctx.beginPath()
    ctx.arc(xc, yc, 5, 0, 2 * Math.PI)
    ctx.fill()
    ctx.fillStyle = '#ef4444'
    ctx.font = 'bold 10px monospace'
    ctx.textAlign = 'left'
    ctx.fillText('\u03C9c=' + wc.toFixed(1), xc + 8, yc - 4)
  }

  // -- subplot title --
  ctx.fillStyle = textColor
  ctx.font = 'bold 11px sans-serif'
  ctx.textAlign = 'left'
  ctx.fillText('幅值响应  |G(j\u03C9)| (dB)', ML + 4, MAG_TOP + 14)

  // -- y-axis label (rotated) --
  ctx.fillStyle = textColor
  ctx.font = 'bold 12px sans-serif'
  ctx.textAlign = 'center'
  ctx.save()
  ctx.translate(13, (MAG_TOP + MAG_BOT) / 2)
  ctx.rotate(-Math.PI / 2)
  ctx.fillText('幅值 (dB)', 0, 0)
  ctx.restore()

  // ═══════════════════════════════════════════
  //  PHASE PLOT (bottom)
  // ═══════════════════════════════════════════

  // -- horizontal grid (degrees) --
  ctx.strokeStyle = borderCol
  ctx.lineWidth = 0.5
  const phGridStart = Math.ceil(phMin / 45) * 45
  for (let deg = phGridStart; deg <= phMax + 0.5; deg += 45) {
    const y = phaseToY(deg)
    ctx.beginPath()
    ctx.moveTo(ML, y)
    ctx.lineTo(CW - MR, y)
    ctx.stroke()
    ctx.fillStyle = textSec
    ctx.font = '10px monospace'
    ctx.textAlign = 'right'
    ctx.fillText(deg + '\u00B0', ML - 5, y + 3)
  }

  // -- vertical grid (decades) --
  for (const d of decades) {
    const x = freqToX(d)
    ctx.beginPath()
    ctx.moveTo(x, PHASE_TOP)
    ctx.lineTo(x, PHASE_BOT)
    ctx.stroke()
  }

  // -- -180\u00B0 reference line (dashed, if visible) --
  if (phMin <= -180 && phMax >= -180) {
    ctx.strokeStyle = textSec
    ctx.lineWidth = 1
    ctx.setLineDash([5, 4])
    const y180 = phaseToY(-180)
    ctx.beginPath()
    ctx.moveTo(ML, y180)
    ctx.lineTo(CW - MR, y180)
    ctx.stroke()
    ctx.fillStyle = textSec
    ctx.font = '9px monospace'
    ctx.textAlign = 'left'
    ctx.fillText('-180\u00B0', CW - MR - 36, y180 - 3)
    ctx.setLineDash([])
  }

  // -- phase curve --
  ctx.strokeStyle = '#f59e0b'  // amber
  ctx.lineWidth = 2
  ctx.beginPath()
  for (let i = 0; i < freqs.length; i++) {
    const x = freqToX(freqs[i])
    const y = phaseToY(phases[i])
    if (i === 0) ctx.moveTo(x, y)
    else ctx.lineTo(x, y)
  }
  ctx.stroke()

  // -- phase crossover marker (ω_180) --
  if (w180 !== null) {
    const x180 = freqToX(w180)
    const y180 = phaseToY(-180)
    ctx.fillStyle = '#ef4444'
    ctx.beginPath()
    ctx.arc(x180, y180, 5, 0, 2 * Math.PI)
    ctx.fill()
    ctx.fillStyle = '#ef4444'
    ctx.font = 'bold 10px monospace'
    ctx.textAlign = 'left'
    ctx.fillText('\u03C9\u2081\u2088\u2080=' + w180.toFixed(1), x180 + 8, y180 - 4)
  }

  // -- subplot title --
  ctx.fillStyle = textColor
  ctx.font = 'bold 11px sans-serif'
  ctx.textAlign = 'left'
  ctx.fillText('相位响应  \u2220G(j\u03C9) (\u00B0)', ML + 4, PHASE_TOP + 14)

  // -- y-axis label (rotated) --
  ctx.fillStyle = textColor
  ctx.font = 'bold 12px sans-serif'
  ctx.textAlign = 'center'
  ctx.save()
  ctx.translate(13, (PHASE_TOP + PHASE_BOT) / 2)
  ctx.rotate(-Math.PI / 2)
  ctx.fillText('相位 (\u00B0)', 0, 0)
  ctx.restore()

  // ═══════════════════════════════════════════
  //  SHARED X-AXIS
  // ═══════════════════════════════════════════

  // -- decade labels at bottom --
  ctx.fillStyle = textSec
  ctx.font = '10px monospace'
  ctx.textAlign = 'center'
  for (const d of decades) {
    ctx.fillText(String(d), freqToX(d), PHASE_BOT + 14)
  }

  // -- x-axis title --
  ctx.fillStyle = textColor
  ctx.font = 'bold 12px sans-serif'
  ctx.fillText('\u03C9 (rad/s)', CW / 2, CH - 4)
}

// ── Reactivity: redraw on any parameter change ──
watch([K, wz, wp1, wp2], () => {
  draw()
})

// ── Lifecycle ──
onMounted(() => {
  const canvas = canvasRef.value
  if (canvas) {
    canvas.width = CW
    canvas.height = CH
  }
  nextTick(() => draw())
})
</script>

<template>
  <div class="sim-container">
    <div class="sim-header">
      <h4>Bode 图交互仿真</h4>
      <span class="sim-hint">
        传递函数: G(s) = K &middot; (s/&omega;<sub>z</sub> + 1) / ((s/&omega;<sub>p1</sub> + 1)(s/&omega;<sub>p2</sub> + 1))
      </span>
    </div>

    <canvas ref="canvasRef" class="sim-canvas" width="600" height="500"></canvas>

    <div class="sim-params">
      <div class="params-grid">
        <div class="param-item">
          <label>K (增益): <span class="param-val">{{ K }}</span></label>
          <input type="range" v-model.number="K" min="1" max="500" step="1" />
        </div>
        <div class="param-item">
          <label>&omega;<sub>z</sub> (零点): <span class="param-val">{{ wz }} rad/s</span></label>
          <input type="range" v-model.number="wz" min="1" max="500" step="1" />
        </div>
        <div class="param-item">
          <label>&omega;<sub>p1</sub> (极点1): <span class="param-val">{{ wp1 }} rad/s</span></label>
          <input type="range" v-model.number="wp1" min="1" max="500" step="1" />
        </div>
        <div class="param-item">
          <label>&omega;<sub>p2</sub> (极点2): <span class="param-val">{{ wp2 }} rad/s</span></label>
          <input type="range" v-model.number="wp2" min="1" max="500" step="1" />
        </div>
      </div>

      <div class="computed-row">
        <span class="computed-item">
          <span class="computed-label">穿越频率 &omega;<sub>c</sub>:</span>
          <span class="computed-val">{{ formatVal(omegaC, 'rad/s') }}</span>
        </span>
        <span class="computed-item">
          <span class="computed-label">相位裕度 PM:</span>
          <span class="computed-val" :class="{ 'warning': phaseMargin !== null && phaseMargin < 30 }">
            {{ formatVal(phaseMargin, '\u00B0') }}
          </span>
        </span>
        <span class="computed-item">
          <span class="computed-label">增益裕度 GM:</span>
          <span class="computed-val" :class="{ 'warning': gainMargin !== null && gainMargin < 6 }">
            {{ formatVal(gainMargin, 'dB') }}
          </span>
        </span>
      </div>

      <div class="explain-panel">
        <details>
          <summary>📖 教学说明 — 理解 Bode 图</summary>
          <div class="explain-content">
            <div class="explain-section">
              <h5>传递函数说明</h5>
              <p>当前传递函数为 <code>G(s) = K · (s/ω<sub>z</sub>+1) / ((s/ω<sub>p1</sub>+1)(s/ω<sub>p2</sub>+1))</code>，包含 <strong>1 个零点</strong>（分子）和 <strong>2 个极点</strong>（分母）。</p>
              <ul>
                <li><strong>增益 K</strong>：使幅频曲线整体上移 20lg(K) dB，不影响相位。增大 K 使系统响应更快但可能降低稳定性。</li>
                <li><strong>零点 ω<sub>z</sub></strong>：幅频 +20dB/dec 转折，相位 +90°（超前）。零点可改善相位裕度，常用于补偿。</li>
                <li><strong>极点 ω<sub>p1</sub>, ω<sub>p2</sub></strong>：幅频 -20dB/dec 转折，相位 -90°（滞后）。每个极点贡献 -90° 相移，两个极点可达 -180°，导致不稳定。</li>
              </ul>
            </div>
            <div class="explain-section">
              <h5>稳定性判据速查</h5>
              <table class="stability-table">
                <thead><tr><th>指标</th><th>合格</th><th>良好</th><th>说明</th></tr></thead>
                <tbody>
                  <tr><td>相位裕度 PM</td><td>&gt; 30°</td><td>&gt; 45°</td><td>系统对延迟的容忍度。PM 越大，超调越小，系统越"稳定"。</td></tr>
                  <tr><td>增益裕度 GM</td><td>&gt; 6 dB</td><td>&gt; 10 dB</td><td>增益可增大多少倍才不稳定。GM 越大，对参数变化的鲁棒性越强。</td></tr>
                </tbody>
              </table>
              <p><strong>经验法则</strong>：伺服系统通常要求 PM ≥ 45°、GM ≥ 10dB。当 PM &lt; 30°（红色警告）时系统有明显振荡趋势。</p>
            </div>
            <div class="explain-section">
              <h5>典型场景预设</h5>
              <div class="preset-buttons">
                <button @click="K=1; wz=100; wp1=10; wp2=100" class="preset-btn">低通滤波器</button>
                <button @click="K=100; wz=10; wp1=50; wp2=200" class="preset-btn">超前补偿</button>
                <button @click="K=200; wz=500; wp1=5; wp2=200" class="preset-btn">不稳定系统</button>
              </div>
              <p class="preset-hint">点击预设按钮快速切换参数，观察 Bode 图和稳定裕度的变化。</p>
            </div>
          </div>
        </details>
      </div>
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

.sim-hint {
  font-size: 0.78rem;
  color: var(--text-secondary);
  font-style: italic;
}

.sim-canvas {
  display: block;
  margin: 0 auto;
  max-width: 100%;
  border-radius: 6px;
  border: 1px solid var(--border-color);
}

.sim-params {
  margin-top: 0.75rem;
}

.params-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 0.6rem 1.2rem;
}

.param-item {
  display: flex;
  flex-direction: column;
  gap: 0.2rem;
  font-size: 0.8rem;
}

.param-item label {
  color: var(--text-color);
  white-space: nowrap;
}

.param-val {
  font-weight: 600;
  color: var(--primary-color);
  font-family: monospace;
}

.param-item input[type="range"] {
  width: 100%;
  cursor: pointer;
  accent-color: var(--primary-color);
}

.computed-row {
  display: flex;
  gap: 1.5rem;
  margin-top: 0.75rem;
  padding-top: 0.6rem;
  border-top: 1px solid var(--border-color);
  font-size: 0.82rem;
  justify-content: center;
  flex-wrap: wrap;
}

.computed-item {
  display: flex;
  gap: 0.35rem;
  align-items: baseline;
}

.computed-label {
  color: var(--text-secondary);
}

.computed-val {
  font-weight: 600;
  color: var(--text-color);
  font-family: monospace;
  background: var(--code-bg, #f1f5f9);
  padding: 0.1rem 0.4rem;
  border-radius: 3px;
}

.computed-val.warning {
  color: #ef4444;
  background: #fef2f2;
}

[data-theme="dark"] .computed-val.warning {
  background: #3b1515;
}

.explain-panel { margin-top: 0.75rem; padding: 0.6rem 0.75rem; background: var(--content-bg); border: 1px solid var(--border-color); border-radius: 6px; font-size: 0.8rem; }
.explain-panel details summary { font-weight: 600; color: var(--primary-color); cursor: pointer; font-size: 0.85rem; }
.explain-content { margin-top: 0.5rem; line-height: 1.6; }
.explain-section { margin-bottom: 0.75rem; }
.explain-section h5 { font-size: 0.82rem; font-weight: 700; margin: 0 0 0.3rem 0; color: var(--text-color); }
.explain-section p { margin: 0.25rem 0; color: var(--text-secondary); }
.explain-section ul { margin: 0.2rem 0; padding-left: 1.2rem; color: var(--text-secondary); }
.explain-section ul li { margin: 0.15rem 0; }
.stability-table { width: 100%; border-collapse: collapse; margin: 0.4rem 0; font-size: 0.76rem; }
.stability-table th, .stability-table td { border: 1px solid var(--border-color); padding: 0.25rem 0.5rem; }
.stability-table th { background: var(--code-bg, #f1f5f9); }
.preset-buttons { display: flex; gap: 0.5rem; flex-wrap: wrap; margin: 0.4rem 0; }
.preset-btn { background: var(--primary-color); color: #fff; border: none; padding: 0.25rem 0.7rem; border-radius: 4px; font-size: 0.76rem; cursor: pointer; transition: opacity 0.15s; }
.preset-btn:hover { opacity: 0.85; }
.preset-hint { font-size: 0.74rem; color: #94a3b8; }
</style>