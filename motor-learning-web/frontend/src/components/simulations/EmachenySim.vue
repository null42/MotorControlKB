<script setup lang="ts">
import { ref, computed, nextTick, watch } from 'vue'
import axios from 'axios'

interface SimResult {
  success: boolean
  time: number[]
  id: number[]
  iq: number[]
  rpm: number[]
  ud: number[]
  uq: number[]
  KE_identified: number
  Js_identified: number
  total_points: number
}

const canvasRef = ref<HTMLCanvasElement | null>(null)
const CANVAS_W = 800
const CANVAS_H = 600
const PLOT_COUNT = 4
const MARGIN = { top: 8, right: 50, bottom: 35, left: 55 }

const R = ref(0.45)
const L = ref(0.00415)
const KE = ref(0.504)
const Js = ref(0.06)
const Vdc = ref(300)
const speedRef = ref(500)
const curKp = ref(15)
const curTi = ref(0.080)
const steps = ref(50000)

const loading = ref(false)
const errorMsg = ref('')
const result = ref<SimResult | null>(null)
const showUd = ref(true)

const plotConfigs = computed(() => {
  const base = [
    { key: 'id',  label: 'd 轴电流 i_d [A]',     color: '#3b82f6', unit: 'A' },
    { key: 'iq',  label: 'q 轴电流 i_q [A]',     color: '#ef4444', unit: 'A' },
    { key: 'rpm', label: '转速 [rpm]',            color: '#10b981', unit: 'rpm' },
  ]
  const voltageLabel = showUd.value ? 'd 轴电压 u_d [V]' : 'q 轴电压 u_q [V]'
  const voltageColor = showUd.value ? '#f59e0b' : '#8b5cf6'
  const voltageKey = showUd.value ? 'ud' : 'uq'
  base.push({ key: voltageKey, label: voltageLabel, color: voltageColor, unit: 'V' })
  return base
})

function getPlotBounds(plotIdx: number) {
  const plotHeight = (CANVAS_H - MARGIN.top - MARGIN.bottom) / PLOT_COUNT
  const y0 = MARGIN.top + plotIdx * plotHeight
  return {
    top: y0 + 4,
    bottom: y0 + plotHeight - 4,
    left: MARGIN.left,
    right: CANVAS_W - MARGIN.right,
  }
}

function autoRange(values: number[]): { min: number; max: number } {
  if (!values.length) return { min: -1, max: 1 }
  let min = Infinity, max = -Infinity
  for (const v of values) {
    if (v < min) min = v
    if (v > max) max = v
  }
  if (min === max) {
    return { min: min - 1, max: max + 1 }
  }
  const margin = (max - min) * 0.1
  return { min: min - margin, max: max + margin }
}

function draw() {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  canvas.width = CANVAS_W
  canvas.height = CANVAS_H

  const style = getComputedStyle(document.documentElement)
  const bgColor = style.getPropertyValue('--bg-color').trim() || '#f8fafc'
  const contentBg = style.getPropertyValue('--content-bg').trim() || '#ffffff'
  const textColor = style.getPropertyValue('--text-color').trim() || '#1e293b'
  const textSecondary = style.getPropertyValue('--text-secondary').trim() || '#64748b'
  const borderColor = style.getPropertyValue('--border-color').trim() || '#e2e8f0'

  ctx.clearRect(0, 0, CANVAS_W, CANVAS_H)
  ctx.fillStyle = contentBg
  ctx.fillRect(0, 0, CANVAS_W, CANVAS_H)

  if (!result.value || !result.value.time.length) {
    ctx.fillStyle = textSecondary
    ctx.font = '14px sans-serif'
    ctx.textAlign = 'center'
    ctx.fillText('点击「运行仿真」以查看波形', CANVAS_W / 2, CANVAS_H / 2)
    return
  }

  const r = result.value

  const keyToData: Record<string, number[]> = {
    id: r.id,
    iq: r.iq,
    rpm: r.rpm,
    ud: r.ud,
    uq: r.uq,
  }

  const timeRange = { min: r.time[0], max: r.time[r.time.length - 1] }

  for (let p = 0; p < PLOT_COUNT; p++) {
    const cfg = plotConfigs.value[p]
    const data = keyToData[cfg.key] || []
    const bounds = getPlotBounds(p)
    const yRange = autoRange(data)

    ctx.strokeStyle = borderColor
    ctx.lineWidth = 0.5
    ctx.setLineDash([3, 3])

    for (let i = 0; i <= 4; i++) {
      const py = bounds.bottom - (i / 4) * (bounds.bottom - bounds.top)
      ctx.beginPath()
      ctx.moveTo(bounds.left, py)
      ctx.lineTo(bounds.right, py)
      ctx.stroke()
    }

    ctx.setLineDash([])

    ctx.strokeStyle = textColor
    ctx.lineWidth = 1
    ctx.beginPath()
    ctx.moveTo(bounds.left, bounds.top)
    ctx.lineTo(bounds.left, bounds.bottom)
    ctx.lineTo(bounds.right, bounds.bottom)
    ctx.stroke()

    ctx.strokeStyle = cfg.color
    ctx.lineWidth = 1.5
    ctx.beginPath()
    let first = true
    for (let j = 0; j < data.length; j++) {
      const t = r.time[j]
      const v = data[j]
      if (!t || v === undefined) continue
      const px = bounds.left + ((t - timeRange.min) / (timeRange.max - timeRange.min)) * (bounds.right - bounds.left)
      const py = bounds.bottom - ((v - yRange.min) / (yRange.max - yRange.min)) * (bounds.bottom - bounds.top)
      if (first) {
        ctx.moveTo(px, Math.max(bounds.top, Math.min(bounds.bottom, py)))
        first = false
      } else {
        ctx.lineTo(px, Math.max(bounds.top, Math.min(bounds.bottom, py)))
      }
    }
    ctx.stroke()

    ctx.fillStyle = textColor
    ctx.font = '10px monospace'
    ctx.textAlign = 'left'
    ctx.fillText(cfg.label, bounds.left + 4, bounds.top + 12)

    ctx.fillStyle = textSecondary
    ctx.font = '9px monospace'
    ctx.textAlign = 'right'
    for (let i = 0; i <= 4; i++) {
      const val = yRange.min + (yRange.max - yRange.min) * (i / 4)
      const py = bounds.bottom - (i / 4) * (bounds.bottom - bounds.top)
      ctx.fillText(val.toPrecision(3), bounds.left - 4, py + 3)
    }
  }

  const lastPlot = getPlotBounds(PLOT_COUNT - 1)
  ctx.fillStyle = textSecondary
  ctx.font = '10px sans-serif'
  ctx.textAlign = 'center'
  const tMin = r.time[0]
  const tMax = r.time[r.time.length - 1]
  for (let i = 0; i <= 4; i++) {
    const t = tMin + (tMax - tMin) * (i / 4)
    const px = lastPlot.left + (i / 4) * (lastPlot.right - lastPlot.left)
    ctx.fillText(t.toFixed(2), px, lastPlot.bottom + 12)
  }
  ctx.fillText('时间 [s]', lastPlot.left + (lastPlot.right - lastPlot.left) / 2, CANVAS_H - 4)
}

async function runSimulation() {
  loading.value = true
  errorMsg.value = ''
  result.value = null

  try {
    const response = await axios.post<SimResult>('/api/simulate', {
      R: R.value,
      Ld: L.value,
      Lq: L.value,
      KE: KE.value,
      Js: Js.value,
      npp: 2,
      Vdc: Vdc.value,
      cur_Kp: curKp.value,
      cur_Ti: curTi.value,
      cur_limit: 400,
      spd_Kp: 0.5,
      spd_Ti: 1.05,
      spd_limit: 8.0,
      speed_ref: speedRef.value,
      steps: steps.value,
      Ts: 0.00025,
      down_sample: 10,
    })
    result.value = response.data
    await nextTick()
    draw()
  } catch (e: any) {
    errorMsg.value = e?.response?.data?.detail || e.message || '仿真执行失败'
  } finally {
    loading.value = false
  }
}

function toggleVoltageChannel() {
  showUd.value = !showUd.value
  if (result.value) {
    nextTick(() => draw())
  }
}

watch([R, L, KE, Js, Vdc, speedRef, curKp, curTi, steps], () => {
  if (result.value) {
    nextTick(() => draw())
  }
})
</script>

<template>
  <div class="sim-container">
    <div class="sim-header">
      <h4>Emacheny SPMSM 辨识仿真</h4>
      <span class="sim-subtitle">ACMSIMC_TUT — 在线参数辨识</span>
    </div>

    <canvas ref="canvasRef" class="sim-canvas"></canvas>

    <div class="sim-params">
      <div class="param-group">
        <span class="param-group-title">电机参数</span>
        <label>
          <span>R [&Omega;]</span>
          <input type="range" v-model.number="R" min="0.1" max="2" step="0.01" />
          <span class="param-value">{{ R.toFixed(2) }}</span>
        </label>
        <label>
          <span>L [H]</span>
          <input type="range" v-model.number="L" min="0.001" max="0.02" step="0.0001" />
          <span class="param-value">{{ L.toFixed(4) }}</span>
        </label>
        <label>
          <span>K<sub>E</sub> [Vs/rad]</span>
          <input type="range" v-model.number="KE" min="0.1" max="1" step="0.001" />
          <span class="param-value">{{ KE.toFixed(3) }}</span>
        </label>
        <label>
          <span>J<sub>s</sub> [kg&middot;m<sup>2</sup>]</span>
          <input type="range" v-model.number="Js" min="0.01" max="0.5" step="0.001" />
          <span class="param-value">{{ Js.toFixed(3) }}</span>
        </label>
      </div>

      <div class="param-group">
        <span class="param-group-title">控制器参数</span>
        <label>
          <span>V<sub>dc</sub> [V]</span>
          <input type="range" v-model.number="Vdc" min="100" max="600" step="1" />
          <span class="param-value">{{ Vdc }}</span>
        </label>
        <label>
          <span>&omega;<sub>ref</sub> [rpm]</span>
          <input type="range" v-model.number="speedRef" min="100" max="2000" step="10" />
          <span class="param-value">{{ speedRef }}</span>
        </label>
        <label>
          <span>K<sub>p,cur</sub></span>
          <input type="range" v-model.number="curKp" min="1" max="50" step="0.5" />
          <span class="param-value">{{ curKp.toFixed(1) }}</span>
        </label>
        <label>
          <span>T<sub>i,cur</sub> [s]</span>
          <input type="range" v-model.number="curTi" min="0.01" max="0.5" step="0.001" />
          <span class="param-value">{{ curTi.toFixed(3) }}</span>
        </label>
        <label>
          <span>仿真步数</span>
          <input type="range" v-model.number="steps" min="10000" max="250000" step="10000" />
          <span class="param-value">{{ steps }}</span>
        </label>
      </div>
    </div>

    <div class="sim-actions">
      <button class="run-btn" :disabled="loading" @click="runSimulation">
        <span v-if="loading" class="spinner"></span>
        {{ loading ? '仿真运行中...' : '▶ 运行仿真' }}
      </button>
      <button class="toggle-btn" :disabled="!result" @click="toggleVoltageChannel">
        {{ showUd ? '切换显示 u_q' : '切换显示 u_d' }}
      </button>
    </div>

    <div v-if="errorMsg" class="error-msg">{{ errorMsg }}</div>

    <div v-if="result" class="sim-metrics">
      <div class="metric-item">
        <span class="metric-label">辨识 K<sub>E</sub> [Vs/rad]</span>
        <span class="metric-value" :class="{ 'metric-warn': Math.abs(result.KE_identified - KE) > KE * 0.2 }">
          {{ result.KE_identified.toFixed(4) }}
        </span>
        <span class="metric-ref">真实值: {{ KE.toFixed(3) }}</span>
      </div>
      <div class="metric-item">
        <span class="metric-label">辨识 J<sub>s</sub> [kg&middot;m<sup>2</sup>]</span>
        <span class="metric-value" :class="{ 'metric-warn': Math.abs(result.Js_identified - Js) > Js * 0.2 }">
          {{ result.Js_identified.toFixed(4) }}
        </span>
        <span class="metric-ref">真实值: {{ Js.toFixed(3) }}</span>
      </div>
      <div class="metric-item">
        <span class="metric-label">总数据点数</span>
        <span class="metric-value">{{ result.total_points }}</span>
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
  align-items: baseline;
  margin-bottom: 0.5rem;
  flex-wrap: wrap;
  gap: 0.5rem;
}

.sim-header h4 {
  margin: 0;
  font-size: 0.95rem;
  color: var(--primary-color);
}

.sim-subtitle {
  font-size: 0.78rem;
  color: var(--text-secondary);
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
  gap: 2rem;
  margin-top: 0.75rem;
  font-size: 0.8rem;
  justify-content: center;
  flex-wrap: wrap;
}

.param-group {
  display: flex;
  flex-direction: column;
  gap: 0.35rem;
  min-width: 220px;
}

.param-group-title {
  font-weight: 600;
  color: var(--text-color);
  font-size: 0.82rem;
  margin-bottom: 0.15rem;
  padding-bottom: 0.2rem;
  border-bottom: 1px solid var(--border-color);
}

.param-group label {
  display: flex;
  align-items: center;
  gap: 0.35rem;
  white-space: nowrap;
}

.param-group label > span:first-child {
  width: 85px;
  text-align: right;
  color: var(--text-secondary);
  font-weight: 500;
  font-size: 0.76rem;
  overflow: hidden;
  text-overflow: ellipsis;
}

.param-group input[type="range"] {
  width: 90px;
  accent-color: var(--primary-color);
}

.param-value {
  width: 42px;
  text-align: right;
  font-family: 'Consolas', 'Courier New', monospace;
  color: var(--text-color);
  font-size: 0.78rem;
}

.sim-actions {
  display: flex;
  gap: 0.75rem;
  margin-top: 0.75rem;
  justify-content: center;
  align-items: center;
}

.run-btn {
  background-color: var(--primary-color);
  color: white;
  border: none;
  padding: 0.5rem 1.5rem;
  border-radius: 6px;
  cursor: pointer;
  font-size: 0.9rem;
  font-weight: 600;
  display: flex;
  align-items: center;
  gap: 0.5rem;
  transition: opacity 0.2s;
}

.run-btn:hover:not(:disabled) {
  opacity: 0.85;
}

.run-btn:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

.spinner {
  width: 14px;
  height: 14px;
  border: 2px solid rgba(255, 255, 255, 0.3);
  border-top-color: white;
  border-radius: 50%;
  animation: spin 0.8s linear infinite;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}

.toggle-btn {
  background: none;
  border: 1px solid var(--border-color);
  padding: 0.5rem 1rem;
  border-radius: 6px;
  cursor: pointer;
  font-size: 0.82rem;
  color: var(--text-color);
  transition: all 0.2s;
}

.toggle-btn:hover:not(:disabled) {
  background-color: var(--bg-color);
  border-color: var(--primary-color);
}

.toggle-btn:disabled {
  opacity: 0.4;
  cursor: not-allowed;
}

.error-msg {
  color: #ef4444;
  text-align: center;
  margin-top: 0.75rem;
  font-size: 0.85rem;
}

.sim-metrics {
  display: flex;
  gap: 1.5rem;
  margin-top: 0.75rem;
  padding-top: 0.6rem;
  border-top: 1px solid var(--border-color);
  font-size: 0.82rem;
  justify-content: center;
  flex-wrap: wrap;
}

.metric-item {
  display: flex;
  align-items: center;
  gap: 0.4rem;
  flex-wrap: wrap;
}

.metric-label {
  color: var(--text-secondary);
}

.metric-value {
  font-weight: 600;
  font-family: 'Consolas', 'Courier New', monospace;
  color: var(--text-color);
  background: var(--code-bg);
  padding: 0.15rem 0.45rem;
  border-radius: 4px;
}

.metric-warn {
  color: #ef4444;
}

.metric-ref {
  font-size: 0.72rem;
  color: var(--text-secondary);
}
</style>