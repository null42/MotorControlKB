<script setup lang="ts">
import { ref, computed, watch, onMounted, nextTick } from 'vue'

// ---- 画布引用 ----
const canvasRef = ref<HTMLCanvasElement | null>(null)

// ---- 电路参数（用户可调） ----
const R = ref(10)       // 电阻，单位 Ω，范围 0.1 ~ 100
const L = ref(10)       // 电感，单位 mH，范围 0.1 ~ 100
const C = ref(100)      // 电容，单位 μF，范围 1 ~ 1000
const Vstep = ref(10)   // 阶跃电压，单位 V，范围 1 ~ 100

// ---- 导出物理量（用于 textInfo 显示） ----
const zeta = computed(() => {
  const lH = L.value / 1000      // mH -> H
  const cF = C.value / 1_000_000 // μF -> F
  if (lH <= 0 || cF <= 0) return 0
  return (R.value / 2) * Math.sqrt(cF / lH)
})

const omegaN = computed(() => {
  const lH = L.value / 1000
  const cF = C.value / 1_000_000
  if (lH <= 0 || cF <= 0) return 0
  return 1 / Math.sqrt(lH * cF)  // rad/s
})

const Q = computed(() => {
  const z = zeta.value
  if (z <= 0) return Infinity
  return 1 / (2 * z)
})

const dampingLabel = computed(() => {
  const z = zeta.value
  if (z > 1.001) return '过阻尼 (ζ > 1)'
  if (z < 0.999) return '欠阻尼 (ζ < 1)'
  return '临界阻尼 (ζ ≈ 1)'
})

const waveformColor = computed(() => {
  const z = zeta.value
  if (z > 1.001) return '#3b82f6' // 蓝：过阻尼
  if (z < 0.999) return '#ef4444' // 红：欠阻尼
  return '#22c55e'                // 绿：临界阻尼
})

// ---- 显示用格式化 ----
const fmtZeta = computed(() => zeta.value.toFixed(3))
const fmtOmegaN = computed(() => {
  const v = omegaN.value
  if (v >= 1000) return (v / 1000).toFixed(2) + ' k'
  return v.toFixed(0)
})
const fmtQ = computed(() => {
  const q = Q.value
  if (!isFinite(q)) return '∞'
  return q.toFixed(2)
})

// ---- 解析 Vc(t) 在若干 t 点的值 ----
const tau = computed(() => {
  // τ = 2L/R，用于估算时间轴范围
  if (R.value <= 0.001) return 0.001 // 防止除零
  return (2 * L.value / 1000) / R.value
})

const timeMax = computed(() => {
  // 时间轴最大值 = 5 * τ，但至少保留一个合理下限
  return Math.max(5 * tau.value, 0.0001)
})

/**
 * 计算 Vc(t) —— 零初始条件 (Vc(0)=0, i(0)=0)
 * 使用用户指定的三区公式
 */
function vcAtT(t: number): number {
  const lH = L.value / 1000
  const cF = C.value / 1_000_000
  const z = zeta.value
  const wn = omegaN.value
  const Vs = Vstep.value

  if (wn <= 0 || lH <= 0 || cF <= 0) return 0
  if (t <= 0) return 0

  const expTerm = Math.exp(-z * wn * t)

  if (z > 1.001) {
    // 过阻尼
    const wd = wn * Math.sqrt(z * z - 1)
    const coshWd = Math.cosh(wd * t)
    const sinhWd = Math.sinh(wd * t)
    return Vs * (1 - expTerm * (coshWd + (z / Math.sqrt(z * z - 1)) * sinhWd))
  }

  if (z < 0.999) {
    // 欠阻尼
    const wd = wn * Math.sqrt(1 - z * z)
    const cosWd = Math.cos(wd * t)
    const sinWd = Math.sin(wd * t)
    return Vs * (1 - expTerm * (cosWd + (z / Math.sqrt(1 - z * z)) * sinWd))
  }

  // 临界阻尼
  return Vs * (1 - (1 + wn * t) * expTerm)
}

// ---- Canvas 绘制 ----
function draw() {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  const W = canvas.width
  const H = canvas.height

  // 边距（留给坐标轴和标签）
  const margin = { top: 40, right: 30, bottom: 50, left: 60 }
  const pw = W - margin.left - margin.right
  const ph = H - margin.top - margin.bottom

  // ---- 清空并填充背景 ----
  const style = getComputedStyle(document.documentElement)
  const bgColor = style.getPropertyValue('--bg-color').trim() || '#f8fafc'
  const textColor = style.getPropertyValue('--text-color').trim() || '#1e293b'
  const gridColor = style.getPropertyValue('--border-color').trim() || '#e2e8f0'

  ctx.clearRect(0, 0, W, H)
  ctx.fillStyle = bgColor
  ctx.fillRect(0, 0, W, H)

  // ---- 坐标映射 ----
  const tMax = timeMax.value
  // y 轴范围：0 ~ Vstep * 1.15（留 15% headroom 用于过冲）
  const vMin = 0
  const vMax = Vstep.value * 1.15

  function toX(t: number) { return margin.left + (t / tMax) * pw }
  function toY(v: number) { return margin.top + ph - ((v - vMin) / (vMax - vMin)) * ph }

  // ---- 网格线 ----
  ctx.strokeStyle = gridColor
  ctx.lineWidth = 0.5
  const gridLinesT = 5
  const gridLinesV = 5
  for (let i = 0; i <= gridLinesT; i++) {
    const x = margin.left + (pw * i) / gridLinesT
    ctx.beginPath()
    ctx.moveTo(x, margin.top)
    ctx.lineTo(x, margin.top + ph)
    ctx.stroke()
  }
  for (let i = 0; i <= gridLinesV; i++) {
    const y = margin.top + (ph * i) / gridLinesV
    ctx.beginPath()
    ctx.moveTo(margin.left, y)
    ctx.lineTo(margin.left + pw, y)
    ctx.stroke()
  }

  // ---- 坐标轴 ----
  ctx.strokeStyle = textColor
  ctx.lineWidth = 1.5
  // X 轴
  ctx.beginPath()
  ctx.moveTo(margin.left, margin.top + ph)
  ctx.lineTo(margin.left + pw, margin.top + ph)
  ctx.stroke()
  // Y 轴
  ctx.beginPath()
  ctx.moveTo(margin.left, margin.top)
  ctx.lineTo(margin.left, margin.top + ph)
  ctx.stroke()

  // ---- 轴标签 ----
  ctx.fillStyle = textColor
  ctx.font = '11px sans-serif'
  ctx.textAlign = 'center'

  // X 轴时间刻度
  for (let i = 0; i <= gridLinesT; i++) {
    const t = (tMax * i) / gridLinesT
    const label = t >= 1 ? t.toFixed(1) + 's' : (t * 1000).toFixed(1) + 'ms'
    ctx.fillText(label, toX(t), margin.top + ph + 16)
  }
  ctx.fillText('时间 t', margin.left + pw / 2, margin.top + ph + 34)

  // Y 轴电压刻度
  ctx.textAlign = 'right'
  for (let i = 0; i <= gridLinesV; i++) {
    const v = vMin + ((vMax - vMin) * i) / gridLinesV
    ctx.fillText(v.toFixed(1) + 'V', margin.left - 8, toY(v) + 4)
  }
  ctx.save()
  ctx.translate(14, margin.top + ph / 2)
  ctx.rotate(-Math.PI / 2)
  ctx.textAlign = 'center'
  ctx.fillText('Vc(t)', 0, 0)
  ctx.restore()

  // ---- Vstep 参考虚线 ----
  const yVstep = toY(Vstep.value)
  ctx.strokeStyle = '#94a3b8'
  ctx.lineWidth = 1
  ctx.setLineDash([5, 5])
  ctx.beginPath()
  ctx.moveTo(margin.left, yVstep)
  ctx.lineTo(margin.left + pw, yVstep)
  ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#94a3b8'
  ctx.textAlign = 'left'
  ctx.font = '10px sans-serif'
  ctx.fillText('Vstep = ' + Vstep.value + 'V', margin.left + 4, yVstep - 4)

  // ---- 绘制 Vc(t) 曲线 ----
  const color = waveformColor.value
  ctx.strokeStyle = color
  ctx.lineWidth = 2.5
  ctx.beginPath()

  const N = 600 // 采样点数
  let firstPoint = true
  for (let i = 0; i <= N; i++) {
    const t = (tMax * i) / N
    const vc = vcAtT(t)
    const x = toX(t)
    const y = toY(vc)

    // 裁剪到绘图区域
    const clampedY = Math.max(margin.top, Math.min(margin.top + ph, y))

    if (firstPoint) {
      ctx.moveTo(x, clampedY)
      firstPoint = false
    } else {
      ctx.lineTo(x, clampedY)
    }
  }
  ctx.stroke()

  // ---- 波形顶端信息标注 ----
  ctx.fillStyle = textColor
  ctx.font = 'bold 11px monospace'
  ctx.textAlign = 'left'
  const infoY = margin.top + 15
  ctx.fillText(
    `ζ=${fmtZeta.value}  ωn=${fmtOmegaN.value} rad/s  Q=${fmtQ.value}`,
    margin.left + 6,
    infoY
  )
  ctx.fillStyle = color
  ctx.fillText(dampingLabel.value, margin.left + 6, infoY + 16)
}

// ---- 参数变化时重绘 ----
watch([R, L, C, Vstep], () => {
  nextTick(draw)
})

onMounted(() => {
  const canvas = canvasRef.value
  if (canvas) {
    canvas.width = 600
    canvas.height = 400
  }
  nextTick(draw)
})
</script>

<template>
  <div class="sim-container">
    <div class="sim-header">
      <h4>RLC 充放电仿真</h4>
    </div>
    <canvas ref="canvasRef" class="sim-canvas"></canvas>
    <div class="sim-params">
      <label>
        R: {{ R }} Ω
        <input type="range" v-model.number="R" min="0.1" max="100" step="0.1" />
      </label>
      <label>
        L: {{ L }} mH
        <input type="range" v-model.number="L" min="0.1" max="100" step="0.1" />
      </label>
      <label>
        C: {{ C }} μF
        <input type="range" v-model.number="C" min="1" max="1000" step="1" />
      </label>
      <label>
        Vstep: {{ Vstep }} V
        <input type="range" v-model.number="Vstep" min="1" max="100" step="1" />
      </label>
    </div>
    <div class="sim-info">
      <span>阻尼比 ζ = {{ fmtZeta }}</span>
      <span>自然频率 ωn = {{ fmtOmegaN }} rad/s</span>
      <span>品质因数 Q = {{ fmtQ }}</span>
      <span :style="{ color: waveformColor, fontWeight: 'bold' }">{{ dampingLabel }}</span>
    </div>
    <div class="explain-panel">
      <details>
        <summary>📖 教学说明 — 理解 RLC 电路响应</summary>
        <div class="explain-content">
          <div class="explain-section">
            <h5>电路原理</h5>
            <p>本仿真模拟 <strong>串联 RLC 电路</strong>在阶跃电压输入下的电容电压响应 Vc(t)。</p>
            <p><strong>阻尼比计算公式</strong>：<code>ζ = (R/2) · √(C/L)</code></p>
            <ul>
              <li><strong>增大 R</strong> → ζ 增大 → 阻尼增强 → 振荡衰减更快</li>
              <li><strong>增大 L</strong> → ζ 减小 → 阻尼减弱 → 更容易振荡</li>
              <li><strong>增大 C</strong> → ζ 增大、ωn 减小 → 响应变慢</li>
            </ul>
          </div>
          <div class="explain-section">
            <h5>三种阻尼状态对比</h5>
            <table class="damping-table">
              <thead><tr><th>状态</th><th>条件</th><th>波形特征</th><th>稳态建立</th></tr></thead>
              <tbody>
                <tr><td><span class="tag over">过阻尼</span></td><td>ζ &gt; 1</td><td>单调上升，无振荡</td><td>最慢</td></tr>
                <tr><td><span class="tag critical">临界阻尼</span></td><td>ζ = 1</td><td>单调上升，无振荡</td><td>最快（无超调）</td></tr>
                <tr><td><span class="tag under">欠阻尼</span></td><td>ζ &lt; 1</td><td>衰减振荡，有超调</td><td>较快，但有振荡</td></tr>
              </tbody>
            </table>
            <p><strong>品质因数 Q = 1/(2ζ)</strong>：Q 越高，谐振峰值越大，能量损耗越小。</p>
          </div>
          <div class="explain-section">
            <h5>电机控制中的应用</h5>
            <ul>
              <li><strong>Buck 变换器 LC 输出滤波</strong>：设计时需保证 ζ 足够大（通常 ≥ 0.5），避免输出电压振荡。过小的输出电容或过大的滤波电感会导致 ζ 过小。</li>
              <li><strong>EMI 滤波</strong>：输入端的 LC 滤波器在特定频率可能谐振，需加阻尼电阻或 RC 吸收网络。</li>
              <li><strong>LLC 谐振变换器</strong>：利用 LC 谐振实现 ZVS（零电压开关），工作在欠阻尼区（Q 值由负载决定）。</li>
              <li><strong>电流采样滤波</strong>：电机相电流采样后的 RC 低通滤波，截止频率 fc = 1/(2πRC)，需远高于电流环带宽。</li>
            </ul>
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
}

.sim-params {
  display: flex;
  gap: 1rem;
  margin-top: 0.75rem;
  font-size: 0.8rem;
  justify-content: center;
  flex-wrap: wrap;
}

.sim-params label {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.25rem;
  min-width: 80px;
}

.sim-params input[type='range'] {
  width: 100px;
  cursor: pointer;
}

.sim-info {
  display: flex;
  gap: 1.25rem;
  margin-top: 0.75rem;
  font-size: 0.8rem;
  justify-content: center;
  flex-wrap: wrap;
  color: var(--text-color);
}
.explain-panel { margin-top: 0.75rem; padding: 0.6rem 0.75rem; background: var(--content-bg); border: 1px solid var(--border-color); border-radius: 6px; font-size: 0.8rem; }
.explain-panel details summary { font-weight: 600; color: var(--primary-color); cursor: pointer; font-size: 0.85rem; }
.explain-content { margin-top: 0.5rem; line-height: 1.6; }
.explain-section { margin-bottom: 0.7rem; }
.explain-section h5 { font-size: 0.82rem; font-weight: 700; margin: 0 0 0.3rem 0; color: var(--text-color); }
.explain-section p { margin: 0.2rem 0; color: var(--text-secondary); }
.explain-section ul { margin: 0.2rem 0; padding-left: 1.2rem; color: var(--text-secondary); }
.explain-section ul li { margin: 0.12rem 0; }
.damping-table { width: 100%; border-collapse: collapse; margin: 0.4rem 0; font-size: 0.76rem; }
.damping-table th, .damping-table td { border: 1px solid var(--border-color); padding: 0.25rem 0.5rem; text-align: left; }
.damping-table th { background: var(--code-bg, #f1f5f9); }
.tag { display: inline-block; padding: 0.1rem 0.4rem; border-radius: 3px; font-weight: 600; font-size: 0.72rem; }
.tag.over { background: #dbeafe; color: #2563eb; }
.tag.critical { background: #dcfce7; color: #16a34a; }
.tag.under { background: #fef2f2; color: #dc2626; }
</style>