<script setup lang="ts">
import { ref, watch, onMounted, onUnmounted, nextTick } from 'vue'

// ── 画布引用 ──
const canvasRef = ref<HTMLCanvasElement | null>(null)
let animFrameId = 0

// ── PID 参数 ──
const kp = ref(2.0)
const ki = ref(1.0)
const kd = ref(0.1)

// ── 被控对象参数 ──
const plantK = ref(1.0)
const plantT = ref(1.0)
const plantTau = ref(0.2)

// ── 性能指标 ──
const overshoot = ref(0)
const settlingTime = ref(0)
const steadyStateError = ref(0)
const maxResponse = ref(0)

// ── 仿真参数 ──
const SIM_DT = 0.001        // 仿真步长 [s]
const SIM_TOTAL = 10        // 总仿真时间 [s]
const STEPS = Math.floor(SIM_TOTAL / SIM_DT)  // 总步数
const UMAX = 10             // 控制器输出上限（抗积分饱和）
const UMIN = -10            // 控制器输出下限

// ── Pade 近似的最小 τ 阈值，低于此值按纯一阶系统处理 ──
const TAU_MIN = 0.005

// ── 绘图边距 ──
const MARGIN = { top: 20, right: 20, bottom: 40, left: 55 }

interface SimResult {
  t: Float64Array       // 时间序列
  y: Float64Array       // 输出响应
  overshoot: number     // 超调量 [%]
  settlingTime: number  // 调节时间（2% 带）[s]
  steadyStateError: number // 稳态误差
  maxY: number          // 最大响应值
}

/**
 * 四阶 Runge-Kutta（RK4）数值仿真
 *
 * 被控对象（含一阶 Pade 延迟近似）建模为两个一阶子系统级联：
 *   Gp(s) = K * (1 - τs/2) / ((Ts+1)*(τs/2+1))
 *
 * 子系统1 — 一阶低通（被控对象本体）：
 *   dx1/dt = (K*u - x1) / T
 *
 * 子系统2 — Pade 全通（延迟近似）：
 *   dx2/dt = -(2/τ)*x2 + (4/τ)*x1
 *   输出 y = x2 - x1
 *
 * 当 τ ≈ 0 时退化为纯一阶系统：dx/dt = (K*u - x)/T, y = x
 *
 * 离散 PID 控制器（并行形式，带抗积分饱和）：
 *   e(k) = r - y(k)                     (r=1 单位阶跃)
 *   I(k) = I(k-1) + e(k)*dt             (仅在未饱和时累加)
 *   D(k) = (e(k) - e(k-1)) / dt
 *   u_raw = Kp*e(k) + Ki*I(k) + Kd*D(k)
 *   u(k) = clamp(u_raw, UMIN, UMAX)
 */
function simulate(): SimResult {
  const K  = plantK.value
  const T  = plantT.value
  const tau = plantTau.value
  const Kp = kp.value
  const Ki = ki.value
  const Kd = kd.value
  const dt = SIM_DT
  const N  = STEPS

  // 预分配输出数组
  const tArr = new Float64Array(N + 1)
  const yArr = new Float64Array(N + 1)

  // 系统状态
  let x1 = 0       // 一阶低通状态
  let x2 = 0       // Pade 全通状态
  let y  = 0       // 当前输出

  // PID 控制器状态
  let ePrev = 1    // 上一时刻误差（初始时 r=1, y=0 => e=1）
  let iState = 0   // 积分累加器

  // 是否使用纯一阶模型（τ 极小）
  const useFirstOrder = tau < TAU_MIN
  const invTau2 = useFirstOrder ? 0 : 2 / tau    // 2/τ
  const invTau4 = useFirstOrder ? 0 : 4 / tau    // 4/τ

  // 记录初始值
  tArr[0] = 0
  yArr[0] = 0

  // ── 使用 DOPRI5(4) 自适应步长或固定步长 RK4 ──
  // 此处使用固定步长 RK4，因被控对象为一阶/二阶线性系统，RK4 精度绰绰有余

  // 定义 RK4 用到的导数函数
  // 返回 [dx1/dt, dx2/dt] 或仅 [dx1/dt]（一阶情况）
  function derivatives(x1: number, x2: number, u: number): [number, number] {
    const dx1 = (K * u - x1) / T
    if (useFirstOrder) {
      return [dx1, 0]
    }
    const dx2 = -invTau2 * x2 + invTau4 * x1
    return [dx1, dx2]
  }

  for (let i = 0; i < N; i++) {
    const t = i * dt
    const r = 1  // 单位阶跃给定

    // ── PID 控制器计算 ──
    const e = r - y
    const dTerm = (e - ePrev) / dt
    const uRaw = Kp * e + Ki * iState + Kd * dTerm

    // 抗积分饱和：clamp + 条件积分
    let u: number
    if (uRaw > UMAX) {
      u = UMAX
    } else if (uRaw < UMIN) {
      u = UMIN
    } else {
      u = uRaw
      // 仅未饱和时累加积分
      iState += e * dt
    }

    ePrev = e

    // ── RK4 数值积分（被控对象状态） ──
    // k1
    const [dx1_k1, dx2_k1] = derivatives(x1, x2, u)

    // k2
    const x1_k2 = x1 + 0.5 * dt * dx1_k1
    const x2_k2 = useFirstOrder ? 0 : x2 + 0.5 * dt * dx2_k1
    const [dx1_k2, dx2_k2] = derivatives(x1_k2, x2_k2, u)

    // k3
    const x1_k3 = x1 + 0.5 * dt * dx1_k2
    const x2_k3 = useFirstOrder ? 0 : x2 + 0.5 * dt * dx2_k2
    const [dx1_k3, dx2_k3] = derivatives(x1_k3, x2_k3, u)

    // k4
    const x1_k4 = x1 + dt * dx1_k3
    const x2_k4 = useFirstOrder ? 0 : x2 + dt * dx2_k3
    const [dx1_k4, dx2_k4] = derivatives(x1_k4, x2_k4, u)

    // 状态更新
    x1 += (dt / 6) * (dx1_k1 + 2 * dx1_k2 + 2 * dx1_k3 + dx1_k4)
    if (!useFirstOrder) {
      x2 += (dt / 6) * (dx2_k1 + 2 * dx2_k2 + 2 * dx2_k3 + dx2_k4)
    }

    // 输出
    y = useFirstOrder ? x1 : (x2 - x1)

    // 记录
    tArr[i + 1] = t + dt
    yArr[i + 1] = y
  }

  // ── 计算性能指标 ──
  let maxY = 0
  for (let i = 0; i <= N; i++) {
    if (yArr[i] > maxY) maxY = yArr[i]
  }

  const overshootVal = maxY > 1 ? (maxY - 1) * 100 : 0
  const steadyErr = Math.abs(yArr[N] - 1)

  // 调节时间 ts（2% 带）：从后往前找到最后离开 2% 带的时刻
  let ts = 0
  for (let i = N; i >= 0; i--) {
    if (Math.abs(yArr[i] - 1) > 0.02) {
      ts = tArr[i]
      break
    }
  }

  return {
    t: tArr,
    y: yArr,
    overshoot: overshootVal,
    settlingTime: ts,
    steadyStateError: steadyErr,
    maxY: maxY,
  }
}

/**
 * Canvas 绘制：单位阶跃响应曲线
 */
function draw() {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  const w = canvas.width
  const h = canvas.height

  // 读取 CSS 变量以支持主题
  const style = getComputedStyle(document.documentElement)
  const bgColor = style.getPropertyValue('--bg-color').trim() || '#f8fafc'
  const textColor = style.getPropertyValue('--text-color').trim() || '#1e293b'
  const textSecondary = style.getPropertyValue('--text-secondary').trim() || '#64748b'
  const borderColor = style.getPropertyValue('--border-color').trim() || '#e2e8f0'
  const primaryColor = style.getPropertyValue('--primary-color').trim() || '#3b82f6'
  const contentBg = style.getPropertyValue('--content-bg').trim() || '#ffffff'

  // 绘图区域
  const plotLeft = MARGIN.left
  const plotRight = w - MARGIN.right
  const plotTop = MARGIN.top
  const plotBottom = h - MARGIN.bottom
  const plotWidth = plotRight - plotLeft
  const plotHeight = plotBottom - plotTop

  // 清空
  ctx.clearRect(0, 0, w, h)
  ctx.fillStyle = contentBg
  ctx.fillRect(0, 0, w, h)

  // ── 执行仿真 ──
  const result = simulate()

  // 更新显示指标
  overshoot.value = result.overshoot
  settlingTime.value = result.settlingTime
  steadyStateError.value = result.steadyStateError
  maxResponse.value = result.maxY

  // ── Y 轴自动缩放 ──
  // 保证至少显示到 1.5，同时包含最大响应
  const yMax = Math.max(1.5, result.maxY * 1.1)
  const yMin = 0

  // ── 辅助坐标转换函数 ──
  const toPixelX = (t: number) => plotLeft + (t / SIM_TOTAL) * plotWidth
  const toPixelY = (val: number) => plotBottom - ((val - yMin) / (yMax - yMin)) * plotHeight

  // ── 绘制网格 ──
  ctx.strokeStyle = borderColor
  ctx.lineWidth = 0.5
  ctx.setLineDash([3, 3])

  // 垂直网格线（每 1s）
  for (let t = 0; t <= SIM_TOTAL; t += 1) {
    const px = toPixelX(t)
    ctx.beginPath()
    ctx.moveTo(px, plotTop)
    ctx.lineTo(px, plotBottom)
    ctx.stroke()
  }

  // 水平网格线
  const yTicks = 5
  for (let i = 0; i <= yTicks; i++) {
    const val = yMin + (yMax - yMin) * (i / yTicks)
    const py = toPixelY(val)
    ctx.beginPath()
    ctx.moveTo(plotLeft, py)
    ctx.lineTo(plotRight, py)
    ctx.stroke()
  }

  ctx.setLineDash([])

  // ── 2% 调节带（0.98 和 1.02） ──
  const bandTop = toPixelY(1.02)
  const bandBottom = toPixelY(0.98)
  ctx.strokeStyle = textSecondary
  ctx.lineWidth = 0.8
  ctx.setLineDash([2, 4])
  ;[bandTop, bandBottom].forEach(py => {
    if (py > plotTop && py < plotBottom) {
      ctx.beginPath()
      ctx.moveTo(plotLeft, py)
      ctx.lineTo(plotRight, py)
      ctx.stroke()
    }
  })
  ctx.setLineDash([])

  // ── 参考线 y=1.0（给定值） ──
  const refY = toPixelY(1.0)
  ctx.strokeStyle = textSecondary
  ctx.lineWidth = 0.5
  ctx.setLineDash([5, 5])
  ctx.beginPath()
  ctx.moveTo(plotLeft, refY)
  ctx.lineTo(plotRight, refY)
  ctx.stroke()
  ctx.setLineDash([])

  // ── 坐标轴 ──
  ctx.strokeStyle = textColor
  ctx.lineWidth = 1.5

  // Y 轴
  ctx.beginPath()
  ctx.moveTo(plotLeft, plotTop)
  ctx.lineTo(plotLeft, plotBottom)
  ctx.stroke()

  // X 轴
  const xAxisY = toPixelY(0)
  ctx.beginPath()
  ctx.moveTo(plotLeft, xAxisY)
  ctx.lineTo(plotRight, xAxisY)
  ctx.stroke()

  // ── 坐标轴标签 ──
  ctx.fillStyle = textColor
  ctx.font = '11px sans-serif'
  ctx.textAlign = 'center'

  // X 轴刻度（每秒）
  for (let t = 0; t <= SIM_TOTAL; t += 2) {
    const px = toPixelX(t)
    ctx.fillText(t.toString(), px, plotBottom + 16)
  }
  // X 轴单位
  ctx.fillText('时间 (s)', plotLeft + plotWidth / 2, h - 4)

  // Y 轴刻度
  ctx.textAlign = 'right'
  ctx.fillStyle = textSecondary
  ctx.font = '10px sans-serif'
  for (let i = 0; i <= yTicks; i++) {
    const val = yMin + (yMax - yMin) * (i / yTicks)
    const py = toPixelY(val)
    ctx.fillText(val.toFixed(2), plotLeft - 6, py + 3)
  }

  // ── 绘制响应曲线 ──
  // 降采样以提高绘制性能（每 4 个点取 1 个，约 2500 个线段）
  const DECIMATE = 4

  ctx.strokeStyle = primaryColor
  ctx.lineWidth = 2
  ctx.beginPath()

  let firstPoint = true
  for (let i = 0; i <= STEPS; i += DECIMATE) {
    const px = toPixelX(result.t[i])
    const py = toPixelY(result.y[i])

    // 边界裁剪，防止线段画出绘图区
    const clippedPY = Math.max(plotTop, Math.min(plotBottom, py))

    if (firstPoint) {
      ctx.moveTo(px, clippedPY)
      firstPoint = false
    } else {
      ctx.lineTo(px, clippedPY)
    }
  }
  ctx.stroke()

  // ── 图例 ──
  ctx.font = '11px monospace'
  ctx.textAlign = 'left'
  ctx.fillStyle = textColor

  const legendX = plotLeft + 10
  const legendY = plotTop + 15

  // 响应曲线图例
  ctx.fillStyle = primaryColor
  ctx.fillRect(legendX, legendY - 8, 14, 3)
  ctx.fillStyle = textColor
  ctx.fillText('阶跃响应', legendX + 20, legendY)

  // 设定值图例
  ctx.fillStyle = textSecondary
  ctx.setLineDash([5, 5])
  ctx.beginPath()
  ctx.moveTo(legendX, legendY + 10)
  ctx.lineTo(legendX + 14, legendY + 10)
  ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = textColor
  ctx.fillText('给定值', legendX + 20, legendY + 14)

  // 2% 带图例
  ctx.fillStyle = textSecondary
  ctx.setLineDash([2, 4])
  ctx.beginPath()
  ctx.moveTo(legendX, legendY + 28)
  ctx.lineTo(legendX + 14, legendY + 28)
  ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = textColor
  ctx.fillText('±2% 带', legendX + 20, legendY + 32)
}

// ── 防抖 draw，避免滑块快速拖动时过度重绘 ──
let drawTimer: ReturnType<typeof setTimeout> | null = null
function scheduleDraw() {
  if (drawTimer) clearTimeout(drawTimer)
  drawTimer = setTimeout(() => {
    // 在 nextTick 中调用确保 canvas ref 已就绪
    nextTick(() => {
      draw()
    })
  }, 30) // 30ms 防抖 ≈ 33fps 上限
}

// ── 监听所有参数变化，触发重绘 ──
watch([kp, ki, kd, plantK, plantT, plantTau], () => {
  scheduleDraw()
})

// ── 生命周期 ──
onMounted(() => {
  const canvas = canvasRef.value
  if (canvas) {
    canvas.width = 600
    canvas.height = 400
  }
  nextTick(() => {
    draw()
  })

  // resize 时重绘
  const handleResize = () => scheduleDraw()
  window.addEventListener('resize', handleResize)

  // 清理
  onUnmounted(() => {
    window.removeEventListener('resize', handleResize)
    if (drawTimer) clearTimeout(drawTimer)
    if (animFrameId) cancelAnimationFrame(animFrameId)
  })
})
</script>

<template>
  <div class="sim-container">
    <div class="sim-header">
      <h4>PID 参数调节仿真</h4>
      <span class="sim-model">
        Gp(s) = <strong>K</strong> &middot; e<sup>-&tau;s</sup> / (<strong>T</strong>s + 1)
      </span>
    </div>

    <canvas ref="canvasRef" class="sim-canvas"></canvas>

    <div class="sim-params">
      <!-- PID 参数组 -->
      <div class="param-group">
        <span class="param-group-title">PID 参数</span>
        <label>
          <span>K<sub>p</sub></span>
          <input type="range" v-model.number="kp" min="0" max="10" step="0.01" />
          <span class="param-value">{{ kp.toFixed(2) }}</span>
        </label>
        <label>
          <span>K<sub>i</sub></span>
          <input type="range" v-model.number="ki" min="0" max="5" step="0.01" />
          <span class="param-value">{{ ki.toFixed(2) }}</span>
        </label>
        <label>
          <span>K<sub>d</sub></span>
          <input type="range" v-model.number="kd" min="0" max="3" step="0.01" />
          <span class="param-value">{{ kd.toFixed(2) }}</span>
        </label>
      </div>

      <!-- 被控对象参数组 -->
      <div class="param-group">
        <span class="param-group-title">被控对象</span>
        <label>
          <span>K</span>
          <input type="range" v-model.number="plantK" min="0.1" max="5" step="0.01" />
          <span class="param-value">{{ plantK.toFixed(2) }}</span>
        </label>
        <label>
          <span>T</span>
          <input type="range" v-model.number="plantT" min="0.1" max="5" step="0.01" />
          <span class="param-value">{{ plantT.toFixed(2) }}</span>
        </label>
        <label>
          <span>&tau;</span>
          <input type="range" v-model.number="plantTau" min="0" max="2" step="0.01" />
          <span class="param-value">{{ plantTau.toFixed(2) }}</span>
        </label>
      </div>
    </div>

    <!-- 性能指标 -->
    <div class="sim-metrics">
      <div class="metric-item">
        <span class="metric-label">超调量 M<sub>p</sub></span>
        <span class="metric-value" :class="{ 'metric-warn': overshoot > 20 }">
          {{ overshoot.toFixed(1) }}%
        </span>
      </div>
      <div class="metric-item">
        <span class="metric-label">调节时间 t<sub>s</sub> (2%)</span>
        <span class="metric-value">
          {{ settlingTime < SIM_TOTAL ? settlingTime.toFixed(2) + ' s' : '> ' + SIM_TOTAL + ' s' }}
        </span>
      </div>
      <div class="metric-item">
        <span class="metric-label">稳态误差 e<sub>ss</sub></span>
        <span class="metric-value" :class="{ 'metric-warn': steadyStateError > 0.05 }">
          {{ steadyStateError.toFixed(3) }}
        </span>
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
  margin-bottom: 0.75rem;
  flex-wrap: wrap;
  gap: 0.5rem;
}

.sim-header h4 {
  margin: 0;
  font-size: 0.95rem;
  color: var(--primary-color);
}

.sim-model {
  font-size: 0.78rem;
  color: var(--text-secondary);
  font-family: 'Times New Roman', serif;
  font-style: italic;
}

.sim-model strong {
  color: var(--text-color);
  font-style: normal;
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
  width: 22px;
  text-align: right;
  color: var(--text-secondary);
  font-weight: 500;
}

.param-group input[type="range"] {
  width: 100px;
  accent-color: var(--primary-color);
}

.param-value {
  width: 36px;
  text-align: right;
  font-family: 'Consolas', 'Courier New', monospace;
  color: var(--text-color);
  font-size: 0.78rem;
}

/* ── 性能指标 ── */
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
</style>