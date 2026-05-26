<script setup lang="ts">
import { ref, onMounted, onUnmounted } from 'vue'

const canvasRef = ref<HTMLCanvasElement | null>(null)
const theta = ref(0)
const ia = ref(10)
const ib = ref(-5)
const ic = ref(-5)
const isRunning = ref(true)
const speed = ref(1)
let animFrameId = 0

const toRad = (deg: number) => deg * Math.PI / 180

const draw = () => {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  const w = canvas.width
  const h = canvas.height
  const cx = w / 2
  const cy = h / 2
  const r = Math.min(w, h) * 0.3

  ctx.clearRect(0, 0, w, h)

  const bgColor = getComputedStyle(document.documentElement).getPropertyValue('--bg-color').trim() || '#f8fafc'
  ctx.fillStyle = bgColor
  ctx.fillRect(0, 0, w, h)

  const thetaRad = toRad(theta.value)

  const axes = [
    { angle: 0, label: 'A', color: '#ef4444' },
    { angle: toRad(120), label: 'B', color: '#22c55e' },
    { angle: toRad(240), label: 'C', color: '#3b82f6' },
  ]

  ctx.lineWidth = 1
  ctx.strokeStyle = '#94a3b8'
  axes.forEach(axis => {
    ctx.beginPath()
    ctx.moveTo(cx, cy)
    ctx.lineTo(cx + r * 1.2 * Math.cos(axis.angle), cy - r * 1.2 * Math.sin(axis.angle))
    ctx.stroke()
    ctx.fillStyle = axis.color
    ctx.font = '14px sans-serif'
    ctx.fillText(axis.label, cx + r * 1.3 * Math.cos(axis.angle) - 5, cy - r * 1.3 * Math.sin(axis.angle) + 5)
  })

  ctx.strokeStyle = '#64748b'
  ctx.setLineDash([4, 4])
  ctx.beginPath()
  ctx.moveTo(cx - r * 1.3, cy)
  ctx.lineTo(cx + r * 1.3, cy)
  ctx.stroke()
  ctx.beginPath()
  ctx.moveTo(cx, cy - r * 1.3)
  ctx.lineTo(cx, cy + r * 1.3)
  ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#64748b'
  ctx.font = '12px sans-serif'
  ctx.fillText('α', cx + r * 1.35, cy + 4)
  ctx.fillText('β', cx + 4, cy - r * 1.35)

  ctx.strokeStyle = '#f59e0b'
  ctx.lineWidth = 2
  ctx.beginPath()
  ctx.moveTo(cx - r * 1.1 * Math.cos(thetaRad), cy + r * 1.1 * Math.sin(thetaRad))
  ctx.lineTo(cx + r * 1.1 * Math.cos(thetaRad), cy - r * 1.1 * Math.sin(thetaRad))
  ctx.stroke()
  ctx.strokeStyle = '#a855f7'
  ctx.beginPath()
  ctx.moveTo(cx + r * 1.1 * Math.sin(thetaRad), cy + r * 1.1 * Math.cos(thetaRad))
  ctx.lineTo(cx - r * 1.1 * Math.sin(thetaRad), cy - r * 1.1 * Math.cos(thetaRad))
  ctx.stroke()
  ctx.fillStyle = '#f59e0b'
  ctx.font = 'bold 13px sans-serif'
  ctx.fillText('d', cx + r * 1.2 * Math.cos(thetaRad) - 4, cy - r * 1.2 * Math.sin(thetaRad) + 4)
  ctx.fillStyle = '#a855f7'
  ctx.fillText('q', cx - r * 1.2 * Math.sin(thetaRad) - 4, cy - r * 1.2 * Math.cos(thetaRad) + 4)

  const iAlpha = ia.value
  const iBeta = (ib.value - ic.value) / Math.sqrt(3)
  const scale = r / 15
  const endX = cx + iAlpha * scale
  const endY = cy - iBeta * scale

  ctx.strokeStyle = '#ef4444'
  ctx.lineWidth = 2
  ctx.beginPath()
  ctx.moveTo(cx, cy)
  ctx.lineTo(endX, endY)
  ctx.stroke()
  ctx.fillStyle = '#ef4444'
  ctx.beginPath()
  ctx.arc(endX, endY, 4, 0, Math.PI * 2)
  ctx.fill()

  const cosT = Math.cos(thetaRad)
  const sinT = Math.sin(thetaRad)
  const id = iAlpha * cosT + iBeta * sinT
  const iq = -iAlpha * sinT + iBeta * cosT

  const dEndX = cx + id * scale * cosT
  const dEndY = cy - id * scale * sinT
  const qEndX = cx - iq * scale * sinT
  const qEndY = cy - iq * scale * cosT

  ctx.strokeStyle = '#f59e0b'
  ctx.lineWidth = 2
  ctx.setLineDash([3, 3])
  ctx.beginPath()
  ctx.moveTo(cx, cy)
  ctx.lineTo(dEndX, dEndY)
  ctx.stroke()
  ctx.strokeStyle = '#a855f7'
  ctx.beginPath()
  ctx.moveTo(cx, cy)
  ctx.lineTo(qEndX, qEndY)
  ctx.stroke()
  ctx.setLineDash([])

  const textColor = getComputedStyle(document.documentElement).getPropertyValue('--text-color').trim() || '#1e293b'
  ctx.fillStyle = textColor
  ctx.font = '12px monospace'
  ctx.fillText(`θ = ${theta.value.toFixed(1)}°`, 10, 20)
  ctx.fillText(`iα = ${iAlpha.toFixed(2)}  iβ = ${iBeta.toFixed(2)}`, 10, 38)
  ctx.fillText(`id = ${id.toFixed(2)}  iq = ${iq.toFixed(2)}`, 10, 56)
}

const animate = () => {
  if (isRunning.value) {
    theta.value = (theta.value + speed.value) % 360
  }
  draw()
  animFrameId = requestAnimationFrame(animate)
}

onMounted(() => {
  const canvas = canvasRef.value
  if (canvas) {
    canvas.width = 400
    canvas.height = 400
  }
  animate()
})

onUnmounted(() => {
  cancelAnimationFrame(animFrameId)
})
</script>

<template>
  <div class="sim-container">
    <div class="sim-header">
      <h4>Park 变换可视化</h4>
      <div class="sim-controls">
        <button @click="isRunning = !isRunning">{{ isRunning ? '暂停' : '播放' }}</button>
        <label>速度: <input type="range" v-model.number="speed" min="0.1" max="5" step="0.1" /></label>
        <label>θ: <input type="range" v-model.number="theta" min="0" max="360" step="1" /></label>
      </div>
    </div>
    <canvas ref="canvasRef" class="sim-canvas"></canvas>
    <div class="sim-params">
      <label>ia: <input type="number" v-model.number="ia" step="0.5" /></label>
      <label>ib: <input type="number" v-model.number="ib" step="0.5" /></label>
      <label>ic: <input type="number" v-model.number="ic" step="0.5" /></label>
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

.sim-controls {
  display: flex;
  gap: 0.75rem;
  align-items: center;
  font-size: 0.8rem;
  flex-wrap: wrap;
}

.sim-controls button {
  padding: 0.25rem 0.75rem;
  border: 1px solid var(--border-color);
  border-radius: 4px;
  background: var(--content-bg);
  cursor: pointer;
  font-size: 0.8rem;
}

.sim-controls button:hover {
  background: var(--primary-color);
  color: white;
}

.sim-controls input[type="range"] {
  width: 80px;
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
}

.sim-params input {
  width: 60px;
  padding: 0.2rem 0.4rem;
  border: 1px solid var(--border-color);
  border-radius: 4px;
  font-size: 0.8rem;
}
</style>
