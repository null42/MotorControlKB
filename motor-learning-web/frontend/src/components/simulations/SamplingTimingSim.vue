<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted } from 'vue'

/**
 * ADC 电流采样时序仿真
 * 说明为何在 PWM 周期中心采样电流能避开开关噪声
 *
 * Canvas 920×500 三段布局：
 *   上段 (0~240): 中心对齐三角波 + Tcmp 阈值 + PWMA/B/C
 *   中段 (250~380): ADC 采样触发时机 — 顶点采样 vs 底点采样
 *   下段 (390~480): 相电流波形 + 开关噪声尖峰 + 干净采样点
 */

const canvasRef = ref<HTMLCanvasElement | null>(null)
let animFrameId = 0

const pwmFreq = ref(10)           // kHz, 默认10kHz
const deadTimeNs = ref(2000)     // ns
const showNoise = ref(true)      // 是否显示开关噪声
const highlightSafe = ref(true)  // 高亮安全采样窗口

// Tcmp 值 (三个比较阈值, 归一化到 [0, 0.5])
const tcmpVals = computed(() => ({
  A: 0.32,
  B: 0.24,
  C: 0.16,
}))

const toRad = (deg: number) => deg * Math.PI / 180

// ═════════════════════════ 绘图 ═════════════════════════
const draw = () => {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  const W = 920, H = 500
  canvas.width = W; canvas.height = H

  const style = getComputedStyle(document.documentElement)
  const bgColor   = style.getPropertyValue('--bg-color').trim() || '#f8fafc'
  const textColor = style.getPropertyValue('--text-color').trim() || '#1e293b'
  const borderColor = style.getPropertyValue('--border-color').trim() || '#e2e8f0'
  const primaryColor = style.getPropertyValue('--primary-color').trim() || '#3b82f6'
  const textSec = style.getPropertyValue('--text-secondary').trim() || '#64748b'

  ctx.clearRect(0, 0, W, H)
  ctx.fillStyle = bgColor; ctx.fillRect(0, 0, W, H)

  // 水平分隔线
  const SEP1_Y = 245, SEP2_Y = 385
  ctx.strokeStyle = borderColor; ctx.lineWidth = 1; ctx.setLineDash([8, 4])
  ctx.beginPath(); ctx.moveTo(15, SEP1_Y); ctx.lineTo(W - 15, SEP1_Y); ctx.stroke()
  ctx.beginPath(); ctx.moveTo(15, SEP2_Y); ctx.lineTo(W - 15, SEP2_Y); ctx.stroke()
  ctx.setLineDash([])

  const Ts_us = 1000 / pwmFreq.value        // 开关周期 μs
  const L = 50, R = W - 50                  // 绘图左右边界
  const DW = R - L

  // ═══════════════════════════════════════════════════════
  //  上段: 中心对齐三角波 + Tcmp + 三相PWM (0~240)
  // ═══════════════════════════════════════════════════════
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText(`中心对齐 PWM 三角载波 (Ts = ${Ts_us.toFixed(0)} μs, f = ${pwmFreq} kHz)`, W / 2, 20)

  const triTop = 36, triH = 94, triBot = triTop + triH
  const triPeak = 0.5

  // 三角载波
  const N_TRI = 400
  ctx.strokeStyle = '#8b5cf6'; ctx.lineWidth = 1.8
  ctx.beginPath()
  for (let i = 0; i <= N_TRI; i++) {
    const frac = i / N_TRI
    const tv = frac < 0.5 ? frac * triPeak * 2 : (1 - frac) * triPeak * 2
    const x = L + frac * DW
    const y = triBot - (tv / triPeak) * triH
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
  }
  ctx.stroke()
  ctx.fillStyle = '#8b5cf6'; ctx.font = '10px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText('Ts/2', L + DW / 2 - 5, triTop - 3)
  ctx.fillText('0', L - 5, triBot + 12)
  ctx.fillText('Ts', R, triBot + 12)

  // Tcmp 阈值线
  const tcmp = tcmpVals.value
  const tcmEntries = [
    { val: tcmp.A, color: '#ef4444', name: 'TcmpA' },
    { val: tcmp.B, color: '#10b981', name: 'TcmpB' },
    { val: tcmp.C, color: '#3b82f6', name: 'TcmpC' },
  ]

  tcmEntries.forEach(tc => {
    const ty = triBot - (tc.val / triPeak) * triH
    ctx.strokeStyle = tc.color; ctx.lineWidth = 1; ctx.setLineDash([4, 3])
    ctx.beginPath(); ctx.moveTo(L, ty); ctx.lineTo(R, ty); ctx.stroke()
    ctx.setLineDash([])
    // 比较点标注
    const riseFrac = tc.val / triPeak * 0.5
    const riseX = L + riseFrac * DW
    ctx.fillStyle = tc.color; ctx.beginPath(); ctx.arc(riseX, ty, 4.5, 0, 2 * Math.PI); ctx.fill()
    ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'left'
    ctx.fillText(tc.name, riseX + 8, ty - 4)
  })

  // PWM 输出波形 (三相)
  const pwmPhases = [
    { name: 'PWMA', color: '#ef4444' },
    { name: 'PWMB', color: '#10b981' },
    { name: 'PWMC', color: '#3b82f6' },
  ]
  const pwmLabelW = 52
  const pwmDL = L + pwmLabelW, pwmDWFinal = DW - pwmLabelW
  const pwmRowH = 32

  pwmPhases.forEach((pp, pi) => {
    const rowTop = triBot + 12 + pi * pwmRowH
    const rowMid = rowTop + pwmRowH / 2

    ctx.fillStyle = pp.color; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'right'
    ctx.fillText(pp.name, pwmDL - 8, rowMid + 4)

    ctx.strokeStyle = borderColor; ctx.lineWidth = 0.5
    ctx.strokeRect(pwmDL, rowTop, pwmDWFinal, pwmRowH)

    const tcmVal = [tcmp.A, tcmp.B, tcmp.C][pi]
    // 填充高电平区间
    ctx.fillStyle = pp.color + '55'
    for (let i = 0; i < N_TRI; i++) {
      const frac = i / N_TRI
      const tv = frac < 0.5 ? frac * triPeak * 2 : (1 - frac) * triPeak * 2
      if (tv > tcmVal) {
        const px = pwmDL + frac * pwmDWFinal
        const pxNext = pwmDL + ((i + 1) / N_TRI) * pwmDWFinal
        ctx.fillRect(px, rowTop + 3, Math.max(1, pxNext - px), pwmRowH - 6)
      }
    }
  })

  // 死区标注
  if (deadTimeNs.value > 0) {
    const deadFrac = deadTimeNs.value * 1e-3 / Ts_us
    const deadPx = deadFrac * DW
    const deadNoteY = triBot + 12 + 3 * pwmRowH + 14
    ctx.fillStyle = '#ef4444'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'center'
    ctx.fillText(`死区 Tdead = ${deadTimeNs}ns (占比 ${(deadFrac * 100).toFixed(1)}%)`, W / 2, deadNoteY)
    // 死区示意小方块
    if (deadPx > 2) {
      const riseFrac = tcmp.A / triPeak * 0.5
      const riseX = pwmDL + riseFrac * pwmDWFinal
      ctx.fillStyle = 'rgba(239,68,68,0.3)'; ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 0.8
      ctx.fillRect(riseX, triBot + 12, deadPx, 3 * pwmRowH - 4)
      ctx.strokeRect(riseX, triBot + 12, deadPx, 3 * pwmRowH - 4)
      ctx.fillStyle = '#ef4444'; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'left'
      ctx.fillText('死区', riseX + deadPx / 2 - 12, triBot + 12 + 3 * pwmRowH / 2)
    }
  }

  // ═══════════════════════════════════════════════════════
  //  中段: ADC 采样触发时机 (250~380)
  // ═══════════════════════════════════════════════════════
  const midTop = SEP1_Y + 8
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('ADC 电流采样触发策略', W / 2, midTop + 14)

  // 方案1: 顶点采样 (推荐) — 画在三角波底部
  const stratTop = midTop + 26, stratH = 90

  // 简化三角波
  ctx.strokeStyle = 'rgba(139,92,246,0.4)'; ctx.lineWidth = 1.2
  ctx.beginPath()
  for (let i = 0; i <= N_TRI; i++) {
    const frac = i / N_TRI
    const tv = frac < 0.5 ? frac * triPeak * 2 : (1 - frac) * triPeak * 2
    const x = L + frac * DW
    const y = stratTop + stratH - (tv / triPeak) * stratH
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
  }
  ctx.stroke()

  // 安全采样窗口 (绿色区域: 三角波峰附近 = 零矢量中心)
  if (highlightSafe.value) {
    const safeBandW = DW * 0.06  // 峰值的6%区域
    // Ts/2 附近 (顶部)
    const peakX = L + DW / 2
    ctx.fillStyle = 'rgba(16,185,129,0.15)'
    ctx.fillRect(peakX - safeBandW / 2, stratTop, safeBandW, stratH)
    // Ts 附近 (底部 — 也是V0区)
    const botX = L + DW
    ctx.fillRect(botX - safeBandW / 2, stratTop, safeBandW, stratH)
    // 0 附近 (底部 — V0区开始)
    const bot0X = L
    ctx.fillRect(bot0X - safeBandW / 2, stratTop, safeBandW, stratH)
  }

  // 方案1标注
  const peakX = L + DW / 2
  ctx.fillStyle = '#10b981'; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('方案1: 顶点采样 (推荐)', peakX, stratTop - 2)

  // 采样触发箭头 (Ts/2)
  ctx.strokeStyle = '#10b981'; ctx.lineWidth = 2
  ctx.beginPath(); ctx.moveTo(peakX, stratTop - 10); ctx.lineTo(peakX, stratTop + 6); ctx.stroke()
  ctx.fillStyle = '#10b981'
  ctx.beginPath(); ctx.moveTo(peakX, stratTop - 10); ctx.lineTo(peakX - 6, stratTop - 2); ctx.lineTo(peakX + 6, stratTop - 2); ctx.closePath(); ctx.fill()

  // 采样点 (三角波顶点)
  ctx.fillStyle = '#10b981'; ctx.beginPath(); ctx.arc(peakX, stratTop + stratH / 2, 6, 0, 2 * Math.PI); ctx.fill()
  ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5
  ctx.beginPath(); ctx.arc(peakX, stratTop + stratH / 2, 6, 0, 2 * Math.PI); ctx.stroke()
  ctx.fillStyle = '#10b981'; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('ADC', peakX, stratTop + stratH / 2 + 3)

  // 零矢量标注
  ctx.fillStyle = textSec; ctx.font = '9px monospace'; ctx.textAlign = 'center'
  ctx.fillText('V0/V7 零矢量中心', peakX, stratTop + stratH + 14)
  ctx.fillText('开关噪声最小', peakX, stratTop + stratH + 26)

  // 方案2: 底点采样 (次优)
  const botStratX = L + DW * 0.85
  ctx.fillStyle = '#f59e0b'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('方案2: 底点采样', botStratX, stratTop - 2)
  ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 1.5
  ctx.beginPath(); ctx.moveTo(botStratX, stratTop - 8); ctx.lineTo(botStratX, stratTop + 6); ctx.stroke()
  ctx.fillStyle = '#f59e0b'; ctx.beginPath(); ctx.arc(botStratX, stratTop + stratH / 2 + 10, 4.5, 0, 2 * Math.PI); ctx.fill()
  ctx.font = 'bold 10px sans-serif'
  ctx.fillText('ADC', botStratX, stratTop + stratH / 2 + 26)
  ctx.font = '9px monospace'; ctx.fillStyle = textSec
  ctx.fillText('切换时刻附近', botStratX, stratTop + stratH + 14)
  ctx.fillText('噪声较大', botStratX, stratTop + stratH + 26)

  // 红色噪声区标注
  const noiseLeft = L + DW * 0.15, noiseRight = L + DW * 0.35
  ctx.fillStyle = 'rgba(239,68,68,0.1)'; ctx.fillRect(noiseLeft, stratTop, noiseRight - noiseLeft, stratH)
  ctx.fillStyle = '#ef4444'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('开关噪声区', (noiseLeft + noiseRight) / 2, stratTop - 4)
  ctx.fillText('✗', (noiseLeft + noiseRight) / 2, stratTop + stratH / 2 + 3)

  // ═══════════════════════════════════════════════════════
  //  下段: 相电流波形 + 开关噪声 (390~480)
  // ═══════════════════════════════════════════════════════
  const botTop = SEP2_Y + 6
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('相电流波形 — 开关噪声 vs 安全采样点', W / 2, botTop + 12)

  const currTop = botTop + 22, currH = 62, currMid = currTop + currH / 2
  const currScale = currH * 0.42

  // 基波正弦
  const N_CURR = 400
  // 电流 = 基波正弦 + 开关纹波 (锯齿)
  ctx.strokeStyle = '#3b82f6'; ctx.lineWidth = 1.8
  ctx.beginPath()
  for (let i = 0; i <= N_CURR; i++) {
    const frac = i / N_CURR
    const deg = frac * 360 * 2  // 两个周期
    const iBase = Math.sin(toRad(deg))
    // 叠加开关纹波 (三角波)
    const rippleFrac = (i / (N_CURR / 40)) % 1  // 每个载波周期内的位置
    const ripple = (rippleFrac < 0.5 ? rippleFrac * 2 - 1 : 1 - (rippleFrac - 0.5) * 2) * 0.08
    const currVal = iBase + ripple
    const x = L + frac * DW
    const y = currMid - currVal * currScale
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
  }
  ctx.stroke()

  if (showNoise.value) {
    // 开关噪声尖峰 (在三角波比较点)
    ctx.strokeStyle = 'rgba(239,68,68,0.8)'; ctx.lineWidth = 1.2
    for (let k = 0; k < 80; k++) {
      const frac = k / 40  // 两个周期内40个开关周期
      // 每次开关有两个切换点 (上升和下降)
      const riseFrac = frac + tcmp.A / triPeak * 0.5 / 2
      const fallFrac = frac + (1 - tcmp.A / triPeak * 0.5) / 2
      for (const swFrac of [riseFrac, fallFrac]) {
        if (swFrac < 0 || swFrac > 2) continue
        const x = L + (swFrac / 2) * DW
        const iBase = Math.sin(toRad(swFrac * 180))
        const spikeH = 0.12 * currScale
        ctx.beginPath(); ctx.moveTo(x, currMid - iBase * currScale)
        ctx.lineTo(x, currMid - iBase * currScale - spikeH)
        ctx.stroke()
      }
    }
  }

  // 安全采样点 (绿色, 在三角波顶点)
  ctx.fillStyle = '#10b981'
  for (let k = 0; k <= 80; k++) {
    const frac = k / 40
    const iBase = Math.sin(toRad(frac * 180))
    const x = L + frac * DW
    ctx.beginPath(); ctx.arc(x, currMid - iBase * currScale, 3, 0, 2 * Math.PI); ctx.fill()
  }

  // 图例
  ctx.fillStyle = textSec; ctx.font = '10px sans-serif'; ctx.textAlign = 'left'
  const legY = currTop + currH + 10
  ctx.fillStyle = '#3b82f6'; ctx.fillRect(L + 10, legY - 5, 14, 3)
  ctx.fillStyle = textSec; ctx.fillText('相电流 (含纹波)', L + 28, legY)
  ctx.fillStyle = '#ef4444'; ctx.fillRect(L + 150, legY - 5, 14, 3)
  ctx.fillStyle = textSec; ctx.fillText('开关噪声尖峰', L + 168, legY)
  ctx.fillStyle = '#10b981'; ctx.beginPath(); ctx.arc(L + 290, legY - 3, 4, 0, 2 * Math.PI); ctx.fill()
  ctx.fillStyle = textSec; ctx.fillText('安全采样点 (顶点)', L + 300, legY)
}

const animate = () => {
  try { draw() } catch { /* keep loop alive */ }
  animFrameId = requestAnimationFrame(animate)
}

onMounted(() => { animFrameId = requestAnimationFrame(animate) })
onUnmounted(() => cancelAnimationFrame(animFrameId))
</script>

<template>
  <div class="sim-container">
    <div class="sim-header">
      <h4>ADC 电流采样时序 — 避开开关噪声的最佳采样时机</h4>
    </div>

    <canvas ref="canvasRef" class="sim-canvas"></canvas>

    <div class="sim-params">
      <div class="param-group">
        <label>PWM 频率: <span class="param-val">{{ pwmFreq }} kHz</span></label>
        <input type="range" v-model.number="pwmFreq" :min="1" :max="20" :step="1" />
        <span class="param-range">1 ~ 20 kHz (Ts = {{ (1000 / pwmFreq).toFixed(0) }} μs)</span>
      </div>
      <div class="param-group">
        <label>死区时间: <span class="param-val">{{ deadTimeNs }} ns</span></label>
        <input type="range" v-model.number="deadTimeNs" :min="0" :max="3000" :step="50" />
        <span class="param-range">0 ~ 3000 ns</span>
      </div>
      <div class="param-group">
        <label>显示噪声:
          <button class="toggle-btn" :class="{ active: showNoise }" @click="showNoise = !showNoise">
            {{ showNoise ? 'ON' : 'OFF' }}
          </button>
        </label>
      </div>
      <div class="param-group">
        <label>安全窗口:
          <button class="toggle-btn" :class="{ active: highlightSafe }" @click="highlightSafe = !highlightSafe">
            {{ highlightSafe ? 'ON' : 'OFF' }}
          </button>
        </label>
      </div>
    </div>

    <div class="sim-info">
      <div class="info-block">
        <div class="info-block-title">采样策略对比</div>
        <div class="info-row"><span class="info-label" style="color:#10b981">顶点采样 (推荐):</span><span class="info-value">Ts/2 处采样 → V0/V7零矢量中心 → 无开关动作 → 噪声最小</span></div>
        <div class="info-row"><span class="info-label" style="color:#f59e0b">底点采样 (次优):</span><span class="info-value">Ts 处采样 → 靠近切换时刻 → 有一定噪声</span></div>
        <div class="info-row"><span class="info-label" style="color:#ef4444">随机/连续: </span><span class="info-value">任意时刻 → 可能在开关瞬态 → 采样无效</span></div>
      </div>
      <div class="info-block">
        <div class="info-block-title">关键参数</div>
        <div class="info-row"><span class="info-label">PWM: </span><span class="info-value highlight">{{ pwmFreq }} kHz</span><span class="info-note">Ts={{ (1000/pwmFreq).toFixed(0) }}μs</span></div>
        <div class="info-row"><span class="info-label">死区: </span><span class="info-value">{{ deadTimeNs }}ns</span></div>
        <div class="info-row"><span class="info-label">TcmpA/B/C: </span><span class="info-value">{{ (tcmpVals.A*100).toFixed(0) }}% / {{ (tcmpVals.B*100).toFixed(0) }}% / {{ (tcmpVals.C*100).toFixed(0) }}%</span></div>
      </div>
    </div>

    <div class="explain-panel">
      <details open>
        <summary>电流采样时序原理</summary>
        <div class="explain-content">
          <p><strong>为何采样时机至关重要：</strong>PWM 驱动的逆变器中，相电流含有高频开关纹波。在开关管导通/关断瞬间，电流存在剧烈振荡和 di/dt 噪声。若 ADC 在此时采样，会得到完全错误的电流读数，导致 FOC 控制崩溃。</p>
          <p><strong>最优方案 — 三角波顶点采样：</strong>七段式 SVPWM 中，三角波顶点 (Ts/2) 恰好对应零矢量 V0(000) 或 V7(111) 的中间时刻。此时所有三个桥臂要么全导通要么全关断，没有开关动作，相电流处于最干净的状态。这是 ADC 采样的黄金窗口。</p>
          <p><strong>次优方案 — 三角波底点采样：</strong>三角波底点 (0 或 Ts) 也对应零矢量时刻，但靠近切换沿，可能受前一周期尾迹影响。</p>
          <p><strong>实际工程做法：</strong>配置 MCU 定时器在三角波顶点 (计数器=周期值) 触发 ADC 注入组采样，双 ADC 同步采样两相电流，第三相由 ia+ib+ic=0 推算。采样完成后触发 DMA 传输至 FOC 算法缓冲区。</p>
        </div>
      </details>
    </div>
  </div>
</template>

<style scoped>
.sim-container { border: 1px solid var(--border-color); border-radius: 8px; padding: 0.8rem; margin: 1.5rem 0; background: var(--bg-color); }
.sim-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 0.6rem; flex-wrap: wrap; gap: 0.5rem; }
.sim-header h4 { margin: 0; font-size: 0.92rem; color: var(--primary-color); }
.sim-canvas { display: block; margin: 0 auto; max-width: 100%; border-radius: 6px; border: 1px solid var(--border-color); }
.sim-params { display: flex; flex-wrap: wrap; gap: 1.5rem; margin-top: 0.6rem; justify-content: center; }
.param-group { display: flex; flex-direction: column; align-items: center; gap: 0.15rem; font-size: 0.78rem; min-width: 185px; }
.param-group label { font-weight: 600; color: var(--text-color); }
.param-val { color: var(--primary-color); font-weight: 700; }
.param-group input[type="range"] { width: 100%; cursor: pointer; }
.param-range { font-size: 0.68rem; color: #0ea5e9; }
.toggle-btn { padding: 0.15rem 0.55rem; border: 1px solid var(--border-color); border-radius: 4px; background: var(--content-bg); color: var(--text-color); cursor: pointer; font-size: 0.72rem; font-weight: 500; margin-left: 0.4rem; }
.toggle-btn.active { background: var(--primary-color); color: #fff; border-color: var(--primary-color); }
.sim-info { display: flex; flex-wrap: wrap; gap: 0.8rem; margin-top: 0.6rem; padding: 0.65rem; background: var(--content-bg); border: 1px solid var(--border-color); border-radius: 6px; font-size: 0.75rem; justify-content: center; }
.info-block { display: flex; flex-direction: column; gap: 0.15rem; min-width: 200px; max-width: 400px; }
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