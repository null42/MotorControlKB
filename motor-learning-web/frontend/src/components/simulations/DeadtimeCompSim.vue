<script setup lang="ts">
import { ref, computed, watch, onMounted, onUnmounted } from 'vue'

/**
 * 死区补偿仿真
 * 展示死区电压误差、电流零位钳位效应及三种补偿策略
 *
 * Canvas 920×520，三行布局:
 *   上栏 (0~175): 死区电压误差 — 理想 PWM vs 实际 PWM 对比
 *   中栏 (180~355): 电压误差 vs 电流 — 死区非线性
 *   下栏 (360~520): 三种补偿方法对比 — 补偿 vs 未补偿相电流
 */

const canvasRef = ref<HTMLCanvasElement | null>(null)
let animFrameId = 0

// --- 参数 ---
const deadTime = ref(2000)         // 死区时间 ns (0 ~ 3000)
const pwmFreq = ref(10)            // PWM 频率 kHz (1 ~ 20)
const compensationMethod = ref<'none' | 'avg' | 'pwm' | 'obs'>('avg')
const phaseCurrent = ref(3.0)      // 当前相电流 A

// --- 计算 ---
const tsUs = computed(() => (1000 / pwmFreq.value).toFixed(1))  // PWM 周期 μs
const deadPct = computed(() => {
  return ((deadTime.value * 1e-9) / (1 / (pwmFreq.value * 1000)) * 100).toFixed(2)
})  // 死区占空比 %

const voltageError = computed(() => {
  // ΔV = Vdc × Tdead / Ts (假设 Vdc=100V)
  return (100 * deadTime.value * 1e-9 * pwmFreq.value * 1000).toFixed(2)
})  // 电压误差 V

// 死区效应仿真数据
function generateDeadZoneData(): { current: number; vErr: number }[] {
  const pts: { current: number; vErr: number }[] = []
  const Imax = 8  // ±8A 范围
  const dV = parseFloat(voltageError.value)

  for (let i = -Imax; i <= Imax; i += 0.1) {
    // 符号函数过渡: 死区电压误差 ∝ sign(i)
    // 使用平滑过渡模拟实际过渡区
    const smoothSign = Math.tanh(i / 1.5)
    const vErr = dV * smoothSign
    pts.push({ current: i, vErr })
  }
  return pts
}

function generatePhaseCurrents(compensated: boolean): number[] {
  const N = 200
  const data: number[] = []
  const Im = 5  // 幅值

  for (let i = 0; i < N; i++) {
    const theta = (i / N) * 2 * Math.PI
    let val = Im * Math.sin(theta)

    if (!compensated) {
      // 未补偿: 在过零点附近产生钳位 (六拍阶梯)
      if (Math.abs(val) < 1.2) {
        val *= 0.3  // 零位钳位
      }
    }
    data.push(val)
  }
  return data
}

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

  // 水平分隔线
  const SEP1 = 175, SEP2 = 355
  ctx.strokeStyle = borderColor; ctx.lineWidth = 1; ctx.setLineDash([6, 4])
  ;[SEP1, SEP2].forEach(y => { ctx.beginPath(); ctx.moveTo(12, y); ctx.lineTo(W - 12, y); ctx.stroke() })
  ctx.setLineDash([])

  // ═════════════════════ 上栏: 死区电压误差 ═════════════════════
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('死区电压误差 (一相 PWM 波形)', W / 2, 18)

  const PL = 40, PT = 30, PW = W - 80, PH = 130
  const PCY = PT + PH / 2

  // 左半: 理想 PWM
  const lw = PW * 0.45

  // 理想 PWM 高电平
  ctx.strokeStyle = primaryColor; ctx.lineWidth = 2
  const duty = 0.4
  ctx.beginPath()
  ctx.moveTo(PL, PCY + 40); ctx.lineTo(PL + lw * duty, PCY + 40)
  ctx.lineTo(PL + lw * duty, PCY - 30); ctx.lineTo(PL + lw, PCY - 30)
  ctx.stroke()

  ctx.fillStyle = textSec; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('理想 PWM', PL + lw / 2, PCY + 40 + 16)
  ctx.fillText('无死区', PL + lw / 2, PCY + 40 + 30)

  // 实际 PWM (有死区)
  const gapPx = (deadTime.value * 1e-9 * pwmFreq.value * 1000) * lw * 400 / PW
  const rL = PL + lw + 40

  // 上管实际 (上升沿延迟)
  ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 2
  ctx.beginPath()
  ctx.moveTo(rL, PCY + 40); ctx.lineTo(rL + lw * duty + gapPx, PCY + 40)
  ctx.lineTo(rL + lw * duty + gapPx, PCY - 30); ctx.lineTo(rL + lw, PCY - 30)
  ctx.stroke()

  // 死区阴影
  ctx.fillStyle = 'rgba(239, 68, 68, 0.18)'
  ctx.fillRect(rL + lw * duty, PCY - 32, gapPx, 72)
  ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 0.8; ctx.setLineDash([3, 3])
  ctx.strokeRect(rL + lw * duty, PCY - 32, gapPx, 72)
  ctx.setLineDash([])

  // 标注
  ctx.fillStyle = '#ef4444'; ctx.font = 'bold 8px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText(`Tdead`, rL + lw * duty + gapPx / 2, PCY - 38)
  ctx.fillText(`${deadTime}ns`, rL + lw * duty + gapPx / 2, PCY + 52)

  ctx.fillStyle = textSec; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('实际 PWM (有死区)', rL + lw / 2, PCY + 40 + 30)

  // 电压误差公式
  const formX = rL + lw + 10
  ctx.fillStyle = contentBg; ctx.fillRect(formX, PT + 5, 170, 55)
  ctx.strokeStyle = borderColor; ctx.lineWidth = 0.7; ctx.strokeRect(formX, PT + 5, 170, 55)
  ctx.fillStyle = '#ef4444'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('电压误差', formX + 85, PT + 18)
  ctx.fillStyle = textColor; ctx.font = 'bold 11px monospace'; ctx.textAlign = 'center'
  ctx.fillText('\u0394V = Vdc × Tdead/Ts', formX + 85, PT + 38)
  ctx.font = '10px monospace'
  ctx.fillText(`= ${voltageError} V (${deadPct}%)`, formX + 85, PT + 54)

  // ═════════════════════ 中栏: 电压误差 vs 电流 ═════════════════════
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('电压误差 vs 相电流 (死区非线性)', W / 2, SEP1 + 18)

  const ML = 60, MT = SEP1 + 30, MW = W - 120, MH = SEP2 - MT - 10
  const MCX = ML + MW / 2, MCY = MT + MH / 2

  // 坐标轴
  ctx.strokeStyle = borderColor; ctx.lineWidth = 0.8
  ctx.beginPath(); ctx.moveTo(ML, MCY); ctx.lineTo(ML + MW, MCY); ctx.stroke()
  ctx.beginPath(); ctx.moveTo(MCX, MT + 4); ctx.lineTo(MCX, MT + MH - 4); ctx.stroke()
  ctx.fillStyle = textSec; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('i 相电流 (A)', ML + MW / 2, MT + MH - 2)
  ctx.fillText('\u0394V (V)', ML - 24, MCY + 4)

  // 死区特性曲线
  const dz = generateDeadZoneData()
  const dVmax = parseFloat(voltageError.value)
  const scaleX = (MW - 20) / 16
  const scaleY = (MH - 30) / (2 * Math.max(dVmax, 1))

  ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 2
  ctx.beginPath()
  dz.forEach((p, i) => {
    const x = MCX + p.current * scaleX
    const y = MCY - p.vErr * scaleY
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
  })
  ctx.stroke()

  // 标注死区
  ctx.fillStyle = '#ef4444'; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText(`\u0394V = +${dVmax.toFixed(1)}V (i>0 时)`, MCX + 20, MCY - dVmax * scaleY - 4)
  ctx.fillText(`\u0394V = -${dVmax.toFixed(1)}V (i<0 时)`, MCX + 20, MCY + dVmax * scaleY + 12)

  // 过零过渡区标注
  ctx.fillStyle = 'rgba(245, 158, 11, 0.1)'
  ctx.fillRect(MCX - 15, MT + 4, 30, MH - 8)
  ctx.fillStyle = '#f59e0b'; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('零位钳位区', MCX, MT + MH * 0.82)
  ctx.fillText('(电流过零失真)', MCX, MT + MH * 0.82 + 14)

  // ═════════════════════ 下栏: 补偿方法对比 ═════════════════════
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('补偿效果对比 — 相电流波形', W / 2, SEP2 + 18)

  const BL = 50, BT = SEP2 + 30, BW = W - 100, BH = H - BT - 14

  const uncompCurrent = generatePhaseCurrents(false)
  const compCurrent = generatePhaseCurrents(true)

  const waveToY = (val: number) => BT + BH / 2 - (val / 7) * (BH / 2 - 14)

  // 未补偿波形
  ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 2.5; ctx.setLineDash([4, 3])
  ctx.beginPath()
  uncompCurrent.forEach((v, i) => {
    const x = BL + (i / uncompCurrent.length) * BW
    const y = waveToY(v)
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
  })
  ctx.stroke()
  ctx.setLineDash([])

  // 补偿后波形
  ctx.strokeStyle = '#10b981'; ctx.lineWidth = 2.5
  ctx.beginPath()
  compCurrent.forEach((v, i) => {
    const x = BL + (i / compCurrent.length) * BW
    const y = waveToY(v)
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
  })
  ctx.stroke()

  // 图例
  ctx.fillStyle = '#ef4444'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'left'
  ctx.fillText('--- 未补偿 (零位钳位)', BL + 10, BT + 14)
  ctx.fillStyle = '#10b981'
  ctx.fillText('--- 已补偿 (平滑正弦)', BL + 10, BT + 28)

  // 方法标签
  const methodNames: Record<string, string> = {
    'none': '无补偿',
    'avg': '平均电压前馈补偿',
    'pwm': 'PWM 占空比直接修正',
    'obs': '观测器扰动估计补偿',
  }
  ctx.fillStyle = textSec; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'right'
  ctx.fillText(`当前方法: ${methodNames[compensationMethod.value]}`, BL + BW - 10, BT + 28)

  // 补偿原理说明小框
  const sbY = BT + BH - 40
  const sbW = BW
  ctx.fillStyle = contentBg; ctx.fillRect(BL, sbY, sbW, 34)
  ctx.strokeStyle = borderColor; ctx.lineWidth = 0.6; ctx.strokeRect(BL, sbY, sbW, 34)

  const methodDesc: Record<string, string> = {
    'none': '选择一种补偿方法查看效果。死区导致低速时电流畸变显著。',
    'avg': '平均电压法: 根据电流符号判断 ±ΔV 前馈, 在电压指令中直接补偿。实现简单, 零位仍有误差。',
    'pwm': 'PWM 修正法: 直接修改各相 PWM 占空比, 提前/延迟开通关断边沿。精度高, 需精确死区时间。',
    'obs': '观测器法: 构建死区扰动电压观测器, 实时估计 ΔV 并补偿。自适应性强, 计算量较大。',
  }

  ctx.fillStyle = primaryColor; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText(methodDesc[compensationMethod.value], BL + sbW / 2, sbY + 20)
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
      <h4>死区补偿仿真 -- 电压误差分析与补偿策略对比</h4>
      <div class="header-controls">
        <div class="mode-selector">
          <button :class="{ active: compensationMethod === 'none' }" @click="compensationMethod = 'none'">无补偿</button>
          <button :class="{ active: compensationMethod === 'avg' }" @click="compensationMethod = 'avg'">平均电压</button>
          <button :class="{ active: compensationMethod === 'pwm' }" @click="compensationMethod = 'pwm'">PWM修正</button>
          <button :class="{ active: compensationMethod === 'obs' }" @click="compensationMethod = 'obs'">观测器</button>
        </div>
      </div>
    </div>

    <canvas ref="canvasRef" class="sim-canvas"></canvas>

    <div class="sim-params">
      <div class="param-group">
        <label>死区时间 Tdead: <span class="param-val">{{ deadTime >= 1000 ? (deadTime/1000).toFixed(1)+'μs' : deadTime+'ns' }}</span></label>
        <input type="range" v-model.number="deadTime" :min="0" :max="3000" :step="50" />
        <span class="param-range">0 ~ 3000 ns</span>
      </div>
      <div class="param-group">
        <label>PWM 频率: <span class="param-val">{{ pwmFreq }} kHz</span></label>
        <input type="range" v-model.number="pwmFreq" :min="1" :max="20" :step="1" />
        <span class="param-range">1 ~ 20 kHz (Ts={{ tsUs }}μs)</span>
      </div>
    </div>

    <div class="sim-info">
      <div class="info-block">
        <div class="info-block-title">死区参数</div>
        <div class="info-row"><span class="info-label">Tdead:</span><span class="info-value highlight">{{ deadTime }} ns</span></div>
        <div class="info-row"><span class="info-label">PWM Ts:</span><span class="info-value">{{ tsUs }} μs</span></div>
        <div class="info-row"><span class="info-label">占空比损失:</span><span class="info-value highlight">{{ deadPct }}%</span></div>
      </div>
      <div class="info-block">
        <div class="info-block-title">电压误差</div>
        <div class="info-row"><span class="info-label">ΔV:</span><span class="info-value" style="color:#ef4444;font-weight:700">{{ voltageError }} V</span></div>
        <div class="info-row"><span class="info-label">每周期:</span><span class="info-value">Vdc × Tdead/Ts</span></div>
      </div>
      <div class="info-block">
        <div class="info-block-title">补偿方法</div>
        <div class="info-row"><span class="info-label">当前:</span><span class="info-value highlight">{{ { none:'无补偿', avg:'平均电压前馈', pwm:'PWM占空比修正', obs:'观测器估计' }[compensationMethod] }}</span></div>
      </div>
    </div>

    <div class="explain-panel">
      <details open>
        <summary>死区效应与补偿策略原理</summary>
        <div class="explain-content">
          <p><strong>死区来源：</strong>逆变器上下桥臂 IGBT/MOSFET 不能同时导通 (直通短路)。上管关断后需延时 Tdead (通常 1~3μs) 才开通下管，导致实际输出电压与指令存在偏差。</p>
          <p><strong>死区效应：</strong>ΔV = Vdc × Tdead/Ts，电流极性决定误差方向。当相电流过零时误差符号翻转，形成非线性"死区"，导致：</p>
          <p>(1) <strong>电压损失：</strong>输出电压幅值小于指令值，影响转矩输出。</p>
          <p>(2) <strong>电流零位钳位 (六拍阶梯)：</strong>过零点附近电流被钳制，波形畸变产生 5、7、11、13 次谐波。</p>
          <p>(3) <strong>转矩脉动：</strong>低速时尤为严重，占空比损失显著 (低速时调制比小, ΔV/Vref 比例大)。</p>
          <p><strong>三种补偿方法：</strong>平均电压前馈 (简单有效), PWM 直接修正 (精度高), 观测器自适应估计 (鲁棒性强但复杂)。实际工程中常用平均电压法 + 过零滞环避免频繁切换。</p>
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