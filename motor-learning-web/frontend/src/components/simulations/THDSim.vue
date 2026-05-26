<script setup lang="ts">
import { ref, computed, watch, onMounted, onUnmounted } from 'vue'

/**
 * THD 谐波分析仿真
 * 对比 SPWM / SVPWM / 六步方波调制的谐波频谱
 *
 * Canvas 920×480，左右两栏:
 *   左栏 (0~445): 谐波柱状图 — 频率 vs 幅值% (基波至25次)
 *   右栏 (450~910): THD 汇总指标 — THD%, WTHD, 基波利用率, 对比表
 */

const canvasRef = ref<HTMLCanvasElement | null>(null)
let animFrameId = 0

// --- 调制参数 ---
const modulationIndex = ref(0.8)   // 调制比 m (0.3 ~ 1.155)

// --- 谐波数据生成 ---
interface HarmonicData {
  order: number
  spwm: number      // SPWM 谐波幅值 %
  svpwm: number     // SVPWM 谐波幅值 %
  sixstep: number   // 六步方波谐波幅值 %
}

// 典型谐波数据 (基于文献, 归一化到基波)
function generateHarmonics(m: number): HarmonicData[] {
  const harmonics: HarmonicData[] = []

  // 基波
  harmonics.push({ order: 1, spwm: 100, svpwm: 100, sixstep: 100 })

  // SPWM 特征: 主要谐波在 fc±2f0, fc±4f0 ... (载波附近)
  // 这里简化为低频段表现
  // 3,5,7,9,11,13,15,17,19,21,23,25

  const spwmOdd = [0, 0.5, 0.8, 1.2, 1.8, 2.5, 3.2, 4.0, 4.8, 5.5, 6.0, 6.5]
  // SVPWM: 低频谐波更小, 高频谐波稍大但易滤除
  const svpwmOdd = [0, 0.15, 0.2, 0.3, 0.4, 0.6, 2.8, 4.5, 5.2, 5.8, 6.2, 6.4]
  // 六步方波: 包含大量 5,7,11,13,... 6k±1 次谐波
  const sixOdd = [0, 1.5, 20, 5.0, 14.3, 5.0, 9.1, 3.8, 7.7, 3.0, 5.9, 2.5]

  for (let k = 0; k < 12; k++) {
    const order = 3 + k * 2  // 3,5,7,9,...
    const scale = m / 0.8  // 调制比影响
    harmonics.push({
      order,
      spwm: Math.max(0, spwmOdd[k] * scale * (1 + 0.3 * (1 - m))),
      svpwm: Math.max(0, svpwmOdd[k] * scale * (1 + 0.2 * (1 - m))),
      sixstep: sixOdd[k],
    })
  }

  return harmonics
}

const harmonicData = computed(() => generateHarmonics(modulationIndex.value))

// THD 计算
const thdMetrics = computed(() => {
  const h = harmonicData.value
  let spwmSum = 0, svpwmSum = 0, sixSum = 0
  let spwmWSum = 0, svpwmWSum = 0, sixWSum = 0

  for (let i = 1; i < h.length; i++) {
    spwmSum += h[i].spwm * h[i].spwm
    svpwmSum += h[i].svpwm * h[i].svpwm
    sixSum += h[i].sixstep * h[i].sixstep

    // WTHD: 加权 THD, 第 n 次谐波除以 n
    const n = h[i].order
    spwmWSum += (h[i].spwm / n) * (h[i].spwm / n)
    svpwmWSum += (h[i].svpwm / n) * (h[i].svpwm / n)
    sixWSum += (h[i].sixstep / n) * (h[i].sixstep / n)
  }

  const thdSpwm = Math.sqrt(spwmSum) / 100 * 100
  const thdSvpwm = Math.sqrt(svpwmSum) / 100 * 100
  const thdSix = Math.sqrt(sixSum) / 100 * 100

  const wthdSpwm = Math.sqrt(spwmWSum)
  const wthdSvpwm = Math.sqrt(svpwmWSum)
  const wthdSix = Math.sqrt(sixWSum)

  // 基波利用率: SPWM=1, SVPWM≈1.155 (15.5% 提升)
  const fundUtilSpwm = 1.0
  const fundUtilSvpwm = 1.155
  const fundUtilSix = 1.273  // 六步方波: 4/π * Vdc/2

  // 主导谐波
  function dominantHarm(hd: HarmonicData[], key: 'spwm' | 'svpwm' | 'sixstep'): string {
    let maxVal = 0, maxOrder = 0
    for (let i = 1; i < hd.length; i++) {
      if (hd[i][key] > maxVal) { maxVal = hd[i][key]; maxOrder = hd[i].order }
    }
    return maxOrder > 0 ? `${maxOrder}次(${maxVal.toFixed(1)}%)` : '--'
  }

  return {
    thdSpwm: thdSpwm.toFixed(1),
    thdSvpwm: thdSvpwm.toFixed(1),
    thdSix: thdSix.toFixed(1),
    wthdSpwm: wthdSpwm.toFixed(1),
    wthdSvpwm: wthdSvpwm.toFixed(1),
    wthdSix: wthdSix.toFixed(1),
    fundUtilSpwm: (fundUtilSpwm * 100).toFixed(0),
    fundUtilSvpwm: (fundUtilSvpwm * 100).toFixed(0),
    fundUtilSix: (fundUtilSix * 100).toFixed(0),
    domSpwm: dominantHarm(harmonicData.value, 'spwm'),
    domSvpwm: dominantHarm(harmonicData.value, 'svpwm'),
    domSix: dominantHarm(harmonicData.value, 'sixstep'),
  }
})

// ═════════════════════════ 绘图 ═════════════════════════
const draw = () => {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  const W = 920, H = 480
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

  // ═════════════════════ 左栏: 谐波柱状图 ═════════════════════
  const L_L = 30, L_T = 28, L_W = 405, L_H = 430
  const L_B = L_T + L_H

  // 标题
  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('谐波频谱对比 (基波至 25 次)', L_L + L_W / 2, L_T - 6)

  const hd = harmonicData.value
  const barGroupW = (L_W - 60) / hd.length
  const barW = Math.max(1, barGroupW * 0.22)
  const colors = ['#3b82f6', '#10b981', '#ef4444']
  const names = ['SPWM', 'SVPWM', '六步']

  // 基线
  ctx.strokeStyle = borderColor; ctx.lineWidth = 0.6
  ctx.beginPath(); ctx.moveTo(L_L + 24, L_B - 28); ctx.lineTo(L_L + L_W - 10, L_B - 28); ctx.stroke()

  // 绘制三个并排柱状
  hd.forEach((h, i) => {
    const gx = L_L + 38 + i * barGroupW
    const bars = [
      { val: h.spwm, color: colors[0] },
      { val: h.svpwm, color: colors[1] },
      { val: h.sixstep, color: colors[2] },
    ]

    bars.forEach((bar, bi) => {
      const bx = gx + bi * (barW + 1.5)
      const bh = (bar.val / 25) * (L_H - 50)
      const by = L_B - 28 - Math.max(1, bh)

      ctx.fillStyle = bar.color + 'CC'
      ctx.fillRect(bx, by, barW, Math.max(1, bh))
      ctx.strokeStyle = bar.color; ctx.lineWidth = 0.4
      ctx.strokeRect(bx, by, barW, Math.max(1, bh))
    })

    // X 轴标签
    ctx.fillStyle = textSec; ctx.font = 'bold 7px sans-serif'; ctx.textAlign = 'center'
    if (h.order === 1) {
      ctx.fillText('基波', gx + barW * 1.5, L_B - 10)
    } else if (h.order % 5 === 0 || h.order <= 7) {
      ctx.fillText(`${h.order}`, gx + barW * 1.5, L_B - 10)
    }
  })

  // 图例
  const legendY = L_T
  names.forEach((name, i) => {
    const lx = L_L + L_W - 160 + i * 52
    ctx.fillStyle = colors[i]; ctx.fillRect(lx, legendY, 12, 9)
    ctx.fillStyle = textColor; ctx.font = 'bold 8px sans-serif'; ctx.textAlign = 'left'
    ctx.fillText(name, lx + 16, legendY + 9)
  })

  ctx.fillStyle = textSec; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('SVPWM: 低频谐波小 → 易于电机电感滤波', L_L + L_W / 2, L_B + 8)

  // ═════════════════════ 右栏: THD 汇总 ═════════════════════
  const R_L = 462, R_T = 28, R_W = 440

  ctx.fillStyle = textColor; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('THD 汇总指标', R_L + R_W / 2, R_T)

  const tm = thdMetrics.value

  // 三大 THD 数字
  const bigNumsY = R_T + 30
  const bigNums = [
    { label: 'SPWM', thd: tm.thdSpwm, wthd: tm.wthdSpwm, color: colors[0] },
    { label: 'SVPWM', thd: tm.thdSvpwm, wthd: tm.wthdSvpwm, color: colors[1] },
    { label: '六步方波', thd: tm.thdSix, wthd: tm.wthdSix, color: colors[2] },
  ]

  const numW = 130
  bigNums.forEach((bn, i) => {
    const nx = R_L + 15 + i * (numW + 14)
    ctx.fillStyle = contentBg; ctx.fillRect(nx, bigNumsY, numW, 82)
    ctx.strokeStyle = borderColor; ctx.lineWidth = 0.8; ctx.strokeRect(nx, bigNumsY, numW, 82)

    ctx.fillStyle = bn.color; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'
    ctx.fillText(bn.label, nx + numW / 2, bigNumsY + 18)
    ctx.font = 'bold 26px monospace'
    ctx.fillText(`${bn.thd}%`, nx + numW / 2, bigNumsY + 52)
    ctx.font = 'bold 8px sans-serif'; ctx.fillStyle = textSec
    ctx.fillText(`THD`, nx + numW / 2, bigNumsY + 68)
    ctx.fillText(`WTHD: ${bn.wthd}`, nx + numW / 2, bigNumsY + 78)
  })

  // 基波利用率对比
  const fundY = bigNumsY + 96
  ctx.fillStyle = textColor; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('基波电压利用率对比', R_L + R_W / 2, fundY)

  const fundData = [
    { label: 'SPWM', pct: 100, color: colors[0] },
    { label: 'SVPWM', pct: 115.5, color: colors[1] },
    { label: '六步', pct: 127.3, color: colors[2] },
  ]

  const fundBarMaxW = 240, fundBarH = 22
  fundData.forEach((fd, i) => {
    const fy = fundY + 18 + i * (fundBarH + 8)
    ctx.fillStyle = textSec; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'right'
    ctx.fillText(fd.label, R_L + 55, fy + fundBarH / 2 + 3)
    const bw2 = (fd.pct / 130) * fundBarMaxW
    ctx.fillStyle = fd.color + '88'
    ctx.fillRect(R_L + 62, fy, bw2, fundBarH)
    ctx.strokeStyle = fd.color; ctx.lineWidth = 1.2; ctx.strokeRect(R_L + 62, fy, bw2, fundBarH)
    ctx.fillStyle = textColor; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'left'
    ctx.fillText(`${fd.pct.toFixed(1)}%`, R_L + 62 + bw2 + 6, fy + fundBarH / 2 + 3)
    if (fd.label === 'SVPWM') {
      ctx.fillText('(+15.5%)', R_L + 62 + bw2 + 50, fy + fundBarH / 2 + 3)
    }
  })

  // 对比表格
  const tblY = fundY + 18 + 3 * (fundBarH + 8) + 16
  ctx.fillStyle = textColor; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('各次谐波幅值对比 (%)', R_L + R_W / 2, tblY)

  const tblTop = tblY + 10, rowH = 17
  const colWs = [44, 70, 70, 70]

  // 表头
  ;['阶次', 'SPWM', 'SVPWM', '六步'].forEach((hdr, hi) => {
    const colX = R_L + 10 + (hi > 0 ? colWs[0] : 0) + (hi <= 1 ? 0 : colWs[1]) + (hi <= 2 ? 0 : colWs[2])
    const cw = colWs[Math.min(hi, 3)]
    ctx.fillStyle = '#f1f5f9'; ctx.fillRect(colX, tblTop, cw, rowH)
    ctx.strokeStyle = borderColor; ctx.lineWidth = 0.5; ctx.strokeRect(colX, tblTop, cw, rowH)
    ctx.fillStyle = textColor; ctx.font = 'bold 8px sans-serif'; ctx.textAlign = 'center'
    ctx.fillText(hdr, colX + cw / 2, tblTop + rowH / 2 + 3)
  })

  // 表内容 (仅显示关键行)
  const showOrders = [1, 5, 7, 11, 13, 17, 19, 23, 25]
  showOrders.forEach((order, ri) => {
    const ry = tblTop + rowH + ri * rowH
    const hRow = hd.find(h => h.order === order)
    if (!hRow) return

    const rowData = [order, hRow.spwm.toFixed(1), hRow.svpwm.toFixed(1), hRow.sixstep.toFixed(1)]

    rowData.forEach((val, ci) => {
      const colX = R_L + 10 + (ci > 0 ? colWs[0] : 0) + (ci <= 1 ? 0 : colWs[1]) + (ci <= 2 ? 0 : colWs[2])
      const cw = colWs[Math.min(ci, 3)]
      ctx.fillStyle = ri % 2 === 0 ? '#fafbfc' : '#fff'
      ctx.fillRect(colX, ry, cw, rowH)
      ctx.strokeStyle = borderColor; ctx.lineWidth = 0.3; ctx.strokeRect(colX, ry, cw, rowH)
      ctx.fillStyle = (ci > 1 && Number(val) > 10) ? '#ef4444' : textColor
      ctx.font = `${ci === 0 ? 'bold' : 'normal'} 8px monospace`; ctx.textAlign = 'center'
      ctx.fillText(String(val), colX + cw / 2, ry + rowH / 2 + 3)
    })
  })

  // 底部要点
  const noteY = tblTop + rowH + showOrders.length * rowH + 10
  ctx.fillStyle = primaryColor; ctx.font = 'bold 9px sans-serif'; ctx.textAlign = 'center'
  ctx.fillText('SVPWM 将低频(5,7,11,13)谐波向高频转移 → 电机电感自然衰减 → 电流 THD 最低', R_L + R_W / 2, noteY)
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
      <h4>THD 谐波分析仿真 -- SPWM vs SVPWM vs 六步方波</h4>
    </div>

    <canvas ref="canvasRef" class="sim-canvas"></canvas>

    <div class="sim-params">
      <div class="param-group">
        <label>调制比 m: <span class="param-val">{{ modulationIndex.toFixed(3) }}</span></label>
        <input type="range" v-model.number="modulationIndex" :min="0.3" :max="1.155" :step="0.005" />
        <span class="param-range">0.3 ~ 1.155 (SVPWM 线性上限)</span>
      </div>
    </div>

    <div class="sim-info">
      <div class="info-block">
        <div class="info-block-title">SPWM</div>
        <div class="info-row"><span class="info-label">THD:</span><span class="info-value highlight">{{ thdMetrics.thdSpwm }}%</span></div>
        <div class="info-row"><span class="info-label">WTHD:</span><span class="info-value">{{ thdMetrics.wthdSpwm }}</span></div>
        <div class="info-row"><span class="info-label">主导谐波:</span><span class="info-value">{{ thdMetrics.domSpwm }}</span></div>
        <div class="info-row"><span class="info-label">基波利用率:</span><span class="info-value">{{ thdMetrics.fundUtilSpwm }}%</span></div>
      </div>
      <div class="info-block">
        <div class="info-block-title">SVPWM</div>
        <div class="info-row"><span class="info-label">THD:</span><span class="info-value highlight" style="color:#10b981">{{ thdMetrics.thdSvpwm }}%</span></div>
        <div class="info-row"><span class="info-label">WTHD:</span><span class="info-value">{{ thdMetrics.wthdSvpwm }}</span></div>
        <div class="info-row"><span class="info-label">主导谐波:</span><span class="info-value">{{ thdMetrics.domSvpwm }}</span></div>
        <div class="info-row"><span class="info-label">基波利用率:</span><span class="info-value" style="color:#10b981">{{ thdMetrics.fundUtilSvpwm }}% (+15.5%)</span></div>
      </div>
      <div class="info-block">
        <div class="info-block-title">六步方波</div>
        <div class="info-row"><span class="info-label">THD:</span><span class="info-value highlight" style="color:#ef4444">{{ thdMetrics.thdSix }}%</span></div>
        <div class="info-row"><span class="info-label">WTHD:</span><span class="info-value">{{ thdMetrics.wthdSix }}</span></div>
        <div class="info-row"><span class="info-label">主导谐波:</span><span class="info-value">{{ thdMetrics.domSix }}</span></div>
        <div class="info-row"><span class="info-label">基波利用率:</span><span class="info-value">{{ thdMetrics.fundUtilSix }}% (+27.3%)</span></div>
      </div>
    </div>

    <div class="explain-panel">
      <details open>
        <summary>THD / WTHD 与调制策略谐波分析</summary>
        <div class="explain-content">
          <p><strong>THD (总谐波畸变率)：</strong>衡量输出电压波形偏离正弦的程度。THD = √(ΣVn²) / V1 × 100%。电流 THD 决定电机附加铜损、铁损和转矩脉动。</p>
          <p><strong>WTHD (加权 THD)：</strong>考虑电机电感的低通滤波效应，高频谐波产生的电流畸变更小。WTHD = √(Σ(Vn/n)²)，更准确反映电流质量。</p>
          <p><strong>SVPWM 优势：</strong>相比 SPWM，SVPWM 将低频 (5,7,11,13次) 谐波能量转移到更高频段，经电机绕组电感衰减后电流 THD 更低。同时基波利用率提升 15.5%，相当于同等母线电压下多输出 15.5% 转矩。</p>
          <p><strong>六步方波：</strong>只切换 6 次/电周期，基波利用率最高 (1.273倍)，但含有大量低次谐波 (5次=20%, 7次=14.3%)，适用于对谐波不敏感的高速大功率场合。</p>
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
.sim-info { display: flex; flex-wrap: wrap; gap: 0.8rem; margin-top: 0.6rem; padding: 0.65rem; background: var(--content-bg); border: 1px solid var(--border-color); border-radius: 6px; font-size: 0.75rem; justify-content: center; }
.info-block { display: flex; flex-direction: column; gap: 0.15rem; min-width: 180px; max-width: 300px; }
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