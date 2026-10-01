<script setup lang="ts">
import { computed, ref, watch } from 'vue'
import type { AnyRecord } from './api'
const props = defineProps<{rows: AnyRecord[]}>()
const selected = ref<string[]>([])
const palette = ['#2664df', '#e27926', '#0e9d8b', '#955bd3', '#c1486c']
const numericKeys = computed(() => [...new Set(props.rows.slice(0, 100).flatMap(row => Object.keys(row)))].filter(key => props.rows.some(row => row[key] !== '' && row[key] !== null && Number.isFinite(Number(row[key])))))
watch(numericKeys, keys => { selected.value = keys.filter(key => /yaw|pitch|angle/i.test(key)).slice(0, 3) }, {immediate: true})
const xKey = computed(() => numericKeys.value.find(key => /^(timestamp|time|t)(_s|_ms|_ns)?$|source_time/i.test(key)))
const points = computed(() => props.rows.map((row, index) => ({x: xKey.value && Number.isFinite(Number(row[xKey.value])) ? Number(row[xKey.value]) : index, row})))
const domain = computed(() => {
  const xs = points.value.map(p => p.x)
  const ys = points.value.flatMap(p => selected.value.map(key => p.row[key] === '' || p.row[key] == null ? NaN : Number(p.row[key]))).filter(Number.isFinite)
  return {xmin: Math.min(...xs), xmax: Math.max(...xs), ymin: Math.min(...ys), ymax: Math.max(...ys)}
})
function line(key: string) {
  const d = domain.value; const sx = 740 / (d.xmax - d.xmin || 1); const sy = 230 / (d.ymax - d.ymin || 1)
  const stride = Math.max(1, Math.ceil(points.value.length / 1500))
  return points.value.filter((_, i) => i % stride === 0).map(p => ({x: 65 + (p.x - d.xmin) * sx, y: 260 - (Number(p.row[key]) - d.ymin) * sy, valid: p.row[key] !== '' && p.row[key] != null && Number.isFinite(Number(p.row[key]))})).reduce((path, p, i, rows) => `${path} ${p.valid ? `${i && rows[i - 1].valid ? 'L' : 'M'}${p.x.toFixed(1)},${p.y.toFixed(1)}` : ''}`, '')
}
const tick = (n: number) => Number.isFinite(n) ? Number(n.toPrecision(4)).toString() : '—'
</script>
<template><div><div class="chart-controls"><label v-for="key in numericKeys" :key="key" class="check"><input type="checkbox" :value="key" v-model="selected">{{key}}</label></div><p class="muted small">选择 TSV 中的真实数值列；纵轴使用列内原值与原单位，横轴：{{xKey || '记录序号'}}。{{rows.length.toLocaleString()}} 条记录。</p><svg v-if="rows.length && selected.length && Number.isFinite(domain.ymin)" class="line-chart" viewBox="0 0 850 310" role="img" aria-label="命令 TSV 角度曲线"><g v-for="i in 5" :key="i"><line x1="65" x2="805" :y1="30+(i-1)*57.5" :y2="30+(i-1)*57.5" stroke="#e4e9f0"/><text x="56" :y="34+(i-1)*57.5" text-anchor="end">{{tick(domain.ymax-(i-1)*(domain.ymax-domain.ymin)/4)}}</text></g><path v-for="(key,i) in selected" :key="key" :d="line(key)" fill="none" :stroke="palette[i % palette.length]" stroke-width="1.7"/><text x="65" y="286">{{tick(domain.xmin)}}</text><text x="805" y="286" text-anchor="end">{{tick(domain.xmax)}}</text><text x="430" y="307" text-anchor="middle">{{xKey || '记录序号'}}</text></svg><div v-else class="empty">未产生可绘制的数值记录，请选择数据列。</div><div class="legend"><span v-for="(key,i) in selected" :key="key"><i :style="{background:palette[i % palette.length]}"></i>{{key}}</span></div></div></template>
