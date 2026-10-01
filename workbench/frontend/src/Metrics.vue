<script setup lang="ts">
import { computed } from 'vue'
import { flatten, printable } from './api'
const props = defineProps<{data: any; search?: string}>()
const rows = computed(() => flatten(props.data).filter(row => !props.search || row.key.toLowerCase().includes(props.search.toLowerCase())))
</script>
<template><div class="table-wrap"><table class="metric-table"><thead><tr><th>字段 / 指标</th><th>实际产出</th></tr></thead><tbody><tr v-for="row in rows" :key="row.key"><td class="mono">{{row.key}}</td><td :class="{'muted': printable(row.value) === '未产生'}">{{printable(row.value)}}</td></tr><tr v-if="!rows.length"><td colspan="2" class="empty">未产生</td></tr></tbody></table></div></template>
