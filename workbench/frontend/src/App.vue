<script setup lang="ts">
import { computed, onMounted, onUnmounted, reactive, ref, watch } from 'vue'
import { api, at, copy, flatten, printable, put } from './api'
import type { AnyRecord, Artifact, Build, Contract, Entry, Field, FileList, Job, ModelAcceptance, NucConfiguration, NucRunOptions, NucTarget, Profile, Results } from './api'
import Metrics from './Metrics.vue'
import LineChart from './LineChart.vue'

const tabs = [
  {id:'config', title:'参数配置', subtitle:'运行参数与基础契约', icon:'◫'},
  {id:'algorithms', title:'算法方案', subtitle:'NUC 运行与编译组合', icon:'◇'},
  {id:'data', title:'数据管理', subtitle:'输入文件与合成数据', icon:'▤'},
  {id:'jobs', title:'任务中心', subtitle:'队列、命令与运行日志', icon:'▷'},
  {id:'results', title:'结果分析', subtitle:'报告、帧预览与曲线', icon:'⌁'}
]
const activeTab = ref('config')
const context = ref<AnyRecord>({})
const fields = ref<Field[]>([])
const templateValues = ref<AnyRecord>({})
const contracts = ref<Contract[]>([])
const templateContracts = ref<Contract[]>([])
const loadedValues = ref<AnyRecord>({})
const loadedContracts = ref<Contract[]>([])
const drafts = reactive<Record<string,string | boolean>>({})
const enumOptions: Record<string,string[]> = {
  active_detector:['traditional','yolov5','yolo11'],
  'common.detector.enemy':['red','blue'],
  'common.operator_input.button_mode':['toggle','hold']
}
const profiles = ref<Profile[]>([])
const builds = ref<Build[]>([])
const jobs = ref<Job[]>([])
const connected = ref(false)
const loading = ref(true)
const working = ref(false)
const error = ref('')
const notice = ref('')
const fieldSearch = ref('')
const onlyChanged = ref(false)
const profileName = ref('')
const loadedProfileId = ref('')
const selectedContract = ref('')
const taskBuild = ref('')
const taskProfile = ref('')
const selectedJobId = ref('')
const selectedJob = computed(() => jobs.value.find(j => j.id === selectedJobId.value))
const jobLog = ref('')
let logOffset = 0
let logJobId = ''
let pollTimer: ReturnType<typeof setInterval> | undefined
let pollBusy = false
let playTimer: ReturnType<typeof setInterval> | undefined
const jobKinds: Record<string,string> = {nuc_run:'NUC 运行',nuc_probe:'NUC 连接检查',build:'构建与测试',check_config:'配置检查',synthetic:'合成数据',replay:'离线回放',benchmark:'批量比较',single_image:'单图计时',ctest:'CTest 测试'}
const localJobKinds = Object.fromEntries(Object.entries(jobKinds).filter(([kind]) => !kind.startsWith('nuc_')))
const statuses: Record<string,string> = {queued:'等待中',running:'运行中',succeeded:'已完成',failed:'失败',cancelled:'已取消',interrupted:'已中断'}
const isLive = (status: string) => ['queued','running'].includes(status)
const successBuilds = computed(() => builds.value.filter(b => b.status === 'succeeded'))
const currentTitle = computed(() => tabs.find(tab => tab.id === activeTab.value)!)
const queueCount = computed(() => jobs.value.filter(j => isLive(j.status)).length)
const ready = computed(() => !!taskBuild.value && !!taskProfile.value)

async function action(operation: () => Promise<void>) {
  if (working.value) return
  working.value = true; error.value = ''; notice.value = ''
  try { await operation() } catch (e) { error.value = e instanceof Error ? e.message : String(e) }
  finally { working.value = false }
}
function fill(values: AnyRecord) {
  fields.value.forEach(field => { const value = at(values, field.path); drafts[field.path] = field.type === 'boolean' ? Boolean(value) : field.type === 'array' ? JSON.stringify(value) : String(value ?? '') })
}
function parseField(field: Field): any {
  const raw = drafts[field.path]
  if (field.type === 'boolean') return raw === true
  if (field.type === 'string') { const value = String(raw ?? '');if (!value.trim()) throw new Error('字符串不能为空');return value }
  if (field.type === 'array') { const value = JSON.parse(String(raw)); if (!Array.isArray(value)) throw new Error('请输入 JSON 数组，例如 [1, 2, 3]');if (value.some(item => typeof item !== 'number' || !Number.isFinite(item))) throw new Error('数组各项必须是有限数值'); return value }
  if (String(raw).trim() === '') throw new Error('数值不能为空')
  const value = Number(raw)
  if (!Number.isFinite(value)) throw new Error('数值必须有限')
  if (field.type === 'integer' && !Number.isInteger(value)) throw new Error('请输入整数')
  return value
}
function fieldError(field: Field) { try { parseField(field); return '' } catch (e) { return e instanceof Error ? e.message : '格式无效' } }
function changed(field: Field) { try { return JSON.stringify(parseField(field)) !== JSON.stringify(at(loadedValues.value,field.path)) } catch { return true } }
const changedFields = computed(() => fields.value.filter(changed))
const changedContracts = computed(() => contracts.value.filter(c => c.path !== loadedContracts.value.find(old => old.key === c.key)?.path))
const totalChanges = computed(() => changedFields.value.length + changedContracts.value.length)
const invalidFields = computed(() => fields.value.filter(field => fieldError(field)))
const groups = computed(() => {
  const search = fieldSearch.value.toLowerCase().trim()
  const visible = fields.value.filter(field => (!search || `${field.path} ${field.group} ${field.description}`.toLowerCase().includes(search)) && (!onlyChanged.value || changed(field)))
  return [...new Set(visible.map(f => f.group))].map(name => ({name, fields:visible.filter(f => f.group === name)}))
})
function loadProfile(id: string) {
  const profile = profiles.value.find(p => p.id === id)
  loadedProfileId.value = id
  loadedValues.value = copy(profile?.values ?? templateValues.value)
  contracts.value = copy(profile?.contracts ?? templateContracts.value)
  loadedContracts.value = copy(contracts.value)
  fill(loadedValues.value); profileName.value = profile ? `${profile.name} · 副本` : ''
}
async function saveProfile() {
  await action(async () => {
    if (!profileName.value.trim()) throw new Error('请输入参数方案名称')
    if (invalidFields.value.length) throw new Error(`${invalidFields.value[0].path}：${fieldError(invalidFields.value[0])}`)
    for (const key of ['traditional','yolov5','yolo11']) {
      const field = fields.value.find(f => f.path === `detectors.${key}.config_file`)
      if (field && changed(field)) {
        const path = String(drafts[field.path])
        if (contracts.value.find(c => c.key === key)?.path !== path) await changeBaseContract(key, path)
      }
    }
    const values = copy(templateValues.value)
    fields.value.forEach(field => put(values, field.path, parseField(field)))
    const profile = await api<Profile>('/api/profiles', {name:profileName.value.trim(),values,contracts:contracts.value})
    profiles.value = await api<Profile[]>('/api/profiles')
    loadProfile(profile.id); taskProfile.value = profile.id; notice.value = `已保存独立参数方案「${profile.name}」`
  })
}
const flags = [
  {key:'AUTOAIM_I1_CONTRAST_IRLS',title:'I1 · 对比度 IRLS',detail:'角点精修加权拟合'},
  {key:'AUTOAIM_I2_LM',title:'I2 · LM 优化',detail:'位姿非线性优化'},
  {key:'AUTOAIM_I3_LINEAR_CA',title:'I3 · 平移 CA',detail:'平移恒加速度模型 · 与 ESO 互斥'},
  {key:'AUTOAIM_I9_THROUGHPUT',title:'I9 · THROUGHPUT',detail:'OpenVINO 吞吐提示 · 需要双模型验收'},
  {key:'AUTOAIM_I9_PREALLOC',title:'I9 · 队列预分配',detail:'推理队列资源预分配 · 需要双模型验收'},
  {key:'AUTOAIM_USE_ESO',title:'ESO · 扩张状态',detail:'平移观测器 · 与 I3 互斥'}
]
const buildForm = reactive({name:'baseline',flags:Object.fromEntries(flags.map(f => [f.key,false])) as Record<string,boolean>,openvino:false,yolov5_model:'',yolo11_model:'',openvino_dir:'',build_type:'Release',jobs:2})
const presetId = ref('baseline')
const fallbackPresets = [{id:'baseline',name:'基线',flags:Object.fromEntries(flags.map(f => [f.key,false])),openvino:false}, ...flags.map(f => ({id:f.key,name:f.title,flags:Object.fromEntries(flags.map(g => [g.key,g.key === f.key])),openvino:f.key.startsWith('AUTOAIM_I9')}))]
const presets = computed(() => Array.isArray(context.value.presets) && context.value.presets.length ? context.value.presets : fallbackPresets)
const nucDefaults = (): NucTarget => ({host:'10.141.143.124',user:'xtyf',port:22,identity_file:'',project_dir:'',build_dir:'',executable:'autoaim_node',device_config:'',workspace_dir:'',yolov5_model:'',yolo11_model:''})
const nucConfiguration = ref<NucConfiguration>({target:null,configured:false,ssh_available:false})
const nucDraft = reactive<NucTarget>(nucDefaults())
const nucSaved = ref<NucTarget>(nucDefaults())
const nucProfileId = ref('')
const nucProfile = computed(() => profiles.value.find(profile => profile.id === nucProfileId.value))
const nucTargetSaved = computed(() => !!nucConfiguration.value.target?.host && !!nucConfiguration.value.target?.user)
const nucDirty = computed(() => JSON.stringify(nucDraft) !== JSON.stringify(nucSaved.value))
const nucActiveJob = computed(() => jobs.value.find(job => job.kind === 'nuc_run' && isLive(job.status)))
const nucPresetName = computed(() => presetId.value === 'custom' ? '自定义组合' : presets.value.find((preset: AnyRecord) => preset.id === presetId.value)?.name || presetId.value)
const nucTargetProblems = computed(() => {
  const problems: string[] = []
  if (!nucDraft.host.trim()) problems.push('请填写 NUC 主机地址。')
  if (!nucDraft.user.trim()) problems.push('请填写 SSH 用户。')
  if (!Number.isInteger(nucDraft.port) || nucDraft.port < 1 || nucDraft.port > 65535) problems.push('SSH 端口必须为 1–65535 的整数。')
  return problems
})
const nucRunProblems = computed(() => {
  const problems: string[] = []
  const requiredPaths = {project_dir:'NUC 项目目录',build_dir:'NUC 构建目录',executable:'NUC 可执行程序',device_config:'NUC 设备配置',workspace_dir:'NUC 运行目录'} as const
  for (const [key,label] of Object.entries(requiredPaths)) if (!String(nucDraft[key as keyof NucTarget]).trim()) problems.push(`请填写${label}。`)
  if (buildForm.flags.AUTOAIM_I3_LINEAR_CA && buildForm.flags.AUTOAIM_USE_ESO) problems.push('I3 平移 CA 与 ESO 不能同时启用。')
  if (buildForm.flags.AUTOAIM_I9_THROUGHPUT && !buildForm.openvino) problems.push('THROUGHPUT 要求启用 OpenVINO。')
  if (buildForm.flags.AUTOAIM_I9_PREALLOC && !buildForm.openvino) problems.push('I9 队列预分配要求启用 OpenVINO。')
  const detector = nucProfile.value?.values.active_detector
  if (detector && detector !== 'traditional' && !buildForm.openvino) problems.push('所选 YOLO 检测参数要求启用 OpenVINO。')
  return problems
})
const nucCanConnect = computed(() => nucTargetSaved.value && nucConfiguration.value.ssh_available && !nucDirty.value && !nucTargetProblems.value.length)
function loadNucConfiguration(configuration: NucConfiguration) {
  nucConfiguration.value = configuration
  const target = {...nucDefaults(),...configuration.target}
  if (!configuration.configured) {target.host ||= '10.141.143.124';target.user ||= 'xtyf';target.port ||= 22;target.executable ||= 'autoaim_node'}
  Object.assign(nucDraft,target); nucSaved.value = copy(target)
}
async function saveNucTarget() {
  await action(async () => {
    if (nucTargetProblems.value.length) throw new Error(nucTargetProblems.value.join('\n'))
    const target = copy(nucDraft)
    for (const key of Object.keys(target) as (keyof NucTarget)[]) if (key !== 'port') target[key] = target[key].trim()
    loadNucConfiguration(await api<NucConfiguration>('/api/nuc',target))
    notice.value = 'NUC 连接设置已保存；尚未代表已连接或设备程序已验收。'
  })
}
async function submitNuc(kind: 'nuc_probe' | 'nuc_run') {
  await action(async () => {
    if (!nucCanConnect.value) throw new Error(!nucTargetSaved.value || nucDirty.value ? '请先保存 NUC 连接设置。' : '请先填写主机与 SSH 用户，并确认服务端可使用 SSH。')
    if (kind === 'nuc_probe') {await createJob(kind,{});return}
    if (!nucProfile.value) throw new Error('请明确选择一个已保存参数方案。')
    if (nucRunProblems.value.length) throw new Error(nucRunProblems.value.join('\n'))
    const options: NucRunOptions = {profile_id:nucProfile.value.id,flags:copy(buildForm.flags),openvino:buildForm.openvino,build_type:buildForm.build_type}
    await createJob(kind,options)
  })
}
function openNucRun() {
  if (!profiles.value.some(profile => profile.id === loadedProfileId.value)) return
  nucProfileId.value = loadedProfileId.value;activeTab.value = 'algorithms'
}
function showJob(job: Job) {selectedJobId.value = job.id;activeTab.value = 'jobs'}
function choosePreset(id: string) {
  presetId.value = id
  const preset = presets.value.find((p: AnyRecord) => p.id === id)
  if (!preset) return
  flags.forEach(flag => { buildForm.flags[flag.key] = !!preset.flags?.[flag.key] })
  buildForm.openvino = !!preset.openvino; buildForm.name = id === 'baseline' ? 'baseline' : String(preset.id)
}
const buildProblems = computed(() => {
  const problems: string[] = []
  if (buildForm.flags.AUTOAIM_I3_LINEAR_CA && buildForm.flags.AUTOAIM_USE_ESO) problems.push('I3 平移 CA 与 ESO 不能同时启用。')
  if (buildForm.flags.AUTOAIM_I9_THROUGHPUT && !buildForm.openvino) problems.push('THROUGHPUT 要求启用 OpenVINO。')
  if (buildForm.flags.AUTOAIM_I9_PREALLOC && !buildForm.openvino) problems.push('I9 队列预分配双模型验收要求启用 OpenVINO。')
  if ((buildForm.flags.AUTOAIM_I9_THROUGHPUT || buildForm.flags.AUTOAIM_I9_PREALLOC) && (!buildForm.yolov5_model.trim() || !buildForm.yolo11_model.trim())) problems.push('I9 候选要求提供 YOLOv5 与 YOLO11 两套模型 XML，以执行双模型验收。')
  if (!buildForm.name.trim()) problems.push('请输入编译方案名称。')
  if (!Number.isInteger(buildForm.jobs) || buildForm.jobs < 1) problems.push('编译并行数必须为正整数。')
  return problems
})
function modelAcceptance(build: Build): ModelAcceptance {
  return typeof build.model_acceptance === 'object' && build.model_acceptance !== null ? build.model_acceptance : {status:build.model_acceptance ?? 'not_verified'}
}
function modelAcceptanceLabel(build: Build) {
  if (build.status === 'queued') return '构建与验收排队中'
  if (build.status === 'running') return '构建与验收进行中'
  const status = modelAcceptance(build).status
  return status === 'passed' ? '已通过实际模型验收' : status === 'failed_or_incomplete' ? '实际模型验收失败或未完成' : '尚未验收'
}
function modelAcceptanceClass(build: Build) {
  if (isLive(build.status)) return build.status
  const status = modelAcceptance(build).status
  return status === 'passed' ? 'succeeded' : status === 'failed_or_incomplete' ? 'failed' : 'amber'
}
function providedModels(build: Build) { return modelAcceptance(build).provided ?? Object.keys(build.models ?? {}) }
function missingModels(build: Build) { return modelAcceptance(build).missing ?? ['YOLOV5','YOLO11'].filter(name => !providedModels(build).includes(name)) }
function modelFileName(build: Build, name: string) { return build.models?.[name]?.xml?.split(/[\\/]/).pop() ?? name }
function registeredModelTests(build: Build): number | undefined {
  const acceptance = modelAcceptance(build)
  const required = acceptance.required_tests
  if (!Array.isArray(required)) return undefined
  // The backend writes passed only after strict registry validation and full CTest success.
  if (acceptance.status === 'passed') return required.length
  const registry = build.information?.ctest_registry?.tests
  let registered: string[] | undefined
  if (Array.isArray(registry)) registered = registry.map(test => String(test.name))
  else if (Array.isArray(build.information?.required_tests) && Number.isInteger(build.information?.registered_tests)) registered = build.information.required_tests
  if (!registered) return undefined
  return required.filter(name => registered.includes(name)).length
}
async function submitBuild() { await action(async () => { if (buildProblems.value.length) throw new Error(buildProblems.value.join('\n')); const options = copy(buildForm);if (!options.openvino) {options.yolov5_model='';options.yolo11_model='';options.openvino_dir=''}await createJob('build',options) }) }

const browser = reactive({open:false,path:'',parent:null as string | null,entries:[] as Entry[],target:'',selected:'',preview:'',busy:false,error:''})
const rootPath = ref('')
const dataset = ref('')
const image = ref('')
async function browse(path = '') {
  browser.busy = true; browser.error = ''
  try { const listing = await api<FileList>(`/api/files?path=${encodeURIComponent(path)}`); browser.path = listing.path; browser.parent = listing.parent; browser.entries = listing.entries; browser.selected = ''; browser.preview = '' }
  catch (e) { browser.error = e instanceof Error ? e.message : String(e) }
  finally { browser.busy = false }
}
async function openPicker(target: string) { browser.open = true; browser.target = target; await browse(browser.path) }
function selectEntry(entry: Entry) {
  if (entry.directory) { void browse(entry.path); return }
  browser.selected = entry.path; browser.preview = entry.url ?? (/\.(png|jpe?g)$/i.test(entry.path) ? `/api/file?path=${encodeURIComponent(entry.path)}` : '')
}
async function changeBaseContract(key: string, path: string) {
  const updated = await api<Contract[]>(`/api/config/contracts?path=${encodeURIComponent(path)}&key=${encodeURIComponent(key)}`)
  contracts.value = [...contracts.value.filter(c => c.key !== key && !c.key.startsWith(`${key}.`)), ...updated]
  drafts[`detectors.${key}.config_file`] = path
}
async function useSelected() {
  if (!browser.selected) return
  const path = browser.selected
  if (browser.target === 'dataset') dataset.value = path
  else if (browser.target === 'image') image.value = path
  else if (browser.target.startsWith('contract:')) {
    const key = browser.target.slice(9)
    if (['traditional','yolov5','yolo11'].includes(key)) {
      browser.busy = true; browser.error = ''
      try { await changeBaseContract(key,path) }
      catch (e) { browser.error = e instanceof Error ? e.message : String(e); return }
      finally { browser.busy = false }
    } else { const c = contracts.value.find(c => c.key === key); if (c) {c.path = path;c.content = ''} }
  }
  else if (browser.target === 'yolov5_model' || browser.target === 'yolo11_model') buildForm[browser.target] = path
  browser.open = false
}
async function registerRoot() { await action(async () => { if (!rootPath.value.trim()) throw new Error('请输入服务端目录'); await api('/api/roots',{path:rootPath.value.trim()}); context.value = await api('/api/context'); await browse(rootPath.value.trim()); notice.value = '目录已注册，可以浏览文件。' }) }
const bytes = (size: number) => size < 1024 ? `${size ?? 0} B` : size < 1024 ** 2 ? `${(size / 1024).toFixed(1)} KB` : `${(size / 1024 ** 2).toFixed(1)} MB`

const taskForm = reactive({kind:'check_config',frames:120,iterations:100,iou:0.5,pose_reference:'',position_limit:'',rotation_limit:''})
const benchmarkRuns = ref<{build_id:string;profile_id:string}[]>([{build_id:'',profile_id:''}])
const jobFilter = ref('')
const visibleJobs = computed(() => jobs.value.filter(j => !jobFilter.value || j.status === jobFilter.value))
async function createJob(kind: string, options: AnyRecord) {
  const job = await api<Job>('/api/jobs',{kind,options})
  await refreshLists(); selectedJobId.value = job.id; activeTab.value = 'jobs'; notice.value = `${jobKinds[kind]}已进入队列。`
}
async function submitTask(kind = taskForm.kind) {
  await action(async () => {
    if (!taskBuild.value && kind !== 'benchmark') throw new Error('请选择已成功完成的构建')
    if (kind !== 'ctest' && !taskProfile.value && kind !== 'benchmark') throw new Error('请选择已保存的参数方案')
    const options: AnyRecord = {build_id:taskBuild.value}
    if (kind !== 'ctest') options.profile_id = taskProfile.value
    if (kind === 'synthetic') {if (!Number.isInteger(taskForm.frames) || taskForm.frames < 1) throw new Error('生成帧数必须为正整数');options.frames = taskForm.frames}
    if (kind === 'replay' || kind === 'benchmark') { if (!dataset.value.trim()) throw new Error('请选择输入数据清单'); options.dataset = dataset.value.trim() }
    if (kind === 'single_image') { if (!image.value.trim()) throw new Error('请选择单图输入');if (!Number.isInteger(taskForm.iterations) || taskForm.iterations < 1) throw new Error('单图计时迭代次数必须为正整数'); options.image = image.value.trim(); options.iterations = taskForm.iterations }
    if (kind === 'benchmark') {
      if (!benchmarkRuns.value.length || benchmarkRuns.value.some(run => !run.build_id || !run.profile_id)) throw new Error('每个比较项都需要已成功构建和已保存参数方案')
      if (!Number.isFinite(taskForm.iou) || taskForm.iou <= 0 || taskForm.iou > 1) throw new Error('IoU 必须为 (0,1] 范围内有限数值')
      const poseInputs = [!!taskForm.pose_reference.trim(),taskForm.position_limit !== '',taskForm.rotation_limit !== '']
      if (poseInputs.some(Boolean) && !poseInputs.every(Boolean)) throw new Error('位姿评测需同时填写参考系标识、真值位置与旋转不确定度上限')
      if (poseInputs.every(Boolean) && [taskForm.position_limit,taskForm.rotation_limit].some(value => !Number.isFinite(Number(value)) || Number(value) < 0)) throw new Error('真值不确定度上限必须为有限非负数')
      delete options.build_id; delete options.profile_id
      options.runs = copy(benchmarkRuns.value); options.iou = taskForm.iou
      if (taskForm.pose_reference.trim()) options.pose_reference = taskForm.pose_reference.trim()
      if (taskForm.position_limit !== '') options.position_limit = Number(taskForm.position_limit)
      if (taskForm.rotation_limit !== '') options.rotation_limit = Number(taskForm.rotation_limit)
    }
    await createJob(kind,options)
  })
}
async function refreshLists() {
  const responses = await Promise.all([api<Profile[]>('/api/profiles'),api<Build[]>('/api/builds'),api<Job[]>('/api/jobs')])
  profiles.value = responses[0]; builds.value = responses[1]; jobs.value = responses[2]
  if (!taskBuild.value && successBuilds.value.length) taskBuild.value = successBuilds.value[0].id
  if (!taskProfile.value && profiles.value.length) taskProfile.value = profiles.value[0].id
  if (!benchmarkRuns.value[0]?.build_id && taskBuild.value && benchmarkRuns.value[0]) benchmarkRuns.value[0].build_id = taskBuild.value
  if (!benchmarkRuns.value[0]?.profile_id && taskProfile.value && benchmarkRuns.value[0]) benchmarkRuns.value[0].profile_id = taskProfile.value
}
async function poll() {
  if (pollBusy || !fields.value.length) return
  pollBusy = true
  try {
    await refreshLists()
    if (selectedJobId.value) {
      const id = selectedJobId.value
      if (logJobId !== id) {logOffset = 0;jobLog.value = '';logJobId = id}
      const chunk = await api<{text:string;next_offset:number}>(`/api/jobs/${encodeURIComponent(id)}/log?offset=${logOffset}`)
      if (selectedJobId.value === id) { jobLog.value += chunk.text; logOffset = chunk.next_offset }
    }
    connected.value = true
  } catch (e) {connected.value = false; error.value = `连接 / 轮询失败：${e instanceof Error ? e.message : String(e)}` }
  finally { pollBusy = false }
}
watch(selectedJobId, () => { jobLog.value = ''; logOffset = 0; logJobId = ''; void poll() })
async function cancelJob(job: Job) { await action(async () => { await api(`/api/jobs/${encodeURIComponent(job.id)}/cancel`,{}); await refreshLists(); notice.value = job.kind === 'nuc_run' ? '已提交停止 NUC 运行请求，请查看任务状态与日志确认远端停止结果。' : '已提交取消请求，日志与已生成产物将保留。' }) }

const resultJobId = ref('')
const results = ref<Results>({reports:[],frames:[],commands:[]})
const artifacts = ref<Artifact[]>([])
const resultLoading = ref(false)
const frameIndex = ref(0)
const playing = ref(false)
const reportSearch = ref('')
const compareIds = ref<string[]>([])
const compareSearch = ref('')
const comparisons = ref<{job:Job;results:Results}[]>([])
const artifactSearch = ref('')
const curveRows = ref<AnyRecord[]>([])
const curveArtifact = ref('')
const currentFrame = computed(() => results.value.frames[frameIndex.value])
const resultJob = computed(() => jobs.value.find(j => j.id === resultJobId.value))
const resultJobs = computed(() => jobs.value.filter(j => j.kind !== 'build' && j.kind !== 'ctest'))
const filteredArtifacts = computed(() => artifacts.value.filter(a => `${a.name} ${a.path}`.toLowerCase().includes(artifactSearch.value.toLowerCase())))
const tsvArtifacts = computed(() => artifacts.value.filter(a => /\.tsv$/i.test(a.path)))
const originalFrameUrl = computed(() => {
  return currentFrame.value?.original_url ?? ''
})
const comparisonRows = computed(() => results.value.comparison?.rows ?? [])
const comparisonMetrics = [
  {label:'检测精确率',path:'summary.precision'},
  {label:'检测召回率',path:'summary.recall'},
  {label:'可用处理链召回率',path:'summary.usable_chain_recall'},
  {label:'平均角点误差（px）',path:'summary.mean_corner_error_px'},
  {label:'角点 RMS（px）',path:'summary.corner_rms_px'},
  {label:'最长连续空检测',path:'logic.maximum_empty_detection_streak'},
  {label:'最长连续跟踪丢失',path:'logic.maximum_track_loss_streak'},
  {label:'队列丢弃',path:'logic.queue_drops'},
  {label:'结果丢弃',path:'logic.result_drops'},
  {label:'初始化（ms）',path:'timing.initialization_ms'},
  {label:'回放总墙钟时间（ms）',path:'timing.replay_wall_ms'},
  {label:'预处理均值（ms）',path:'timing.preprocessing.mean_ms'},
  {label:'推理墙钟时间均值（ms）',path:'timing.inference_wall.mean_ms'},
  {label:'后处理均值（ms）',path:'timing.postprocessing.mean_ms'}
]
const effectiveParameterDiffs = computed(() => {
  const parameters = comparisonRows.value.map(row => flatten(row.logic?.effective_configuration ?? {}))
  const keys = [...new Set(parameters.flatMap(rows => rows.map(row => row.key)))]
  return keys.map(key => ({key,values:parameters.map(rows => rows.find(row => row.key === key)?.value)})).filter(row => new Set(row.values.map(stableValue)).size > 1)
})
const comparedFields = computed(() => {
  const jobs = new Map<string,Map<string,any>>()
  for (const item of comparisons.value) {
    const values = new Map<string,any>()
    for (const report of item.results.reports) {
      for (const field of flatten(report.data,report.name)) values.set(field.key,field.value)
    }
    jobs.set(item.job.id,values)
  }
  return jobs
})
const compareColumns = computed(() => [...new Set([...comparedFields.value.values()].flatMap(values => [...values.keys()]))])
const filteredCompareColumns = computed(() => compareColumns.value.filter(key => key.toLowerCase().includes(compareSearch.value.trim().toLowerCase())))
function comparedValue(item: {job:Job}, key:string) { return printable(comparedFields.value.get(item.job.id)?.get(key)) }
function stableValue(value: any): string {
  if (value === undefined) return '未产生'
  if (value === null) return 'null'
  if (Array.isArray(value)) return `[${value.map(stableValue).join(',')}]`
  if (typeof value === 'object') return `{${Object.keys(value).sort().map(key => `${JSON.stringify(key)}:${stableValue(value[key])}`).join(',')}}`
  return JSON.stringify(value)
}
const compareAudit = computed(() => {
  const reasons: string[] = []
  const samples = comparisons.value.map(item => {
    const reports = item.results.reports.filter(r => r.kind === 'logic' && r.data && typeof r.data === 'object')
    if (!reports.length) reasons.push(`${item.job.id}：未产生逻辑报告，不能比较。`)
    const conditions = reports.flatMap(report => {
      const data = report.data
      if (!Array.isArray(data.runs) || !data.runs.length) { reasons.push(`${item.job.id}：逻辑报告缺少实际 runs。`);return [] }
      return data.runs.map((run: AnyRecord) => ({dataset_fingerprint:data.dataset_fingerprint,enemy:run.effective_configuration?.detector?.enemy,minimum_iou:data.minimum_iou,pose_metrics:data.pose_metrics,pose_reference:data.pose_reference ?? null,position_limit:item.job.options.position_limit ?? null,rotation_limit:item.job.options.rotation_limit ?? null}))
    })
    for (const key of ['dataset_fingerprint','enemy','minimum_iou','pose_metrics']) {
      if (conditions.some(c => c[key] === undefined || c[key] === null)) reasons.push(`${item.job.id}：缺少必要比较条件 ${key}。`)
    }
    if (item.results.comparison && !item.results.comparison.compatible) reasons.push(`${item.job.id}：本任务报告条件尚不兼容：${item.results.comparison.reasons.join('；')}`)
    return conditions
  })
  const conditions = ['dataset_fingerprint','enemy','minimum_iou','pose_metrics','pose_reference','position_limit','rotation_limit'].map(key => {
    const values = samples.map(sample => [...new Set(sample.map(c => stableValue(c[key])))].join(' | '))
    const compatible = samples.every(sample => sample.length && new Set(sample.map(c => stableValue(c[key]))).size === 1) && new Set(values).size === 1 && !values.includes('未产生')
    if (!compatible) reasons.push(`比较条件 ${key} 不一致、缺失或未验证。`)
    return {key,values,compatible}
  })
  return {conditions,reasons:[...new Set(reasons)],compatible:comparisons.value.length >= 2 && !reasons.length}
})
function stopPlayback() { if (playTimer) clearInterval(playTimer); playTimer = undefined;playing.value = false }
function togglePlayback() {
  if (playing.value) {stopPlayback();return}
  playing.value = true
  playTimer = setInterval(() => { if (frameIndex.value + 1 >= results.value.frames.length) stopPlayback(); else frameIndex.value++ }, 180)
}
async function loadResults(id = resultJobId.value) {
  if (!id) return
  resultJobId.value = id; activeTab.value = 'results';resultLoading.value = true;stopPlayback();error.value = ''
  try {
    const [data, files] = await Promise.all([api<Results>(`/api/jobs/${encodeURIComponent(id)}/results`),api<Artifact[]>(`/api/jobs/${encodeURIComponent(id)}/artifacts`)])
    results.value = {reports:data.reports ?? [],frames:data.frames ?? [],commands:data.commands ?? [],comparison:data.comparison}; artifacts.value = files
    frameIndex.value = 0;curveArtifact.value = '';curveRows.value = results.value.commands
  } catch (e) {error.value = e instanceof Error ? e.message : String(e)}
  finally {resultLoading.value = false}
}
async function loadCurve() {
  await action(async () => {
    if (!curveArtifact.value) {curveRows.value = results.value.commands;return}
    const artifact = artifacts.value.find(a => a.path === curveArtifact.value)!
    const response = await fetch(artifact.url);if (!response.ok) throw new Error(`TSV 读取失败：HTTP ${response.status}`)
    const lines = (await response.text()).split(/\r?\n/).filter(line => line.trim() && !line.startsWith('#'))
    const headers = lines.shift()?.split('\t') ?? []
    curveRows.value = lines.map(line => {const values = line.split('\t');return Object.fromEntries(headers.map((header,i) => [header, values[i] ?? '']))})
  })
}
async function compareReports() { await action(async () => { comparisons.value = await Promise.all(compareIds.value.map(async id => ({job:jobs.value.find(j => j.id === id)!,results:await api<Results>(`/api/jobs/${encodeURIComponent(id)}/results`)}))) }) }
function contractLabel(key:string) {
  const names: Record<string,string> = {traditional:'传统检测',yolov5:'YOLOv5',yolo11:'YOLO11'}
  const [detector,resource] = key.split('.')
  return names[detector] ? `${names[detector]} · ${!resource?'基础配置':resource==='calibration_file'?'相机标定':resource==='geometry_files'?'几何模型 '+(Number(key.split('.')[2])+1):resource}` : key
}
function timestamp(value?:string) { return value ? new Date(value).toLocaleString('zh-CN',{hour12:false}) : '—' }
function buildName(id:string) {return builds.value.find(b => b.id === id)?.name ?? id}
function profileLabel(id:string) {return profiles.value.find(p => p.id === id)?.name ?? id}
async function initialize() {
  loading.value = true;error.value = ''
  try {
    const [ctx,template,nuc] = await Promise.all([api<AnyRecord>('/api/context'),api<{values:AnyRecord;fields:Field[];contracts:Contract[]}>('/api/config/template'),api<NucConfiguration>('/api/nuc')])
    context.value = ctx;fields.value = template.fields;templateValues.value = template.values;templateContracts.value = template.contracts
    loadNucConfiguration(nuc)
    loadProfile(''); await refreshLists();connected.value = true;await browse('');await poll()
  } catch (e) {error.value = e instanceof Error ? e.message : String(e);connected.value = false}
  finally {loading.value = false}
}
onMounted(() => {void initialize();pollTimer = setInterval(() => {void poll()},2000)})
onUnmounted(() => {if (pollTimer) clearInterval(pollTimer);stopPlayback()})
</script>

<template>
  <div class="app-shell">
    <aside class="sidebar">
      <div class="brand"><div class="brand-mark"><span></span><span></span><span></span></div><div><strong>XTYF AUTOAIM</strong><small>实验与设备工作台</small></div></div>
      <div class="sidebar-label">EXPERIMENT WORKSPACE</div>
      <nav aria-label="工作区"><button v-for="tab in tabs" :key="tab.id" :class="{active:activeTab === tab.id}" @click="activeTab=tab.id"><span class="nav-icon">{{tab.icon}}</span><span><strong>{{tab.title}}</strong><small>{{tab.subtitle}}</small></span><i v-if="tab.id==='jobs' && queueCount" class="nav-count">{{queueCount}}</i></button></nav>
      <div class="sidebar-bottom"><div class="connection"><i :class="{online:connected}"></i>{{connected ? '本地服务已连接' : '服务未连接'}}</div><div class="mono small">{{context.platform || '等待服务信息'}}</div><div class="small muted">任务串行执行 · 原始输入只读</div></div>
    </aside>
    <main class="main">
      <header class="topbar"><div class="crumb">工作台 <span>/</span> {{currentTitle.title}}</div><div class="topbar-actions"><span class="badge subtle">{{profiles.length}} 个参数方案</span><span class="badge subtle">{{successBuilds.length}} 个可用构建</span><button class="icon-button" title="刷新服务与任务" :disabled="working || loading" @click="action(initialize)">↻</button></div></header>
      <div class="page-content">
        <div class="page-heading"><div><div class="eyebrow">{{activeTab==='config'?'RUNTIME CONFIGURATION':activeTab==='algorithms'?'BUILD PROFILES':activeTab==='data'?'INPUT DATA':activeTab==='jobs'?'EXECUTION QUEUE':'EXPERIMENT RESULTS'}}</div><h1>{{currentTitle.title}}</h1><p>{{currentTitle.subtitle}}。本地实验与远端设备运行由工作台服务统一管理。</p></div><div class="workspace-note"><span>实验目录</span><code :title="context.workspace_root">{{context.workspace_root || '未连接'}}</code></div></div>
        <div v-if="error" class="alert danger" role="alert"><strong>操作未完成</strong><pre>{{error}}</pre><button class="dismiss" aria-label="关闭错误" @click="error=''">×</button></div>
        <div v-if="notice" class="alert success" role="status">{{notice}}<button class="dismiss" aria-label="关闭通知" @click="notice=''">×</button></div>
        <div v-if="loading" class="panel empty loading">正在读取模板、构建与任务…</div>

        <section v-show="activeTab==='config'" class="config-layout">
          <div class="panel config-main"><div class="panel-heading"><div><h2>运行参数</h2><p>{{fields.length}} 个模板叶字段 · 修改在新任务启动时生效</p></div><span class="badge" :class="totalChanges?'amber':'subtle'">{{totalChanges}} 项更改</span></div>
            <div class="config-toolbar"><select aria-label="加载参数方案" :value="loadedProfileId" @change="loadProfile(($event.target as HTMLSelectElement).value)"><option value="">项目原始模板</option><option v-for="p in profiles" :key="p.id" :value="p.id">{{p.name}}</option></select><input v-model="fieldSearch" type="search" placeholder="搜索字段、单位、范围或注释" aria-label="搜索配置字段"><label class="check"><input v-model="onlyChanged" type="checkbox">仅看更改</label></div>
            <div class="config-groups"><details v-for="group in groups" :key="group.name" open class="field-group"><summary>{{group.name}}<span>{{group.fields.length}} 项</span></summary><div v-for="field in group.fields" :key="field.path" class="field-row" :class="{changed:changed(field),invalid:fieldError(field)}"><div class="field-info"><label :for="'field-'+field.path" class="mono">{{field.path}}</label><p>{{field.description || '模板未提供额外注释'}}</p><span class="type-label">{{field.type === 'integer'?'整数':field.type === 'number'?'数值':field.type === 'boolean'?'布尔':field.type === 'array'?'数组 · 整项替换':'字符串'}}</span></div><div class="field-control"><label v-if="field.type==='boolean'" class="switch"><input :id="'field-'+field.path" v-model="drafts[field.path]" type="checkbox"><span></span><em>{{drafts[field.path]?'true':'false'}}</em></label><select v-else-if="enumOptions[field.path]" :id="'field-'+field.path" v-model="drafts[field.path]"><option v-for="option in enumOptions[field.path]" :key="option" :value="option">{{option}}</option></select><textarea v-else-if="field.type==='array'" :id="'field-'+field.path"  :value="String(drafts[field.path])" @input="drafts[field.path]=($event.target as HTMLTextAreaElement).value" rows="2" spellcheck="false" class="mono"></textarea><input v-else :id="'field-'+field.path" v-model="drafts[field.path]" :type="field.type==='number'||field.type==='integer'?'number':'text'" :step="field.type==='integer'?'1':'any'" spellcheck="false"><small v-if="fieldError(field)" class="error-text">{{fieldError(field)}}</small><div v-if="changed(field)" class="field-old"><span>原值 {{printable(at(loadedValues,field.path))}}</span><button class="text-button" @click="drafts[field.path]=field.type==='boolean'?Boolean(at(loadedValues,field.path)):field.type==='array'?JSON.stringify(at(loadedValues,field.path)):String(at(loadedValues,field.path))">恢复此项</button></div></div></div></details><div v-if="!groups.length" class="empty">没有符合搜索条件的字段。</div></div>
          </div>
          <div class="config-side"><div class="panel"><div class="panel-heading"><h2>另存参数方案</h2></div><div class="panel-body stack"><label>方案名称<input v-model="profileName" placeholder="例如：传统检测 · 回放 A"></label><button class="primary full" :disabled="working || !fields.length || !!invalidFields.length" @click="saveProfile">{{working?'正在处理…':'保存独立配置包'}}</button><p class="small muted">保存会复制基础配置并修正资源路径；已提交任务使用固定快照。</p><button :disabled="!loadedProfileId || working" @click="openNucRun">到 NUC 运行已保存方案 →</button><p v-if="loadedProfileId && totalChanges" class="small muted">跳转使用已保存版本；当前草稿需另存后才能应用。</p><div class="button-row"><button :disabled="!totalChanges" @click="fill(loadedValues);contracts=copy(loadedContracts)">恢复所载方案</button><button :disabled="!fields.length" @click="fill(templateValues);contracts=copy(templateContracts)">恢复模板值</button></div><div v-if="invalidFields.length" class="inline-warning">{{invalidFields.length}} 个字段格式无效，请修正后保存。</div></div></div>
            <div class="panel"><div class="panel-heading"><h2>基础标定与几何契约</h2></div><div class="panel-body stack"><div v-for="contract in contracts" :key="contract.key" class="contract-card"><strong>{{contractLabel(contract.key)}}</strong><code>{{contract.path}}</code><div class="button-row"><button class="small-button" @click="selectedContract=selectedContract===contract.key?'':contract.key">{{selectedContract===contract.key?'收起':'查看 YAML'}}</button><button class="small-button" @click="openPicker('contract:'+contract.key)">选择文件</button></div><pre v-if="selectedContract===contract.key" class="contract-source">{{contract.content || '新选择的文件将在保存时由服务读取并复制。'}}</pre></div><p class="small muted">在这里查看、选择基础文件。相机标定、尺寸与角点顺序应来自已有有效配置。</p></div></div>
            <div v-if="changedFields.length || changedContracts.length" class="panel"><div class="panel-heading"><h2>当前差异</h2></div><div class="diff-list"><div v-for="field in changedFields" :key="field.path"><code>{{field.path}}</code><del>{{printable(at(loadedValues,field.path))}}</del><ins>{{drafts[field.path]}}</ins></div><div v-for="c in changedContracts" :key="c.key"><code>基础配置 · {{c.key}}</code><ins>{{c.path}}</ins></div></div></div>
          </div>
        </section>

        <section v-show="activeTab==='algorithms'" class="stack large-gap">
          <div class="panel nuc-panel">
            <div class="panel-heading"><div><div class="eyebrow">REMOTE DEVICE · SSH</div><h2>在 NUC 上运行方案</h2><p>由工作台服务通过 SSH 启动 Ubuntu 设备上的程序，日志在任务中心保留。</p></div><div class="button-row"><span class="badge" :class="nucDirty || !nucTargetSaved?'amber':'subtle'">{{nucDirty || !nucTargetSaved?'连接设置尚未保存':nucConfiguration.configured?'运行设置已保存':'待补全运行设置'}}</span><button class="primary" :disabled="working || !nucCanConnect || !nucProfile || !!nucRunProblems.length" @click="submitNuc('nuc_run')">在 NUC 上运行</button></div></div>
            <div class="panel-body stack">
              <div class="callout">{{nucConfiguration.live_entry_note || '当前项目入口只支持离线；NUC 须有支持设备配置及 --check-config/--config 的实时程序。'}}</div>
              <details class="nuc-target" :open="!nucConfiguration.configured">
                <summary><strong>NUC 连接与程序设置</strong><span class="mono">{{nucSaved.user || '用户'}}@{{nucSaved.host || '主机'}}:{{nucSaved.port}}</span></summary>
                <div class="form-grid three">
                  <label>NUC 主机地址<input v-model="nucDraft.host" placeholder="IP 地址或主机名" autocomplete="off"></label>
                  <label>SSH 用户<input v-model="nucDraft.user" placeholder="Ubuntu 用户名" autocomplete="off"></label>
                  <label>SSH 端口<input v-model.number="nucDraft.port" type="number" min="1" max="65535" step="1"></label>
                  <label class="span-all">SSH 私钥路径（可选，工作台服务所在电脑）<input v-model="nucDraft.identity_file" placeholder="留空使用 SSH 默认密钥或 agent；这里只填写路径" autocomplete="off" spellcheck="false"></label>
                  <label>NUC 项目目录<input v-model="nucDraft.project_dir" placeholder="NUC 上的项目绝对路径" spellcheck="false"></label>
                  <label>NUC 构建目录<input v-model="nucDraft.build_dir" placeholder="NUC 上已完成编译的目录" spellcheck="false"></label>
                  <label>NUC 可执行程序<input v-model="nucDraft.executable" placeholder="autoaim_node" spellcheck="false"></label>
                  <label>NUC 设备配置<input v-model="nucDraft.device_config" placeholder="NUC 上实时设备配置的绝对路径" spellcheck="false"></label>
                  <label>NUC 运行目录<input v-model="nucDraft.workspace_dir" placeholder="NUC 上存放运行快照和日志的目录" spellcheck="false"></label>
                  <label>NUC YOLOv5 模型 XML（可选）<input v-model="nucDraft.yolov5_model" placeholder="NUC 上模型 XML 的绝对路径" spellcheck="false"></label>
                  <label>NUC YOLO11 模型 XML（可选）<input v-model="nucDraft.yolo11_model" placeholder="NUC 上模型 XML 的绝对路径" spellcheck="false"></label>
                </div>
                <p class="small muted">除 SSH 私钥外，目录、设备配置和模型路径均指向 NUC。程序名默认 autoaim_node，位于所填构建目录。先填写主机、用户和端口即可检查 SSH；运行前补全程序与目录。</p>
                <div v-if="nucTargetProblems.length" class="inline-warning"><div v-for="problem in nucTargetProblems" :key="problem">{{problem}}</div></div>
                <div class="form-footer"><span class="small muted">{{nucDirty?'更改尚未保存；检查与运行只使用已保存设置。':'已填写设置不代表 SSH 已连通；通过检查任务查看实际结果。'}}</span><div class="button-row"><button :disabled="working || !nucDirty" @click="Object.assign(nucDraft,copy(nucSaved))">恢复已保存设置</button><button :disabled="working || !!nucTargetProblems.length" @click="saveNucTarget">保存连接设置</button></div></div>
              </details>
              <div class="nuc-selection">
                <div class="subheading"><h3>本次运行的参数与编译方案</h3><span class="badge subtle">{{nucPresetName}}</span></div>
                <div class="form-grid three">
                  <label>已保存参数方案<select v-model="nucProfileId"><option value="">请明确选择已保存方案</option><option v-for="profile in profiles" :key="profile.id" :value="profile.id">{{profile.name}}</option></select></label>
                  <label>要求的构建类型<select v-model="buildForm.build_type"><option>Release</option><option>Debug</option></select></label>
                  <label class="check form-check"><input v-model="buildForm.openvino" type="checkbox">要求启用 OpenVINO</label>
                </div>
                <div class="preset-grid"><button v-for="preset in presets" :key="preset.id" :class="{selected:presetId===preset.id}" @click="choosePreset(preset.id)">{{preset.name}}</button><button :class="{selected:presetId==='custom'}" @click="presetId='custom';buildForm.name='custom'">自定义组合</button></div>
                <div class="nuc-flag-grid"><label v-for="flag in flags" :key="flag.key" class="nuc-flag" :class="{enabled:buildForm.flags[flag.key]}"><input v-model="buildForm.flags[flag.key]" type="checkbox" @change="presetId='custom'"><span><strong>{{flag.title}}</strong><code>{{flag.key}}</code></span><b>{{buildForm.flags[flag.key]?'ON':'OFF'}}</b></label></div>
                <div class="nuc-summary"><div><span>参数方案</span><strong>{{nucProfile?.name || '尚未选择'}}</strong><code>{{nucProfile?.id || '仅使用已保存参数，不读取编辑草稿'}}</code></div><div><span>检测器 active_detector</span><strong>{{nucProfile?.values.active_detector || '尚未选择'}}</strong><code>{{buildForm.openvino?'OpenVINO ON':'OpenVINO OFF'}} · {{buildForm.build_type}}</code></div><div><span>远端程序</span><strong>{{nucSaved.executable || '尚未填写'}}</strong><code>{{nucSaved.build_dir || '尚未填写 NUC 构建目录'}}</code></div></div>
                <p class="small muted">这些编译开关与下方构建表单同步。运行前核对 NUC 已编译程序的构建信息，不自动编译 NUC，也不依赖本机构建记录。</p>
                <div v-if="totalChanges" class="inline-warning">参数编辑器有 {{totalChanges}} 项未保存更改。本次仅使用上方选中的已保存方案；要应用草稿，请先另存，再选择新方案。</div>
                <div v-if="nucRunProblems.length" class="inline-warning"><div v-for="problem in nucRunProblems" :key="problem">{{problem}}</div></div>
              </div>
              <div v-if="!nucConfiguration.ssh_available" class="inline-warning">工作台服务端未找到可用 SSH 客户端，连接检查与远端运行暂不可用。</div>
              <div v-if="nucDirty || !nucTargetSaved" class="inline-warning">请先保存连接设置，再检查或运行。主机与 SSH 用户已按提供的信息预填。</div>
              <div v-if="!nucProfile" class="small muted">运行前请在上方明确选择已保存参数方案。</div>
              <div v-if="nucActiveJob" class="nuc-active"><div><span class="badge" :class="nucActiveJob.status">{{statuses[nucActiveJob.status]}}</span><code>{{nucActiveJob.id}}</code></div><div class="button-row"><button @click="showJob(nucActiveJob)">查看运行日志</button><button class="danger-button" :disabled="working" @click="cancelJob(nucActiveJob)">停止 NUC 运行</button></div></div>
              <div class="form-footer"><span class="small muted">提交后自动打开任务中心。运行使用固定参数快照；停止结果以远端日志为准。</span><div class="button-row"><button :disabled="working || !nucCanConnect" @click="submitNuc('nuc_probe')">检查 NUC</button></div></div>
            </div>
          </div>
          <div class="panel"><div class="panel-heading"><div><h2>创建独立编译方案</h2><p>编译开关显式传入 CMake；每个组合单独构建并运行完整 CTest。</p></div><span class="badge subtle">C++17</span></div><div class="panel-body"><div class="preset-grid"><button v-for="preset in presets" :key="preset.id" :class="{selected:presetId===preset.id}" @click="choosePreset(preset.id)">{{preset.name}}</button><button :class="{selected:presetId==='custom'}" @click="presetId='custom';buildForm.name='custom'">自定义组合</button></div><div class="flag-grid"><label v-for="flag in flags" :key="flag.key" class="flag-card" :class="{enabled:buildForm.flags[flag.key]}"><input v-model="buildForm.flags[flag.key]" type="checkbox" @change="presetId='custom'"><div><strong>{{flag.title}}</strong><p>{{flag.detail}}</p><code>{{flag.key}}</code></div></label></div><div class="form-grid four"><label>构建名称<input v-model="buildForm.name" placeholder="独立构建名称"></label><label>构建类型<select v-model="buildForm.build_type"><option>Release</option><option>Debug</option></select></label><label>编译并行数<input v-model.number="buildForm.jobs" type="number" min="1" step="1"></label><label class="check form-check"><input v-model="buildForm.openvino" type="checkbox">启用 OpenVINO</label></div><div v-if="buildForm.openvino || buildForm.flags.AUTOAIM_I9_PREALLOC" class="form-grid"><label>YOLOv5 模型 XML<div class="input-action"><input v-model="buildForm.yolov5_model" placeholder="服务端模型 XML 路径"><button @click="openPicker('yolov5_model')">选择</button></div></label><label>YOLO11 模型 XML<div class="input-action"><input v-model="buildForm.yolo11_model" placeholder="服务端模型 XML 路径"><button @click="openPicker('yolo11_model')">选择</button></div></label><label class="span-all">OpenVINO CMake 包目录（可选）<input v-model="buildForm.openvino_dir" placeholder="留空使用环境中的 OpenVINO_DIR"></label></div><div class="callout">两种 I9 候选必须通过 YOLOv5 + YOLO11 实际模型验收。XML 与配套 BIN 必须存在；缺少模型或未完成实际推理时会记录未验收状态。</div><div v-if="buildProblems.length" class="inline-warning"><div v-for="problem in buildProblems" :key="problem">{{problem}}</div></div><div class="form-footer"><span class="small muted">构建产物保存在实验目录中的独立副本。</span><button class="primary" :disabled="working || !!buildProblems.length" @click="submitBuild">创建构建任务</button></div></div></div>
          <div class="panel"><div class="panel-heading"><h2>构建与验收记录</h2><button @click="action(refreshLists)">刷新</button></div><div class="table-wrap"><table><thead><tr><th>编译方案</th><th>组合 / 检测运行时</th><th>状态</th><th>模型与验收</th><th>产物目录</th></tr></thead><tbody><tr v-for="build in builds" :key="build.id"><td><strong>{{build.name}}</strong><small>{{build.build_type}}</small></td><td><div class="tag-list"><span v-for="flag in flags.filter(f=>build.flags?.[f.key])" :key="flag.key" class="badge subtle">{{flag.title}}</span><span v-if="!Object.values(build.flags || {}).some(Boolean)" class="badge subtle">基线</span></div><small>{{build.openvino?'OpenVINO':'关闭 OpenVINO'}}</small></td><td><span class="badge" :class="build.status">{{statuses[build.status] || build.status}}</span></td><td class="model-acceptance-cell"><span class="badge" :class="modelAcceptanceClass(build)">{{modelAcceptanceLabel(build)}}</span><div class="provided-models"><span v-for="name in providedModels(build)" :key="name" class="badge subtle" :title="build.models?.[name]?.xml">{{name}} · {{modelFileName(build,name)}}</span><small v-if="!providedModels(build).length">未提供实际模型</small></div><small>注册模型测试：{{registeredModelTests(build) ?? '未产生'}}</small><small v-if="missingModels(build).length" class="model-missing">未验收模型：{{missingModels(build).join('、')}}（未提供）</small><details><summary>实际验收记录</summary><pre class="small-pre">{{JSON.stringify({model_acceptance:build.model_acceptance,models:build.models,information:build.information},null,2)}}</pre></details></td><td class="mono path-cell">{{build.build_dir}}</td></tr><tr v-if="!builds.length"><td colspan="5" class="empty">尚无构建。首次可以选择基线、关闭 OpenVINO 并构建。</td></tr></tbody></table></div></div>
        </section>

        <section v-show="activeTab==='data'" class="stack large-gap">
          <div class="two-column"><div class="panel"><div class="panel-heading"><h2>输入清单与单图</h2></div><div class="panel-body stack"><label>离线数据清单<div class="input-action"><input v-model="dataset" placeholder="events.yaml 的服务端路径"><button @click="openPicker('dataset')">浏览选择</button></div></label><label>单图计时输入<div class="input-action"><input v-model="image" placeholder="PNG / JPEG 的服务端路径"><button @click="openPicker('image')">浏览选择</button></div></label><p class="small muted">路径使用运行服务的系统格式；WSL 中填写 /mnt/d/… 或 Linux 路径。</p><div class="button-row"><button @click="taskForm.kind='replay';activeTab='jobs'">创建离线回放</button><button @click="taskForm.kind='single_image';activeTab='jobs'">创建单图计时</button></div></div></div><div class="panel"><div class="panel-heading"><h2>生成合成数据</h2><span class="badge subtle">流程验证</span></div><div class="panel-body stack"><div class="form-grid"><label>已成功构建<select v-model="taskBuild"><option value="">请选择</option><option v-for="b in successBuilds" :key="b.id" :value="b.id">{{b.name}}</option></select></label><label>已保存参数方案<select v-model="taskProfile"><option value="">请选择</option><option v-for="p in profiles" :key="p.id" :value="p.id">{{p.name}}</option></select></label></div><label>生成帧数<input v-model.number="taskForm.frames" type="number" min="1" step="1"></label><button class="primary" :disabled="working || !ready" @click="submitTask('synthetic')">执行 synthetic_sim</button><p class="small muted">生成图片、事件清单与真值。完成后在结果产物中选择 events.yaml 继续回放。</p></div></div></div>
          <div class="panel"><div class="panel-heading"><div><h2>服务端文件浏览</h2><p>只列出服务允许访问的目录；注册输入目录后即可选择清单、图像、模型和基础配置。</p></div></div><div class="panel-body"><div class="input-action"><input v-model="rootPath" placeholder="显式注册目录，例如 /mnt/d/datasets 或 /home/user/models" aria-label="注册目录路径"><button :disabled="working || !rootPath.trim()" @click="registerRoot">注册目录</button></div><div class="root-list"><button v-for="root in context.roots" :key="typeof root==='string'?root:root.path" class="path-pill" @click="browse(typeof root==='string'?root:root.path)">{{typeof root==='string'?root:root.path}}</button></div><div class="browser-toolbar"><button :disabled="!browser.parent || browser.busy" @click="browse(browser.parent || '')">↑ 上级目录</button><input v-model="browser.path" aria-label="浏览目录路径" @keydown.enter="browse(browser.path)"><button :disabled="browser.busy" @click="browse(browser.path)">前往 / 刷新</button></div><div v-if="browser.error" class="inline-warning">{{browser.error}}</div><div class="table-wrap"><table><thead><tr><th>名称</th><th>类型</th><th>大小</th><th>操作</th></tr></thead><tbody><tr v-for="entry in browser.entries" :key="entry.path"><td><button class="file-name" @click="selectEntry(entry)">{{entry.directory?'▰':'▱'}} {{entry.name}}</button></td><td>{{entry.directory?'目录':entry.name.split('.').pop()?.toUpperCase()}}</td><td>{{entry.directory?'—':bytes(entry.size)}}</td><td><div v-if="!entry.directory" class="button-row"><button v-if="/\.(yaml|yml)$/i.test(entry.name)" class="small-button" @click="dataset=entry.path;notice='已选择输入清单：'+entry.name">设为清单</button><button v-if="/\.(png|jpe?g)$/i.test(entry.name)" class="small-button" @click="image=entry.path;selectEntry(entry)">设为单图</button><button v-if="/\.xml$/i.test(entry.name)" class="small-button" @click="buildForm.yolov5_model=entry.path;activeTab='algorithms'">YOLOv5 模型</button><button v-if="/\.xml$/i.test(entry.name)" class="small-button" @click="buildForm.yolo11_model=entry.path;activeTab='algorithms'">YOLO11 模型</button></div></td></tr><tr v-if="!browser.entries.length"><td colspan="4" class="empty">{{browser.busy?'正在读取目录…':'此目录没有可见文件，或尚未注册输入目录。'}}</td></tr></tbody></table></div><div v-if="browser.preview" class="data-preview"><img :src="browser.preview" alt="所选原始图像"><code>{{browser.selected}}</code></div></div></div>
        </section>

        <section v-show="activeTab==='jobs'" class="stack large-gap">
          <div class="panel"><div class="panel-heading"><div><h2>创建实验任务</h2><p>非构建任务使用已成功构建与已保存参数快照。</p></div><span class="badge subtle">{{queueCount}} 个等待 / 运行</span></div><div class="panel-body"><div class="form-grid three"><label>任务类型<select v-model="taskForm.kind"><option v-for="(label,kind) in localJobKinds" :key="kind" :value="kind">{{label}}</option></select></label><label v-if="taskForm.kind !== 'build' && taskForm.kind !== 'benchmark'">成功构建<select v-model="taskBuild"><option value="">请选择已成功构建</option><option v-for="b in successBuilds" :key="b.id" :value="b.id">{{b.name}}</option></select></label><label v-if="!['build','ctest','benchmark'].includes(taskForm.kind)">保存的参数方案<select v-model="taskProfile"><option value="">请选择已保存方案</option><option v-for="p in profiles" :key="p.id" :value="p.id">{{p.name}}</option></select></label></div><div v-if="taskForm.kind==='build'" class="callout">编译开关与模型设置在“算法方案”中填写。<button class="text-button" @click="activeTab='algorithms'">打开构建表单 →</button></div><label v-if="['replay','benchmark'].includes(taskForm.kind)" class="block-label">数据清单<div class="input-action"><input v-model="dataset" placeholder="数据清单的绝对路径"><button @click="openPicker('dataset')">选择</button></div></label><div v-if="taskForm.kind==='synthetic'" class="form-grid"><label>合成帧数<input v-model.number="taskForm.frames" type="number" min="1" step="1"></label></div><div v-if="taskForm.kind==='single_image'" class="form-grid"><label>单图输入<div class="input-action"><input v-model="image" placeholder="图像绝对路径"><button @click="openPicker('image')">选择</button></div></label><label>迭代次数<input v-model.number="taskForm.iterations" type="number" min="1" step="1"></label></div><div v-if="taskForm.kind==='benchmark'" class="benchmark-form"><div class="subheading"><h3>比较项</h3><button class="small-button" @click="benchmarkRuns.push({build_id:taskBuild,profile_id:taskProfile})">＋ 添加方案</button></div><div v-for="(run,i) in benchmarkRuns" :key="i" class="run-row"><span class="run-number">{{i+1}}</span><select v-model="run.build_id" aria-label="比较项构建"><option value="">选择成功构建</option><option v-for="b in successBuilds" :key="b.id" :value="b.id">{{b.name}}</option></select><select v-model="run.profile_id" aria-label="比较项参数方案"><option value="">选择参数方案</option><option v-for="p in profiles" :key="p.id" :value="p.id">{{p.name}}</option></select><button class="icon-button" :disabled="benchmarkRuns.length===1" :aria-label="'移除比较项 '+(i+1)" @click="benchmarkRuns.splice(i,1)">×</button></div><div class="form-grid four"><label>IoU 阈值<input v-model.number="taskForm.iou" type="number" min="0" max="1" step="0.05"></label><label>真值位置不确定度上限（m，可选）<input v-model="taskForm.position_limit" type="number" min="0" step="any" placeholder="未提供"></label><label>真值旋转不确定度上限（rad，可选）<input v-model="taskForm.rotation_limit" type="number" min="0" step="any" placeholder="未提供"></label></div><label class="block-label">位姿参考系标识（可选）<input v-model="taskForm.pose_reference" placeholder="例如 test-reference 或 world，以数据标注中的实际 reference 为准"></label><p class="small muted">位姿真值已包含在所选数据清单的标注中。参考系标识、真值位置与旋转不确定度上限须一起填写；两项上限用于筛选合格真值。跨构建报告核对这些条件，缺少合格真值时指标显示“未产生”。</p></div><div v-if="!successBuilds.length && taskForm.kind!=='build'" class="inline-warning">尚无成功构建，请先到“算法方案”执行构建。</div><div class="form-footer"><span class="small muted">任务进入串行队列；取消会终止整组子进程。</span><button v-if="taskForm.kind!=='build'" class="primary" :disabled="working || (taskForm.kind !== 'benchmark' && (!taskBuild || (taskForm.kind!=='ctest' && !taskProfile)))" @click="submitTask()">提交{{jobKinds[taskForm.kind]}}</button></div></div></div>
          <div class="jobs-layout"><div class="panel"><div class="panel-heading"><h2>任务队列</h2><select v-model="jobFilter" aria-label="筛选任务状态"><option value="">全部状态</option><option v-for="(label,status) in statuses" :key="status" :value="status">{{label}}</option></select></div><div class="job-list"><button v-for="job in visibleJobs" :key="job.id" class="job-item" :class="{selected:selectedJobId===job.id}" @click="selectedJobId=job.id"><div><strong>{{jobKinds[job.kind] || job.kind}}</strong><span class="badge" :class="job.status">{{statuses[job.status] || job.status}}</span></div><code>{{job.id}}</code><small>{{timestamp(job.created_at)}}</small></button><div v-if="!visibleJobs.length" class="empty">还没有任务记录。</div></div></div><div class="panel job-detail"><div class="panel-heading"><div><h2>{{selectedJob?jobKinds[selectedJob.kind]:'任务详情'}}</h2><p class="mono">{{selectedJob?.id || '选择任务以查看日志与命令'}}</p></div><div v-if="selectedJob" class="button-row"><button v-if="isLive(selectedJob.status)" class="danger-button" :disabled="working" @click="cancelJob(selectedJob)">{{selectedJob.kind==='nuc_run'?'停止 NUC 运行':'取消任务'}}</button><button @click="loadResults(selectedJob.id)">查看产物</button></div></div><div v-if="selectedJob" class="panel-body"><div class="job-meta"><span class="badge" :class="selectedJob.status">{{statuses[selectedJob.status]}}</span><span>开始 {{timestamp(selectedJob.started_at)}}</span><span>完成 {{timestamp(selectedJob.finished_at)}}</span></div><div v-if="selectedJob.error" class="inline-warning">{{selectedJob.error}}</div><details class="job-options"><summary>任务选项与产物目录</summary><code>{{selectedJob.directory}}</code><pre>{{JSON.stringify(selectedJob.options,null,2)}}</pre></details><details v-if="selectedJob.commands?.length" class="job-options"><summary>实际执行命令（{{selectedJob.commands.length}}）</summary><pre>{{JSON.stringify(selectedJob.commands,null,2)}}</pre></details><div class="log-heading"><h3>运行日志</h3><span class="small muted">每 2 秒读取新增内容 · {{isLive(selectedJob.status)?'实时更新':'记录保留'}}</span></div><pre class="log-view" aria-live="polite">{{jobLog || '尚未写入日志。'}}</pre></div><div v-else class="empty">提交或选择任务后，可在这里检查真实执行输出。</div></div></div>
        </section>

        <section v-show="activeTab==='results'" class="stack large-gap">
          <div class="panel result-toolbar"><label>查看任务结果<select v-model="resultJobId" @change="loadResults()"><option value="">请选择任务</option><option v-for="job in jobs" :key="job.id" :value="job.id">{{jobKinds[job.kind]}} · {{statuses[job.status]}} · {{job.id}}</option></select></label><button :disabled="!resultJobId || resultLoading" @click="loadResults()">{{resultLoading?'正在读取…':'刷新产物'}}</button><span v-if="resultJob" class="badge" :class="resultJob.status">{{statuses[resultJob.status]}}</span><span class="small muted">报告 {{results.reports.length}} · 预览帧 {{results.frames.length}} · 产物 {{artifacts.length}}</span></div>
          <div class="callout">逻辑报告、耗时报告分别呈现。单图计时保留 P50/P95 原义；独立回放预览与批量比较分别保留来源。未生成的指标显示“未产生”。</div>
          <div v-if="resultJob?.kind === 'benchmark' && results.comparison" class="panel"><div class="panel-heading"><h2>本任务方案比较</h2><span class="badge" :class="results.comparison.compatible?'succeeded':'amber'">{{results.comparison.compatible?'评测条件兼容':'条件不兼容 / 未验证'}}</span></div><div class="panel-body"><div v-if="results.comparison.reasons?.length" class="inline-warning"><div v-for="reason in results.comparison.reasons" :key="reason">{{reason}}</div></div><div class="table-wrap comparison-matrix"><table><thead><tr><th>指标</th><th v-for="(row,i) in comparisonRows" :key="i">{{row.name || '未命名方案'}}<small>构建 {{row.build_id ? String(row.build_id).slice(0,8) : '未产生'}}</small></th></tr></thead><tbody><tr v-for="metric in comparisonMetrics" :key="metric.path"><td>{{metric.label}}<small class="mono">{{metric.path}}</small></td><td v-for="(row,i) in comparisonRows" :key="i" :class="{'muted':at(row,metric.path)==null}">{{printable(at(row,metric.path))}}</td></tr><tr v-if="!comparisonRows.length"><td class="empty">未产生比较报告</td></tr></tbody></table></div><details class="parameter-differences"><summary>有效参数差异 <span class="badge subtle">{{effectiveParameterDiffs.length}} 项</span></summary><div class="table-wrap comparison-matrix"><table><thead><tr><th>参数</th><th v-for="(row,i) in comparisonRows" :key="i">{{row.name || '未命名方案'}}<small>构建 {{row.build_id ? String(row.build_id).slice(0,8) : '未产生'}}</small></th></tr></thead><tbody><tr v-for="parameter in effectiveParameterDiffs" :key="parameter.key"><td class="mono">{{parameter.key}}</td><td v-for="(value,i) in parameter.values" :key="i">{{printable(value)}}</td></tr><tr v-if="!effectiveParameterDiffs.length"><td :colspan="comparisonRows.length+1" class="empty">{{comparisonRows.length?'各方案有效参数相同。':'未产生有效参数。'}}</td></tr></tbody></table></div></details></div></div>
          <div class="panel"><div class="panel-heading"><div><h2>实际报告</h2><p>完整展示工具产出的字段、状态与验收诊断。</p></div><input v-model="reportSearch" type="search" placeholder="筛选报告指标" aria-label="筛选报告指标"></div><div class="reports"><details v-for="(report,i) in results.reports" :key="report.name+'-'+i" class="report-card" open><summary><strong>{{report.name}}</strong><span class="badge subtle">{{({logic:'逻辑报告',timing:'耗时报告',configuration:'配置检查',text:'工具输出'} as Record<string,string>)[report.kind] || report.kind}}</span></summary><pre v-if="typeof report.data==='string'" class="report-text">{{report.data}}</pre><Metrics v-else :data="report.data" :search="reportSearch"/></details><div v-if="!results.reports.length" class="empty">未产生报告。选择已完成任务，或检查失败任务的日志。</div></div></div>
          <div class="panel"><div class="panel-heading"><div><h2>跨任务报告对照</h2><p>从历史任务读取实际报告，先核对评测条件，再解读各项指标。</p></div><button :disabled="working || compareIds.length<2" @click="compareReports">对照所选任务</button></div><div class="panel-body"><div class="compare-picks"><label v-for="job in resultJobs" :key="job.id" class="check"><input v-model="compareIds" type="checkbox" :value="job.id">{{jobKinds[job.kind]}} · {{job.id}} <span class="badge" :class="job.status">{{statuses[job.status]}}</span></label><span v-if="!resultJobs.length" class="muted">尚无历史实验任务。</span></div><div v-if="comparisons.length" class="comparison-output"><div v-if="!compareAudit.compatible" class="inline-warning">当前任务报告不可直接比较；请查看下方条件与缺失诊断。<div v-for="reason in compareAudit.reasons" :key="reason">{{reason}}</div></div><div v-for="condition in compareAudit.conditions" :key="condition.key" class="compat-row"><code>{{condition.key}}</code><span class="badge" :class="condition.compatible?'succeeded':'amber'">{{condition.compatible?'一致':'不一致 / 缺失'}}</span><span>{{condition.values.map(v=>v||'未产生').join(' / ')}}</span></div><label class="comparison-search">筛选对照字段<input v-model="compareSearch" type="search" placeholder="筛选对照字段，例如 precision 或 refinement" aria-label="筛选对照字段"></label><div class="table-wrap"><table><thead><tr><th>报告字段</th><th v-for="item in comparisons" :key="item.job.id">{{jobKinds[item.job.kind]}}<small>{{item.job.id}}</small></th></tr></thead><tbody><tr v-for="key in filteredCompareColumns" :key="key"><td class="mono">{{key}}</td><td v-for="item in comparisons" :key="item.job.id">{{comparedValue(item,key)}}</td></tr></tbody></table></div></div></div></div>
          <div class="panel"><div class="panel-heading"><div><h2>逐帧图像浏览</h2><p>{{currentFrame?.source==='independent_replay'?'来源：独立离线回放预览':'按实际生成的图像产物浏览'}} · 原图与叠加图</p></div><span class="badge subtle">{{results.frames.length}} 帧</span></div><div v-if="currentFrame" class="panel-body"><div class="frame-toolbar"><button :disabled="frameIndex===0" @click="stopPlayback();frameIndex--">← 上一帧</button><button @click="togglePlayback">{{playing?'暂停':'播放'}}</button><input v-model.number="frameIndex" type="range" min="0" :max="Math.max(0,results.frames.length-1)" aria-label="逐帧位置" @input="stopPlayback()"><button :disabled="frameIndex>=results.frames.length-1" @click="stopPlayback();frameIndex++">下一帧 →</button><span class="mono">{{frameIndex+1}} / {{results.frames.length}}</span></div><div class="image-grid"><figure><figcaption>原图 <a v-if="originalFrameUrl" :href="originalFrameUrl" target="_blank" rel="noreferrer">打开原图 ↗</a></figcaption><img v-if="originalFrameUrl" :src="originalFrameUrl" alt="当前帧原始图像"><div v-else class="image-empty">原图未产生或未关联到该预览帧。<br>可在数据管理浏览原始输入图像。</div></figure><figure><figcaption>叠加图 <a :href="currentFrame.url" target="_blank" rel="noreferrer">打开图像 ↗</a></figcaption><img :src="currentFrame.url" alt="当前帧回放叠加图"></figure></div><div class="frame-meta"><span>generation <code>{{currentFrame.generation}}</code></span><span>frame_id <code>{{currentFrame.frame_id}}</code></span><span>source <code>{{currentFrame.source || '产物记录'}}</code></span></div></div><div v-else class="empty">未产生逐帧预览。离线回放完成后查看独立 visualizer 输出。</div></div>
          <div class="panel"><div class="panel-heading"><div><h2>命令 TSV · 角度曲线</h2><p>保持真实命令列名与单位，可选绘制字段。</p></div><div class="input-action"><select v-model="curveArtifact" aria-label="命令 TSV 来源"><option value="">任务命令记录</option><option v-for="artifact in tsvArtifacts" :key="artifact.path" :value="artifact.path">{{artifact.name}}</option></select><button :disabled="working || !resultJobId" @click="loadCurve">读取 TSV</button></div></div><div class="panel-body"><LineChart :rows="curveRows"/></div></div>
          <div class="panel"><div class="panel-heading"><h2>任务产物</h2><input v-model="artifactSearch" type="search" placeholder="搜索文件或路径" aria-label="搜索任务产物"></div><div class="table-wrap"><table><thead><tr><th>文件</th><th>类型</th><th>相对路径</th><th>操作</th></tr></thead><tbody><tr v-for="artifact in filteredArtifacts" :key="artifact.path"><td>{{artifact.name}}</td><td><span class="badge subtle">{{artifact.kind}}</span></td><td class="mono path-cell">{{artifact.path}}</td><td><div class="button-row"><a class="button-link" :href="artifact.url" download>下载</a><a v-if="/\.(png|jpe?g|yaml|yml|txt|tsv|json)$/i.test(artifact.path)" class="button-link" :href="artifact.url" target="_blank" rel="noreferrer">打开 ↗</a><button v-if="/events\.ya?ml$/i.test(artifact.path) && resultJob?.directory" class="small-button" @click="dataset=resultJob.directory.replace(/[\\/]$/,'')+'/'+artifact.path;activeTab='jobs';taskForm.kind='replay'">用于回放</button></div></td></tr><tr v-if="!filteredArtifacts.length"><td colspan="4" class="empty">未产生文件产物。</td></tr></tbody></table></div></div>
        </section>
      </div>
      <footer class="app-footer"><span>XTYF AutoAim · 实验与设备工作台</span><span>合成数据验证流程 · 实际模型验收与目标设备测量独立保留</span></footer>
    </main>
    <div v-if="browser.open" class="modal-backdrop" @click.self="browser.open=false"><section class="file-modal" role="dialog" aria-modal="true" aria-labelledby="file-modal-title"><div class="panel-heading"><div><h2 id="file-modal-title">选择服务端文件</h2><p>选择 {{browser.target.startsWith('contract:')?'基础配置':browser.target==='dataset'?'数据清单':browser.target==='image'?'输入图像':'模型 XML'}}</p></div><button class="icon-button" aria-label="关闭文件选择" @click="browser.open=false">×</button></div><div class="panel-body"><div class="root-list"><button v-for="root in context.roots" :key="typeof root==='string'?root:root.path" class="path-pill" @click="browse(typeof root==='string'?root:root.path)">{{typeof root==='string'?root:root.path}}</button></div><div class="browser-toolbar"><button :disabled="!browser.parent || browser.busy" @click="browse(browser.parent || '')">↑</button><input v-model="browser.path" aria-label="目录" @keydown.enter="browse(browser.path)"><button :disabled="browser.busy" @click="browse(browser.path)">前往</button></div><div v-if="browser.error" class="inline-warning">{{browser.error}}</div><div class="file-picker-list"><button v-for="entry in browser.entries" :key="entry.path" :class="{selected:browser.selected===entry.path}" @click="selectEntry(entry)" @dblclick="!entry.directory && useSelected()"><span>{{entry.directory?'▰':'▱'}} {{entry.name}}</span><small>{{entry.directory?'目录':bytes(entry.size)}}</small></button><div v-if="!browser.entries.length" class="empty">{{browser.busy?'读取中…':'没有可见文件，请注册或切换输入目录。'}}</div></div><div v-if="browser.preview" class="picker-preview"><img :src="browser.preview" alt="选中图像预览"></div><code class="selected-path">{{browser.selected || '尚未选择文件'}}</code></div><div class="modal-footer"><button @click="browser.open=false">取消</button><button class="primary" :disabled="!browser.selected" @click="useSelected">使用所选文件</button></div></section></div>
  </div>
</template>






