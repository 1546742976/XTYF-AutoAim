export type AnyRecord = Record<string, any>
export interface Field { path: string; type: 'string' | 'number' | 'integer' | 'boolean' | 'array'; group: string; description: string }
export interface Contract { key: string; path: string; content?: string }
export interface Profile { id: string; name: string; values: AnyRecord; contracts: Contract[]; config_path: string; created_at: string }
export interface ModelAcceptance { status: string; required?: boolean; provided?: string[]; missing?: string[]; required_tests?: string[] }
export interface Build { id: string; name: string; flags: Record<string, boolean>; openvino: boolean; build_type: string; status: string; build_dir: string; information: any; models: any; model_acceptance?: ModelAcceptance | string }
export interface Job { id: string; kind: string; status: string; options: AnyRecord; directory: string; commands?: any[]; created_at?: string; started_at?: string; finished_at?: string; error?: string }
export interface Artifact { path: string; name: string; kind: string; url: string }
export interface Frame { generation: string | number; frame_id: string | number; url: string; original_url?: string; source?: string }
export interface Results { reports: {name: string; kind: string; data: any}[]; frames: Frame[]; commands: AnyRecord[]; comparison?: {compatible: boolean; reasons: string[]; rows: AnyRecord[]} }
export interface Entry { name: string; path: string; directory: boolean; size: number; url?: string }
export interface FileList { path: string; parent: string | null; entries: Entry[] }
export interface NucTarget {
  host: string; user: string; port: number; identity_file: string; project_dir: string;
  build_dir: string; executable: string; device_config: string; workspace_dir: string;
  yolov5_model: string; yolo11_model: string;
}
export interface NucConfiguration { target: Partial<NucTarget> | null; configured: boolean; ssh_available: boolean; live_entry_note?: string }
export interface NucRunOptions { profile_id: string; flags: Record<string, boolean>; openvino: boolean; build_type: string }
export async function api<T>(path: string, body?: any): Promise<T> {
  const response = await fetch(path, { method: body === undefined ? 'GET' : 'POST', headers: body === undefined ? {} : {'Content-Type': 'application/json'}, body: body === undefined ? undefined : JSON.stringify(body) })
  const text = await response.text()
  let data: any
  try { data = text ? JSON.parse(text) : {} } catch { data = {detail: text} }
  if (!response.ok) {
    const detail = data.detail ?? data.error ?? data
    throw new Error(typeof detail === 'string' ? detail : JSON.stringify(detail, null, 2))
  }
  return data as T
}
export const copy = <T>(value: T): T => JSON.parse(JSON.stringify(value))
export function at(object: AnyRecord, path: string) { return path.split('.').reduce<any>((value, key) => value?.[key], object) }
export function put(object: AnyRecord, path: string, value: any) {
  const keys = path.split('.'); const last = keys.pop()!
  let target = object
  keys.forEach(key => { target[key] ??= {}; target = target[key] })
  target[last] = value
}
export function printable(value: any): string {
  if (value === undefined || value === null || value === 'not_produced') return '未产生'
  return typeof value === 'object' ? JSON.stringify(value) : String(value)
}
export function flatten(object: any, prefix = ''): {key: string; value: any}[] {
  if (Array.isArray(object)) {
    if (object.length && object.every(item => item !== null && typeof item === 'object' && !Array.isArray(item))) {
      return object.flatMap((item, index) => flatten(item, prefix ? `${prefix}.${index}` : String(index)))
    }
    return [{key: prefix || '值', value: object}]
  }
  if (object === null || typeof object !== 'object') return [{key: prefix || '值', value: object}]
  return Object.entries(object).flatMap(([key, value]) => flatten(value, prefix ? `${prefix}.${key}` : key))
}
