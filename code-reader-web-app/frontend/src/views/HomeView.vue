﻿﻿﻿<template>
  <div class="home-view">
    <el-container>
      <el-header class="header">
        <div class="logo">
          <el-icon size="24"><Document /></el-icon>
          <span>Code Reader</span>
        </div>
      </el-header>
      
      <el-main class="main">
        <div class="upload-section">
          <h2>导入项目</h2>
          <el-row :gutter="20">
            <el-col :xs="24" :sm="12">
              <el-card class="upload-card">
                <template #header>
                  <div class="card-header">
                    <el-icon><Upload /></el-icon>
                    <span>上传 ZIP 文件</span>
                  </div>
                </template>
                <el-upload
                  ref="zipUploadRef"
                  :auto-upload="false"
                  :limit="1"
                  accept=".zip"
                  :on-change="handleZipChange"
                  :show-file-list="false"
                  drag
                >
                  <el-icon class="el-icon--upload"><UploadFilled /></el-icon>
                  <div class="el-upload__text">
                    拖拽 ZIP 文件到此处，或 <em>点击选择</em>
                  </div>
                </el-upload>
                <el-input
                  v-model="zipProjectName"
                  placeholder="项目名称（可选）"
                  style="margin-top: 12px"
                />
                <el-button
                  type="primary"
                  :loading="uploading"
                  :disabled="!zipFile"
                  @click="uploadZip"
                  style="margin-top: 12px; width: 100%"
                >
                  上传项目
                </el-button>
              </el-card>
            </el-col>
            
            <el-col :xs="24" :sm="12">
              <el-card class="upload-card">
                <template #header>
                  <div class="card-header">
                    <el-icon><FolderOpened /></el-icon>
                    <span>上传文件夹</span>
                  </div>
                </template>
                <div class="folder-upload">
                  <input
                    ref="folderInputRef"
                    type="file"
                    webkitdirectory
                    directory
                    multiple
                    style="display: none"
                    @change="handleFolderChange"
                  />
                  <el-button @click="() => folderInputRef?.click()">
                    选择文件夹
                  </el-button>
                  <div v-if="folderFiles.length > 0" class="selected-files">
                    已选择 {{ folderFiles.length }} 个文件
                  </div>
                </div>
                <el-input
                  v-model="folderProjectName"
                  placeholder="项目名称"
                  style="margin-top: 12px"
                />
                <el-button
                  type="primary"
                  :loading="uploading"
                  :disabled="!folderFiles.length || !folderProjectName"
                  @click="uploadFolder"
                  style="margin-top: 12px; width: 100%"
                >
                  上传项目
                </el-button>
              </el-card>
            </el-col>
          </el-row>
        </div>

        <div class="projects-section">
          <h2>已导入项目</h2>
          <el-table
            :data="projectStore.projects"
            v-loading="projectStore.loading"
            empty-text="暂无项目"
          >
            <el-table-column prop="name" label="项目名称" min-width="200" />
            <el-table-column prop="file_count" label="文件数" width="100" />
            <el-table-column prop="created_at" label="导入时间" width="180">
              <template #default="{ row }">
                {{ formatDate(row.created_at) }}
              </template>
            </el-table-column>
            <el-table-column label="操作" width="150" fixed="right">
              <template #default="{ row }">
                <el-button type="primary" link @click="openProject(row.id)">
                  打开
                </el-button>
                <el-popconfirm
                  title="确定要删除此项目吗？"
                  @confirm="deleteProject(row.id)"
                >
                  <template #reference>
                    <el-button type="danger" link>删除</el-button>
                  </template>
                </el-popconfirm>
              </template>
            </el-table-column>
          </el-table>
        </div>
      </el-main>
    </el-container>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted } from 'vue'
import { useRouter } from 'vue-router'
import { ElMessage } from 'element-plus'
import { useProjectStore } from '@/stores'

const router = useRouter()
const projectStore = useProjectStore()

const zipFile = ref<File | null>(null)
const zipProjectName = ref('')
const folderFiles = ref<File[]>([])
const folderProjectName = ref('')
const uploading = ref(false)
const folderInputRef = ref<HTMLInputElement | null>(null)

onMounted(() => {
  projectStore.loadProjects()
})

const handleZipChange = (file: any) => {
  zipFile.value = file.raw
}

const handleFolderChange = (event: Event) => {
  const input = event.target as HTMLInputElement
  if (input.files) {
    folderFiles.value = Array.from(input.files)
  }
}

const uploadZip = async () => {
  if (!zipFile.value) return
  
  uploading.value = true
  try {
    const project = await projectStore.uploadZip(zipFile.value, zipProjectName.value || undefined)
    ElMessage.success('项目上传成功')
    zipFile.value = null
    zipProjectName.value = ''
    router.push(`/project/${project.id}`)
  } catch (error: any) {
    ElMessage.error(error.response?.data?.detail || '上传失败')
  } finally {
    uploading.value = false
  }
}

const uploadFolder = async () => {
  if (!folderFiles.value.length || !folderProjectName.value) return
  
  uploading.value = true
  try {
    const dataTransfer = new DataTransfer()
    folderFiles.value.forEach((file) => dataTransfer.items.add(file))
    const project = await projectStore.uploadFolder(dataTransfer.files, folderProjectName.value)
    ElMessage.success('项目上传成功')
    folderFiles.value = []
    folderProjectName.value = ''
    router.push(`/project/${project.id}`)
  } catch (error: any) {
    ElMessage.error(error.response?.data?.detail || '上传失败')
  } finally {
    uploading.value = false
  }
}

const openProject = (id: string) => {
  router.push(`/project/${id}`)
}

const deleteProject = async (id: string) => {
  try {
    await projectStore.deleteProject(id)
    ElMessage.success('项目已删除')
  } catch (error: any) {
    ElMessage.error(error.response?.data?.detail || '删除失败')
  }
}

const formatDate = (dateStr: string) => {
  return new Date(dateStr).toLocaleString('zh-CN')
}
</script>

<style lang="scss" scoped>
.home-view {
  height: 100vh;
  background: var(--el-bg-color-page);
}

.header {
  display: flex;
  align-items: center;
  background: var(--el-bg-color);
  border-bottom: 1px solid var(--el-border-color);
  box-shadow: 0 1px 4px rgba(0, 0, 0, 0.1);
}

.logo {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 20px;
  font-weight: 600;
  color: var(--el-text-color-primary);
}

.main {
  max-width: 1200px;
  margin: 0 auto;
  padding: 24px;
}

.upload-section {
  margin-bottom: 32px;
  
  h2 {
    margin-bottom: 16px;
    color: var(--el-text-color-primary);
  }
}

.upload-card {
  height: 100%;
  
  .card-header {
    display: flex;
    align-items: center;
    gap: 8px;
  }
}

.folder-upload {
  text-align: center;
  padding: 20px;
  
  .selected-files {
    margin-top: 12px;
    color: var(--el-text-color-secondary);
  }
}

.projects-section {
  h2 {
    margin-bottom: 16px;
    color: var(--el-text-color-primary);
  }
}

@media (max-width: 768px) {
  .main {
    padding: 16px;
  }
  
  .upload-card {
    margin-bottom: 16px;
  }
}
</style>
