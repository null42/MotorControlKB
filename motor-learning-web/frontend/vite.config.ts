import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'
import { resolve } from 'path'
import { existsSync, readdirSync, unlinkSync } from 'fs'
import { join } from 'path'

const BACKEND_STATIC = resolve(__dirname, '../backend/static')

function cleanOldAssets() {
  const assetsDir = join(BACKEND_STATIC, 'assets')
  if (existsSync(assetsDir)) {
    readdirSync(assetsDir).forEach(file => {
      if (file.endsWith('.js') || file.endsWith('.css')) {
        unlinkSync(join(assetsDir, file))
      }
    })
  }
}

export default defineConfig({
  plugins: [
    vue(),
    {
      name: 'clean-backend-assets',
      buildStart() { cleanOldAssets() },
    },
  ],
  resolve: {
    alias: {
      '@': resolve(__dirname, 'src')
    }
  },
  server: {
    port: 3000,
    proxy: {
      '/api': {
        target: 'http://localhost:8001',
        changeOrigin: true
      },
      '/sims': {
        target: 'http://localhost:8001',
        changeOrigin: true
      }
    }
  },
  build: {
    outDir: BACKEND_STATIC,
    emptyOutDir: false,
    assetsInclude: ['**/*.woff', '**/*.woff2', '**/*.ttf']
  }
})