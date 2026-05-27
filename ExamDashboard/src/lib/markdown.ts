import MarkdownIt from 'markdown-it'
import type { Options as MarkdownItOptions } from 'markdown-it'
import hljs from 'highlight.js'
import katex from 'katex'

const mdOptions: MarkdownItOptions = {
  html: true,
  linkify: true,
  typographer: true,
  highlight: (str: string, lang: string): string => {
    if (lang && hljs.getLanguage(lang)) {
      try {
        return `<pre class="hljs"><code>${hljs.highlight(str, { language: lang }).value}</code></pre>`
      } catch {
        console.error('Highlight error')
      }
    }
    return `<pre class="hljs"><code>${MarkdownIt().utils.escapeHtml(str)}</code></pre>`
  }
}

const md = new MarkdownIt(mdOptions)

const defaultFenceRenderer = md.renderer.rules.fence ||
  function (tokens, idx, options, _env, self) {
    return self.renderToken(tokens, idx, options)
  }

md.renderer.rules.fence = (tokens, idx, options, env, self) => {
  const token = tokens[idx]
  const info = token.info ? token.info.trim() : ''
  if (info === 'mermaid') {
    const code = token.content.trim()
    const escapedCode = code
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
    return `<div class="mermaid-src">${escapedCode}</div>`
  }
  return defaultFenceRenderer(tokens, idx, options, env, self)
}

const renderKatex = (tex: string, displayMode: boolean): string => {
  try {
    return katex.renderToString(tex.trim(), {
      displayMode,
      throwOnError: false,
      output: 'html',
      strict: false,
      trust: true
    })
  } catch (error) {
    console.error('KaTeX error:', error, tex)
    return `<span class="katex-error" style="color:red;">${tex}</span>`
  }
}

const KATEX_BLOCK_PLACEHOLDER_PREFIX = '<!--KATEX-BLOCK-'
const KATEX_INLINE_PLACEHOLDER_PREFIX = '<!--KATEX-INLINE-'
const PLACEHOLDER_SUFFIX = '-->'

const extractLatex = (content: string): { cleanContent: string; blocks: Array<{ tex: string; displayMode: boolean; placeholder: string }> } => {
  const blocks: Array<{ tex: string; displayMode: boolean; placeholder: string }> = []
  let blockIndex = 0

  const result = content.replace(/\$\$\s*([\s\S]+?)\s*\$\$/g, (_, tex) => {
    const placeholder = `${KATEX_BLOCK_PLACEHOLDER_PREFIX}${blockIndex++}${PLACEHOLDER_SUFFIX}`
    blocks.push({ tex, displayMode: true, placeholder })
    return placeholder
  })

  let inlineIndex = 0
  const finalResult = result.replace(/\$\s*([^\$\n]+?)\s*\$/g, (_, tex) => {
    const placeholder = `${KATEX_INLINE_PLACEHOLDER_PREFIX}${inlineIndex++}${PLACEHOLDER_SUFFIX}`
    blocks.push({ tex, displayMode: false, placeholder })
    return placeholder
  })

  return { cleanContent: finalResult, blocks }
}

const restoreLatex = (html: string, blocks: Array<{ tex: string; displayMode: boolean; placeholder: string }>): string => {
  let result = html
  blocks.forEach((block) => {
    const rendered = block.displayMode
      ? `<div class="katex-block">${renderKatex(block.tex, true)}</div>`
      : renderKatex(block.tex, false)
    result = result.replace(block.placeholder, rendered)
  })
  return result
}

const escapeMathUnderscores = (content: string): string => {
  return content.replace(/(\w)_(\w)/g, '$1\\_$2')
}

export const renderMarkdown = (content: string): string => {
  const { cleanContent, blocks } = extractLatex(content)
  const protectedContent = escapeMathUnderscores(cleanContent)
  const renderedMd = md.render(protectedContent)
  const withIds = renderedMd.replace(/<h([1-6])>(.*?)<\/h[1-6]>/g, (_match: string, level: string, text: string) => {
    const plainText = text.replace(/<[^>]*>/g, '')
    const id = plainText.toLowerCase().replace(/\s+/g, '-').replace(/[^\w\u4e00-\u9fa5-]/g, '')
    return `<h${level} id="${id}">${text}</h${level}>`
  })
  return restoreLatex(withIds, blocks)
}
