import MarkdownIt from 'markdown-it'
import hljs from 'highlight.js'
import katex from 'katex'

const md = new MarkdownIt({
  html: true,
  linkify: true,
  typographer: true,
  highlight: (str: string, lang: string) => {
    if (lang && hljs.getLanguage(lang)) {
      try {
        return `<pre class="hljs"><code>${hljs.highlight(str, { language: lang }).value}</code></pre>`
      } catch (error) {
        console.error('Highlight error:', error)
      }
    }
    return `<pre class="hljs"><code>${md.utils.escapeHtml(str)}</code></pre>`
  }
})

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

const KATEX_BLOCK_PLACEHOLDER_PREFIX = '<!--KATEX_BLOCK_'
const KATEX_INLINE_PLACEHOLDER_PREFIX = '<!--KATEX_INLINE_'
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

export const renderMarkdown = (content: string): string => {
  const { cleanContent, blocks } = extractLatex(content)
  const renderedMd = md.render(cleanContent)
  const withIds = renderedMd.replace(/<h([1-6])>(.*?)<\/h[1-6]>/g, (_match, level, text) => {
    const plainText = text.replace(/<[^>]*>/g, '')
    let id = plainText.toLowerCase().replace(/\s+/g, '-').replace(/[^\w\u4e00-\u9fa5-]/g, '')
    if (!id) id = 'heading-' + Math.random().toString(36).slice(2, 8)
    return `<h${level} id="${id}">${text}</h${level}>`
  })
  return restoreLatex(withIds, blocks)
}

export const extractToc = (content: string): Array<{ level: number; text: string; id: string }> => {
  const headings: Array<{ level: number; text: string; id: string }> = []
  const regex = /^(#{1,6})\s+(.+)$/gm
  let match

  while ((match = regex.exec(content)) !== null) {
    const level = match[1].length
    const text = match[2].replace(/\*\*/g, '').replace(/`/g, '')
    const id = text.toLowerCase().replace(/\s+/g, '-').replace(/[^\w\u4e00-\u9fa5-]/g, '')
    headings.push({ level, text, id })
  }

  return headings
}
