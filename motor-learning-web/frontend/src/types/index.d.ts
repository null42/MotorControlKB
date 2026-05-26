declare module 'markdown-it' {
  interface MarkdownIt {
    render(content: string): string
    utils: {
      escapeHtml(str: string): string
    }
  }

  interface Options {
    html?: boolean
    linkify?: boolean
    typographer?: boolean
    highlight?: (str: string, lang: string) => string
  }

  function MarkdownIt(options?: Options): MarkdownIt
  export default MarkdownIt
}

declare module 'katex' {
  interface KatexRenderOptions {
    displayMode?: boolean
    throwOnError?: boolean
    output?: 'html' | 'mathml' | 'htmlAndMathml'
  }

  export function renderToString(tex: string, options?: KatexRenderOptions): string
}

declare module 'highlight.js' {
  interface HighlightResult {
    value: string
    language?: string
  }

  export function highlight(code: string, options: { language: string }): HighlightResult
  export function getLanguage(lang: string): boolean | undefined
}
