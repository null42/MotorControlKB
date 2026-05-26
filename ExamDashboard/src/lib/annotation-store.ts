import { useCallback, useEffect, useState } from "react"

export interface SectionAnnotation {
  sectionId: string
  sectionTitle: string
  content: string
  updatedAt: string
}

export function useAnnotations(docPath: string) {
  const [annotations, setAnnotations] = useState<Record<string, SectionAnnotation>>({})
  const [loading, setLoading] = useState(false)

  const fetchAnnotations = useCallback(async () => {
    if (!docPath) return
    setLoading(true)
    try {
      const res = await fetch(`/api/annotations?docPath=${encodeURIComponent(docPath)}`)
      if (!res.ok) return
      const response = await res.json()
      const data: SectionAnnotation[] = response.annotations || []
      const map: Record<string, SectionAnnotation> = {}
      data.forEach((a) => {
        map[a.sectionId] = a
      })
      setAnnotations(map)
    } catch (e) {
      console.error(e)
    } finally {
      setLoading(false)
    }
  }, [docPath])

  useEffect(() => {
    fetchAnnotations()
  }, [fetchAnnotations])

  const hasAnnotation = useCallback(
    (sectionId: string): boolean => {
      const a = annotations[sectionId]
      return !!a && a.content.length > 0
    },
    [annotations]
  )

  const saveAnnotation = useCallback(
    async (sectionId: string, sectionTitle: string, content: string) => {
      if (!docPath) return
      try {
        const res = await fetch("/api/annotations", {
          method: "PUT",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ docPath, sectionId, sectionTitle, content }),
        })
        if (!res.ok) return
        const entry: SectionAnnotation = {
          sectionId,
          sectionTitle,
          content,
          updatedAt: new Date().toISOString(),
        }
        setAnnotations((prev) => ({ ...prev, [entry.sectionId]: entry }))
      } catch (e) {
        console.error(e)
      }
    },
    [docPath]
  )

  const deleteAnnotation = useCallback(
    async (sectionId: string) => {
      if (!docPath) return
      try {
        const res = await fetch("/api/annotations", {
          method: "DELETE",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ docPath, sectionId }),
        })
        if (!res.ok) return
        setAnnotations((prev) => {
          const next = { ...prev }
          delete next[sectionId]
          return next
        })
      } catch (e) {
        console.error(e)
      }
    },
    [docPath]
  )

  return { annotations, hasAnnotation, saveAnnotation, deleteAnnotation, loading }
}