"use client";

import { useEffect, useRef } from "react";
import cytoscape from "cytoscape";
import type { MCKnowledgeNode } from "@/lib/types";

interface CyElement {
  data: Record<string, unknown>;
  style?: Record<string, unknown>;
  classes?: string;
}

interface CytoscapeGraphProps {
  elements: { nodes: CyElement[]; edges: CyElement[] };
  knowledgeNodes: MCKnowledgeNode[];
  onSelectNode: (node: MCKnowledgeNode | null) => void;
}

export default function CytoscapeGraph({
  elements,
  knowledgeNodes,
  onSelectNode,
}: CytoscapeGraphProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const cyRef = useRef<cytoscape.Core | null>(null);
  const mountedRef = useRef(true);

  useEffect(() => {
    mountedRef.current = true;
    return () => {
      mountedRef.current = false;
    };
  }, []);

  useEffect(() => {
    if (!containerRef.current) return;
    const container = containerRef.current;

    if (cyRef.current) {
      cyRef.current.destroy();
      cyRef.current = null;
    }

    const cy = cytoscape({
      container,
      elements: [...elements.nodes, ...elements.edges] as cytoscape.ElementDefinition[],
      style: [
        {
          selector: "node",
          style: {
            "background-opacity": 0.9,
            label: "data(label)",
            "font-size": "10px",
            "text-valign": "center",
            "text-halign": "center",
            color: "#1a1a2e",
            "text-outline-width": 2,
            "text-outline-color": "#ffffff",
          },
        },
        {
          selector: "edge",
          style: {
            width: 1.5,
            "curve-style": "bezier",
            opacity: 0.6,
          },
        },
        {
          selector: ".prerequisite",
          style: {
            "line-color": "hsl(220, 15%, 40%)",
            "line-style": "solid",
          },
        },
        {
          selector: ".related",
          style: {
            "line-color": "hsl(220, 15%, 25%)",
            "line-style": "dashed",
            "line-dash-pattern": [6, 3],
          },
        },
        {
          selector: ".cross-reference",
          style: {
            "line-color": "hsl(35, 60%, 50%)",
            "line-style": "solid",
            "target-arrow-color": "hsl(35, 60%, 50%)",
            width: 2,
          },
        },
      ],
      layout: {
        name: "cose",
        animate: true,
        animationDuration: 800,
        nodeRepulsion: () => 6000,
        idealEdgeLength: () => 120,
        gravity: 0.5,
      },
      userZoomingEnabled: true,
      userPanningEnabled: true,
      boxSelectionEnabled: false,
      minZoom: 0.3,
      maxZoom: 3,
    });

    cyRef.current = cy;

    cy.on("tap", "node", (evt) => {
      if (!mountedRef.current) return;
      const nodeId = evt.target.id();
      const found = knowledgeNodes.find((n) => n.id === nodeId);
      if (found) onSelectNode(found);
    });

    cy.on("tap", (evt) => {
      if (!mountedRef.current) return;
      if (evt.target === cy) onSelectNode(null);
    });

    return () => {
      mountedRef.current = false;
      if (cyRef.current) {
        const instance = cyRef.current;
        instance.removeAllListeners();
        instance.destroy();
        cyRef.current = null;
      }
    };
  }, [elements, knowledgeNodes, onSelectNode]);

  return <div ref={containerRef} className="flex-1 w-full h-full" />;
}
