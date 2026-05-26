import { NextResponse } from "next/server";
import fs from "fs";
import path from "path";

const KB_DIR = path.resolve(
  process.cwd(),
  "..",
  "motor-control-knowledge-base"
);

interface KnowledgeModule {
  id: string;
  title: string;
  path: string;
}

interface KnowledgeSection {
  id: string;
  title: string;
  description: string;
  modules: KnowledgeModule[];
}

function scanModuleFiles(dirName: string, prefix: string): KnowledgeModule[] {
  const fullDir = path.join(KB_DIR, dirName);
  if (!fs.existsSync(fullDir)) return [];

  const prefixPattern = new RegExp(`^${prefix}-\\d+-`);

  return fs
    .readdirSync(fullDir, { withFileTypes: true })
    .filter(
      (d) =>
        d.isFile() &&
        d.name.endsWith(".md") &&
        !d.name.includes("assessment") &&
        prefixPattern.test(d.name)
    )
    .map((d) => d.name)
    .sort()
    .map((filename) => {
      const id = filename.replace(".md", "");
      const match = id.match(new RegExp(`^${prefix}-(\\d+)-(.*)$`));
      const title = match
        ? `${prefix}-${match[1]} ${match[2].replace(/-/g, " ")}`
        : id;
      return { id, title, path: `${dirName}/${filename}` };
    });
}

export async function GET() {
  const hwModules = scanModuleFiles("hardware", "HW");
  const algModules = scanModuleFiles("algorithm", "ALG");

  const sections: KnowledgeSection[] = [
    {
      id: "hardware",
      title: "硬件路径",
      description: "从硬件底层理解电控系统，建立「硬件决定算法边界」的认知",
      modules: hwModules,
    },
    {
      id: "algorithm",
      title: "算法路径",
      description: "从控制算法理解电控系统，建立「算法受硬件约束」的认知",
      modules: algModules,
    },
  ];

  if (fs.existsSync(path.join(KB_DIR, "algorithm", "MC-LIB"))) {
    const mclibFiles = fs
      .readdirSync(path.join(KB_DIR, "algorithm", "MC-LIB"))
      .filter((f) => f.endsWith(".md") && !f.includes("assessment"))
      .sort();

    if (mclibFiles.length > 0) {
      sections.push({
        id: "mclib",
        title: "MC_LIB代码实践",
        description: "算法路径配套MC_LIB电机控制库的代码实践文档",
        modules: mclibFiles.map((f) => ({
          id: f.replace(".md", ""),
          title: f.replace(".md", ""),
          path: `algorithm/MC-LIB/${f}`,
        })),
      });
    }
  }

  const advancedDir = path.join(KB_DIR, "advanced");
  if (fs.existsSync(advancedDir)) {
    const advSubs = fs
      .readdirSync(advancedDir, { withFileTypes: true })
      .filter((d) => d.isDirectory());
    for (const sub of advSubs) {
      const files = fs
        .readdirSync(path.join(advancedDir, sub.name))
        .filter((f) => f.endsWith(".md") && !f.includes("assessment"))
        .sort();

      if (files.length > 0) {
        sections.push({
          id: `advanced/${sub.name}`,
          title: `高级 - ${sub.name}`,
          description: "",
          modules: files.map((f) => ({
            id: f.replace(".md", ""),
            title: f.replace(".md", ""),
            path: `advanced/${sub.name}/${f}`,
          })),
        });
      }
    }
  }

  const ctModules = scanModuleFiles("control-theory", "CT");
  const eeModules = scanModuleFiles("electronics-basics", "EE");
  const ppModules = scanModuleFiles("power-path", "PP");

  sections.push({
    id: "reference-appendix",
    title: "参考附录",
    description: "控制理论、电子基础、电源路径等交叉参考知识",
    modules: [...ctModules, ...eeModules, ...ppModules],
  });

  return NextResponse.json({ sections });
}