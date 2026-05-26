from fastapi import APIRouter, HTTPException
from pathlib import Path
import re
from typing import List, Dict, Optional

router = APIRouter()

BASE_DIR = Path(__file__).resolve().parent.parent.parent.parent.parent
DOCS_DIR = BASE_DIR / "motor-control-knowledge-base"

_QUESTION_HEADING_RE = re.compile(r'^#{2,3}\s+.*(第\s*\d+\s*题|题目\s*\d)')
_PROBLEM_RE = re.compile(r'^\*\*(问题|题目)[:：]\*\*\s*')
_ANSWER_RE = re.compile(r'^\*\*(参考答案|正确答案|答案|解答)[:：]\*\*\s*')
_SECTION_RE = re.compile(r'^##\s+[^#]')
_META_RE = re.compile(r'^\*\*(模块|题目数量|难度分布|模块编号|题型)[:：]\*\*\s*')


def _is_question_heading(stripped: str) -> bool:
    return bool(_QUESTION_HEADING_RE.match(stripped))


def _is_section_heading(stripped: str) -> bool:
    return bool(_SECTION_RE.match(stripped)) and not _is_question_heading(stripped)


def _is_problem_line(stripped: str) -> bool:
    return bool(_PROBLEM_RE.match(stripped))


def _is_answer_line(stripped: str) -> bool:
    return bool(_ANSWER_RE.match(stripped))


def _is_meta_line(stripped: str) -> bool:
    return bool(_META_RE.match(stripped))


def _extract_bold_value(stripped: str) -> str:
    parts = stripped.split("**", 2)
    return parts[-1].strip() if len(parts) >= 3 else ""


def parse_assessment(file_path: Path) -> Dict:
    content = file_path.read_text(encoding="utf-8")
    lines = content.split("\n")

    title = ""
    module_name = ""
    total_questions = 0
    difficulty_info = ""
    question_type = ""
    sections: List[Dict] = []
    current_section: Optional[Dict] = None
    current_question: Optional[Dict] = None
    collecting_problem = False
    collecting_answer = False

    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        if stripped.startswith("# ") and not title:
            title = stripped.lstrip("# ").strip()
            collecting_problem = False
            collecting_answer = False

        elif _is_meta_line(stripped):
            if stripped.startswith("**模块") or stripped.startswith("**模块编号"):
                module_name = _extract_bold_value(stripped)
            elif stripped.startswith("**题目数量"):
                m = re.search(r'(\d+)', stripped)
                if m:
                    total_questions = int(m.group(1))
            elif stripped.startswith("**难度分布"):
                difficulty_info = _extract_bold_value(stripped)
            elif stripped.startswith("**题型"):
                question_type = _extract_bold_value(stripped)
            collecting_problem = False
            collecting_answer = False

        elif _is_section_heading(stripped):
            collecting_problem = False
            collecting_answer = False
            if current_section:
                if current_question:
                    current_section["questions"].append(current_question)
                    current_question = None
                sections.append(current_section)
            section_title = stripped.lstrip("# ").strip()
            difficulty_match = re.search(r'(★+)', section_title)
            difficulty = difficulty_match.group(1) if difficulty_match else ""
            clean_title = re.sub(r'\s*★+\s*', '', section_title).strip()
            current_section = {
                "title": clean_title,
                "difficulty": difficulty,
                "questions": [],
            }

        elif _is_question_heading(stripped):
            collecting_problem = False
            collecting_answer = False
            if current_question and current_section:
                current_section["questions"].append(current_question)
            elif current_question and not current_section:
                current_section = {"title": "", "difficulty": "", "questions": []}
                current_section["questions"].append(current_question)
                sections.append(current_section)

            q_title = stripped.lstrip("# ").strip()
            current_question = {
                "id": f"q-{len(sections)}-{len(current_section['questions']) if current_section else 0}",
                "title": q_title,
                "problem": "",
                "referenceAnswer": "",
                "difficulty": current_section.get("difficulty", "") if current_section else "",
            }

        elif current_question is not None:
            if _is_problem_line(stripped):
                collecting_problem = True
                collecting_answer = False
                first_line = _PROBLEM_RE.sub("", stripped).strip()
                current_question["problem"] = first_line

            elif _is_answer_line(stripped):
                collecting_problem = False
                collecting_answer = True
                first_line = _ANSWER_RE.sub("", stripped).strip()
                current_question["referenceAnswer"] = first_line

            elif collecting_problem:
                if _is_answer_line(stripped) or _is_question_heading(stripped):
                    collecting_problem = False
                    i -= 1
                elif stripped:
                    current_question["problem"] += "\n" + line.rstrip()

            elif collecting_answer:
                if _is_question_heading(stripped) or stripped.startswith("---"):
                    collecting_answer = False
                    i -= 1
                elif stripped:
                    current_question["referenceAnswer"] += "\n" + line.rstrip()

        i += 1

    if current_question and current_section:
        current_section["questions"].append(current_question)
    elif current_question and not current_section:
        current_section = {"title": "", "difficulty": "", "questions": [current_question]}
        sections.append(current_section)
    if current_section and current_section not in sections:
        sections.append(current_section)

    all_questions = []
    for section in sections:
        all_questions.extend(section["questions"])

    return {
        "title": title,
        "module": module_name,
        "totalQuestions": total_questions,
        "difficultyInfo": difficulty_info,
        "questionType": question_type,
        "sections": sections,
        "questions": all_questions,
    }


@router.get("/parse/{file_path:path}")
async def parse_assessment_api(file_path: str):
    full_path = DOCS_DIR / file_path
    if not full_path.exists():
        raise HTTPException(status_code=404, detail="Assessment not found")
    resolved = full_path.resolve()
    base_resolved = DOCS_DIR.resolve()
    if not str(resolved).startswith(str(base_resolved)):
        raise HTTPException(status_code=403, detail="Access denied")
    return parse_assessment(full_path)
