"""Build an editable Chinese Word report from the reviewed local evidence."""

import hashlib
import json
from functools import lru_cache
from pathlib import Path
from zipfile import ZipFile

from docx import Document
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_TAB_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor
from PIL import Image, ImageDraw, ImageFont


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
STEM = "G1机器人复杂操作任务技术成果报告_20261013"
FONT_FILE = "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf"
LATIN_FONT_FILE = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
INK = "253444"
ACCENT = "185D75"
MUTED = "60717D"
LIGHT = "EAF0F3"
WHITE = "FFFFFF"
BODY_FONT = "宋体"
HEADING_FONT = "黑体"


@lru_cache(maxsize=None)
def font(size, latin=False):
    return ImageFont.truetype(LATIN_FONT_FILE if latin else FONT_FILE, size)


def text_runs(text):
    groups = []
    for character in text:
        latin = ord(character) < 0x2E80
        if groups and groups[-1][0] == latin:
            groups[-1] = (latin, groups[-1][1] + character)
        else:
            groups.append((latin, character))
    return groups


def draw_text(draw, xy, text, size, fill):
    x, y = xy
    for latin, group in text_runs(text):
        selected = font(size, latin)
        draw.text((x, y), group, font=selected, fill=fill, anchor="lt")
        x += draw.textlength(group, font=selected)


def centered(draw, text, x, y, size, fill):
    width = sum(draw.textlength(group, font=font(size, latin)) for latin, group in text_runs(text))
    draw_text(draw, (x - width / 2, y - size / 2), text, size, fill)


def make_architecture():
    image = Image.new("RGB", (1660, 890), "white")
    draw = ImageDraw.Draw(image)
    rows = [
        ("感知输入", "RGB-D目标识别 / 关节与IMU状态 / 实测示教数据", "分层识别、彩深关联与原始数据保留"),
        ("统一建模", "机器人本体 / 工具坐标 / 作业几何 / 负载模型", "空间变换、标定诊断与模型一致性"),
        ("规划优化", "有界逆解 / 多候选搜索 / 连续轨迹 / 局部时间优化", "路径、姿态、连续性与运动限值联合校核"),
        ("受控执行", "控制网关 / 重力补偿 / 手臂手型编排 / 异常恢复", "实际起点、反馈时效与控制权管理"),
        ("任务管理", "业务接口 / 动作资产 / 冻结发布 / 验证归档", "任务状态闭环与技术版本追溯"),
    ]
    for index, (label, main, detail) in enumerate(rows):
        y = 25 + index * 166
        draw.rectangle((20, y, 1500, y + 132), fill="#F2F5F7")
        draw.rectangle((20, y, 250, y + 132), fill="#185D75")
        centered(draw, label, 135, y + 66, 39, "white")
        draw_text(draw, (285, y + 22), main, 34, "#253444")
        draw_text(draw, (285, y + 77), detail, 27, "#60717D")
        if index < len(rows) - 1:
            draw.line((780, y + 136, 780, y + 157), fill="#60717D", width=3)
            draw.polygon(((771, y + 151), (789, y + 151), (780, y + 160)), fill="#60717D")
    draw.line((1570, 40, 1570, 817), fill="#A4B0B8", width=3)
    for index, character in enumerate("全过程质量校核"):
        centered(draw, character, 1610, 220 + index * 53, 34, "#185D75")
    image.save(HERE / "assets/architecture.png", dpi=(240, 240))


def make_optimization():
    image = Image.new("RGB", (1660, 635), "white")
    draw = ImageDraw.Draw(image)
    cases = [
        ("准备位至下杆位", 9.48311245, 6.076703299),
        ("起落架组合", 68.465, 55.52277154),
        ("拉环左拨右拨组合", 82.883, 74.605),
    ]
    draw.rectangle((1050, 20, 1080, 45), fill="#A7B5BF")
    draw_text(draw, (1092, 13), "V4对照", 28, "#60717D")
    draw.rectangle((1300, 20, 1330, 45), fill="#185D75")
    draw_text(draw, (1342, 13), "V5候选", 28, "#253444")
    start, scale = 360, 11.0
    for value in (0, 20, 40, 60, 80):
        x = start + value * scale
        draw.line((x, 100, x, 557), fill="#E6EBEE", width=2)
        centered(draw, str(value), x, 590, 27, "#60717D")
    draw_text(draw, (1452, 574), "时间 / 秒", 27, "#60717D")
    for index, (label, before, after) in enumerate(cases):
        y = 107 + index * 153
        draw_text(draw, (15, y + 33), label, 33, "#253444")
        for offset, value, color in ((0, before, "#A7B5BF"), (53, after, "#185D75")):
            end = start + value * scale
            draw.rectangle((start, y + offset, end, y + offset + 32), fill=color)
            draw_text(draw, (end + 14, y + offset - 2), f"{value:.3f}", 28, "#253444")
        reduction = (before - after) / before * 100
        draw_text(draw, (1390, y + 30), f"-{reduction:.1f}%", 38, "#185D75")
    image.save(HERE / "assets/optimization.png", dpi=(240, 240))


def set_run_font(run, size=None, bold=None, color=None, east_asia=BODY_FONT):
    run.font.name = "Arial"
    properties = run._element.get_or_add_rPr()
    fonts = properties.find(qn("w:rFonts"))
    if fonts is None:
        fonts = OxmlElement("w:rFonts")
        properties.append(fonts)
    fonts.set(qn("w:eastAsia"), east_asia)
    fonts.set(qn("w:ascii"), "Arial")
    fonts.set(qn("w:hAnsi"), "Arial")
    if size is not None:
        run.font.size = Pt(size)
        cjk_size = properties.find(qn("w:szCs"))
        if cjk_size is None:
            cjk_size = OxmlElement("w:szCs")
            properties.append(cjk_size)
        cjk_size.set(qn("w:val"), str(round(size * 2)))
    if bold is not None:
        run.bold = bold
    if color is not None:
        run.font.color.rgb = RGBColor.from_string(color)


def configure_styles(doc):
    section = doc.sections[0]
    section.page_width = Cm(21)
    section.page_height = Cm(29.7)
    section.top_margin = Cm(2.05)
    section.bottom_margin = Cm(1.95)
    section.left_margin = Cm(2.25)
    section.right_margin = Cm(2.25)
    section.header_distance = Cm(0.85)
    section.footer_distance = Cm(0.9)
    section.different_first_page_header_footer = True

    normal = doc.styles["Normal"]
    normal.font.name = "Arial"
    normal.font.size = Pt(12)
    normal.font.color.rgb = RGBColor.from_string(INK)
    normal._element.get_or_add_rPr().get_or_add_rFonts().set(qn("w:eastAsia"), BODY_FONT)
    normal.paragraph_format.line_spacing = 1.42
    normal.paragraph_format.space_after = Pt(7)
    normal.paragraph_format.widow_control = True
    for name, size in (("Heading 1", 18), ("Heading 2", 13), ("Heading 3", 12)):
        style = doc.styles[name]
        style.font.name = "Arial"
        style.font.size = Pt(size)
        style.font.bold = True
        style.font.color.rgb = RGBColor.from_string(ACCENT if name == "Heading 1" else INK)
        style._element.get_or_add_rPr().get_or_add_rFonts().set(qn("w:eastAsia"), HEADING_FONT)
        style.paragraph_format.space_before = Pt(9 if name != "Heading 1" else 0)
        style.paragraph_format.space_after = Pt(7)
        style.paragraph_format.line_spacing = 1.16
        style.paragraph_format.keep_with_next = True
    settings = doc.settings.element
    language = OxmlElement("w:themeFontLang")
    language.set(qn("w:val"), "en-US")
    language.set(qn("w:eastAsia"), "zh-CN")
    settings.append(language)
    doc.core_properties.title = "人形机器人复杂操作任务关键技术攻关与调试成果报告"
    doc.core_properties.subject = "G1上肢操控系统技术成果总结"
    doc.core_properties.author = ""
    doc.core_properties.keywords = "人形机器人,视觉感知,柔顺示教,轨迹优化,边缘部署"
    doc.core_properties.comments = "Based on archived local implementations and commissioning evidence."


def add_field(paragraph, instruction):
    run = paragraph.add_run()
    begin = OxmlElement("w:fldChar")
    begin.set(qn("w:fldCharType"), "begin")
    code = OxmlElement("w:instrText")
    code.set(qn("xml:space"), "preserve")
    code.text = f" {instruction} "
    separate = OxmlElement("w:fldChar")
    separate.set(qn("w:fldCharType"), "separate")
    text = OxmlElement("w:t")
    text.text = "1"
    end = OxmlElement("w:fldChar")
    end.set(qn("w:fldCharType"), "end")
    for element in (begin, code, separate, text, end):
        run._r.append(element)
    set_run_font(run, 9, color=MUTED)


def add_header_footer(doc):
    section = doc.sections[0]
    paragraph = section.header.paragraphs[0]
    paragraph.paragraph_format.space_after = Pt(5)
    set_run_font(paragraph.add_run("G1 上肢操控系统  |  关键技术攻关与调试成果报告"), 9, color=MUTED)
    border = OxmlElement("w:pBdr")
    bottom = OxmlElement("w:bottom")
    for key, value in {"val": "single", "sz": "4", "space": "5", "color": "CCD5DA"}.items():
        bottom.set(qn("w:" + key), value)
    border.append(bottom)
    paragraph._p.get_or_add_pPr().append(border)
    footer = section.footer.paragraphs[0]
    footer.alignment = WD_ALIGN_PARAGRAPH.CENTER
    set_run_font(footer.add_run("第 "), 9, color=MUTED)
    add_field(footer, "PAGE")
    set_run_font(footer.add_run(" 页"), 9, color=MUTED)


def paragraph(doc, text, style=None, note=False):
    p = doc.add_paragraph(style=style)
    p.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    if style is None and not note:
        p.paragraph_format.first_line_indent = Pt(24)
    if note:
        p.paragraph_format.line_spacing = 1.24
        p.paragraph_format.space_before = Pt(5)
        p.paragraph_format.space_after = Pt(4)
    set_run_font(p.add_run(text), 9.5 if note else None,
                 color=MUTED if note else None,
                 east_asia=HEADING_FONT if style else BODY_FONT)
    return p


def add_bookmark(paragraph, name, identifier):
    begin = OxmlElement("w:bookmarkStart")
    begin.set(qn("w:id"), str(identifier))
    begin.set(qn("w:name"), name)
    end = OxmlElement("w:bookmarkEnd")
    end.set(qn("w:id"), str(identifier))
    paragraph._p.insert(0, begin)
    paragraph._p.append(end)


def add_hyperlink(paragraph, text, bookmark):
    link = OxmlElement("w:hyperlink")
    link.set(qn("w:anchor"), bookmark)
    run = OxmlElement("w:r")
    properties = OxmlElement("w:rPr")
    fonts = OxmlElement("w:rFonts")
    fonts.set(qn("w:eastAsia"), BODY_FONT)
    properties.append(fonts)
    run.append(properties)
    value = OxmlElement("w:t")
    value.text = text
    run.append(value)
    link.append(run)
    paragraph._p.append(link)


def add_table(doc, block):
    headers, rows = block["headers"], block["rows"]
    table = doc.add_table(rows=1, cols=len(headers))
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False
    fractions = block.get("widths", [1 / len(headers)] * len(headers))
    for column, width in zip(table.columns, fractions):
        column.width = Cm(16.5 * width)
    for i, value in enumerate(headers):
        table.rows[0].cells[i].text = value
    repeat = OxmlElement("w:tblHeader")
    table.rows[0]._tr.get_or_add_trPr().append(repeat)
    for row in rows:
        cells = table.add_row().cells
        for cell, value in zip(cells, row):
            cell.text = value
    borders = OxmlElement("w:tblBorders")
    for side in ("top", "bottom", "left", "right", "insideH", "insideV"):
        edge = OxmlElement("w:" + side)
        edge.set(qn("w:val"), "single")
        edge.set(qn("w:sz"), "4")
        edge.set(qn("w:color"), "D0D9DE")
        borders.append(edge)
    table._tbl.tblPr.append(borders)
    for index, row in enumerate(table.rows):
        no_split = OxmlElement("w:cantSplit")
        row._tr.get_or_add_trPr().append(no_split)
        for cell, width in zip(row.cells, fractions):
            cell.width = Cm(16.5 * width)
            cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            properties = cell._tc.get_or_add_tcPr()
            shade = OxmlElement("w:shd")
            shade.set(qn("w:fill"), ACCENT if index == 0 else ("F4F7F9" if index % 2 == 0 else WHITE))
            properties.append(shade)
            margins = OxmlElement("w:tcMar")
            for side in ("top", "bottom", "left", "right"):
                margin = OxmlElement("w:" + side)
                margin.set(qn("w:w"), "75" if side in ("top", "bottom") else "100")
                margin.set(qn("w:type"), "dxa")
                margins.append(margin)
            properties.append(margins)
            for p in cell.paragraphs:
                p.paragraph_format.space_after = Pt(0)
                p.paragraph_format.line_spacing = 1.20
                for run in p.runs:
                    set_run_font(run, 10, bold=index == 0,
                                 color=WHITE if index == 0 else INK,
                                 east_asia=HEADING_FONT if index == 0 else BODY_FONT)
    tail = doc.add_paragraph()
    tail.paragraph_format.space_after = Pt(0)
    tail.paragraph_format.line_spacing = 0.4
    set_run_font(tail.add_run(""), 2)
    return table


def add_callout(doc, text):
    p = paragraph(doc, text)
    p.paragraph_format.first_line_indent = Pt(0)
    p.paragraph_format.space_before = Pt(7)
    p.paragraph_format.space_after = Pt(8)
    p.paragraph_format.left_indent = Cm(0.25)
    p.paragraph_format.right_indent = Cm(0.25)
    p.paragraph_format.line_spacing = 1.28
    shade = OxmlElement("w:shd")
    shade.set(qn("w:fill"), LIGHT)
    p._p.get_or_add_pPr().append(shade)
    for run in p.runs:
        set_run_font(run, 11, color=ACCENT)


def cover(doc, content):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(50)
    p.paragraph_format.space_after = Pt(13)
    set_run_font(p.add_run("项目技术成果总结"), 13, bold=True, color=ACCENT, east_asia=HEADING_FONT)
    p = doc.add_paragraph()
    p.paragraph_format.space_after = Pt(23)
    set_run_font(p.add_run("G1"), 58, bold=True, color=INK)
    for key in ("title", "subtitle"):
        p = doc.add_paragraph()
        p.paragraph_format.space_after = Pt(9)
        p.paragraph_format.line_spacing = 1.10
        set_run_font(p.add_run(content[key]), 26, bold=True, color=INK, east_asia=HEADING_FONT)
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(20)
    p.paragraph_format.space_after = Pt(22)
    set_run_font(p.add_run("智能感知 · 柔顺示教 · 约束规划 · 连续优化 · 边缘执行"), 11, color=ACCENT)
    p = doc.add_paragraph()
    p.paragraph_format.space_after = Pt(25)
    p.paragraph_format.line_spacing = 1.50
    set_run_font(p.add_run("面向拉环拨转、滑轨推拉与起落架杆件操控的\n一体化技术体系与工程调试成果"), 13, color=MUTED)
    for label, value in (("项目对象", "Unitree G1 人形机器人上肢操控系统"),
                         ("资料周期", content["period"]), ("编制日期", content["date"])):
        p = doc.add_paragraph()
        p.paragraph_format.space_after = Pt(11)
        set_run_font(p.add_run(label + "："), 11, bold=True)
        set_run_font(p.add_run(value), 11)


def contents(doc, pages):
    entries = [("汇报摘要", 2)]
    for index, page in enumerate(pages, 2):
        if index <= 3:
            continue
        if page.get("kicker"):
            if page["kicker"].startswith("01"):
                entries.append((page["title"], index))
            name = next(b["text"] for b in page["blocks"] if b["type"] == "h2")
            entries.append(("    " + name, index))
        else:
            entries.append((page["title"], index))
    for label, page_number in entries:
        p = doc.add_paragraph()
        p.paragraph_format.space_after = Pt(13)
        p.paragraph_format.line_spacing = 1.22
        p.paragraph_format.tab_stops.add_tab_stop(Cm(15.7), WD_TAB_ALIGNMENT.RIGHT)
        add_hyperlink(p, label, f"page_{page_number}")
        set_run_font(p.add_run("\t" + str(page_number)), 11, color=MUTED)


def build_word(content):
    doc = Document()
    configure_styles(doc)
    add_header_footer(doc)
    cover(doc, content)
    for page_number, page in enumerate(content["pages"], 2):
        doc.add_page_break()
        if page.get("kicker"):
            p = doc.add_paragraph()
            p.paragraph_format.space_after = Pt(5)
            set_run_font(p.add_run(page["kicker"]), 10, color=ACCENT, east_asia=HEADING_FONT)
        heading = paragraph(doc, page["title"], style="Heading 1")
        add_bookmark(heading, f"page_{page_number}", page_number)
        for block in page["blocks"]:
            kind = block["type"]
            if kind == "p":
                paragraph(doc, block["text"])
            elif kind in ("h2", "h3"):
                paragraph(doc, block["text"], style="Heading 2" if kind == "h2" else "Heading 3")
            elif kind == "note":
                paragraph(doc, block["text"], note=True)
            elif kind == "callout":
                add_callout(doc, block["text"])
            elif kind == "table":
                add_table(doc, block)
            elif kind == "toc":
                contents(doc, content["pages"])
            elif kind == "image":
                p = doc.add_paragraph()
                p.alignment = WD_ALIGN_PARAGRAPH.CENTER
                p.paragraph_format.space_before = Pt(5)
                p.paragraph_format.space_after = Pt(3)
                p.paragraph_format.keep_with_next = True
                p.add_run().add_picture(str(HERE / "assets" / block["asset"]), width=Cm(16.2))
                caption = doc.add_paragraph()
                caption.alignment = WD_ALIGN_PARAGRAPH.CENTER
                caption.paragraph_format.space_after = Pt(9)
                set_run_font(caption.add_run(block["caption"]), 9.5, color=MUTED)
            else:
                raise ValueError(f"Unknown content block: {kind}")
    path = HERE / (STEM + ".docx")
    doc.save(path)
    return path


def build_markdown(content):
    lines = ["# " + content["title"] + "：" + content["subtitle"], "", content["platform"], "",
             "编制日期：" + content["date"], "", "资料周期：" + content["period"], ""]
    for page in content["pages"]:
        if page["title"] == "目录":
            continue
        lines.extend(["---", "", "## " + page["title"], ""])
        if page.get("kicker"):
            lines.extend([page["kicker"], ""])
        for block in page["blocks"]:
            kind = block["type"]
            if kind in ("p", "note", "callout"):
                lines.extend([block["text"], ""])
            elif kind in ("h2", "h3"):
                lines.extend([("### " if kind == "h2" else "#### ") + block["text"], ""])
            elif kind == "table":
                lines.append("| " + " | ".join(block["headers"]) + " |")
                lines.append("| " + " | ".join(["---"] * len(block["headers"])) + " |")
                lines.extend("| " + " | ".join(row) + " |" for row in block["rows"])
                lines.append("")
            elif kind == "image":
                lines.extend([f"![{block['caption']}](assets/{block['asset']})", ""])
    path = HERE / (STEM + ".md")
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def validate(word_path, markdown_path, content):
    with ZipFile(word_path) as archive:
        assert archive.testzip() is None
        xml = archive.read("word/document.xml").decode("utf-8")
        assert xml.count('w:type="page"') == len(content["pages"])
        assert xml.count("<w:bookmarkStart") == len(content["pages"])
        assert len([name for name in archive.namelist() if name.startswith("word/media/")]) == 2
        assert not any(word in xml for word in ("待完善事项", "国际领先", "100%成功", "/home/syf", "192.168."))
        assert "68.465" in xml and "55.523" in xml and "74.605" in xml
    loaded = Document(word_path)
    actual_blocks = "\n".join([p.text for p in loaded.paragraphs] +
                             [cell.text for table in loaded.tables for row in table.rows for cell in row.cells])
    for page in content["pages"]:
        for block in page["blocks"]:
            if "text" in block:
                assert block["text"] in actual_blocks, block["text"][:40]
            if block["type"] == "table":
                for row in block["rows"]:
                    assert all(value in actual_blocks for value in row)
    source_paths = [
        "REAL_TASK_APPLICATION.md", "src/g1_unitree_control/README.md",
        "src/g1_upper_body_moveit_config/README.md", "src/g1_kdl_ik/README.md",
        "logs/camera_c_analysis_20260913_153025/REPORT.md",
        "logs/dring_torque_review_20260920/REPORT.md",
        "deployment/rail_jetson/INTERNAL_STOP_OPTIMIZATION_V5.md",
        "deployment/rail_jetson/LATENCY_MOTOR_REVIEW.md",
        "deployment/rail_jetson/GEAR_PROTOCOL_UPDATE.md",
        "deployment/rail_jetson/README.md", "docs/trajectory_only_results_20260929.md",
    ]
    manifest = {
        "content_verified": True,
        "intended_pages": len(content["pages"]) + 1,
        "editable_word_tables": len(loaded.tables),
        "embedded_figures": len(loaded.inline_shapes),
        "chinese_text_characters": len(actual_blocks),
        "sources_sha256": {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in source_paths},
        "artifacts_sha256": {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in (word_path, markdown_path)},
    }
    (HERE / "validation.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in manifest.items() if not key.endswith("sha256")}, ensure_ascii=False, indent=2))
    print(word_path)


def main():
    content = json.loads((HERE / "report_content.json").read_text(encoding="utf-8"))
    make_architecture()
    make_optimization()
    word_path = build_word(content)
    markdown_path = build_markdown(content)
    validate(word_path, markdown_path, content)


if __name__ == "__main__":
    main()
