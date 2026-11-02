"""Produce a lighter Word-only technical summary from the reviewed report."""

from copy import deepcopy
import hashlib
import json
import re
from zipfile import ZipFile

from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.shared import Cm, Pt, RGBColor

from build_report import HERE, add_field, add_table, configure_styles, paragraph, set_run_font


TITLE = "G1机器人调试技术总结"
OUTPUT = HERE / (TITLE + "_修订版.docx")


def revised_sections(source):
    technical = [deepcopy(page) for page in source["pages"] if page.get("kicker")]
    technical[0]["blocks"] = [block for block in technical[0]["blocks"]
                               if block["type"] != "note"]
    for block in technical[0]["blocks"]:
        if block.get("text", "").startswith("完成采样窗口稳定性检查"):
            block["text"] = (
                "通过采样稳定性检查、原始图像与关节状态关联、拟合与验证分组以及参数敏感性分析，"
                "建立了可复算的标定评估方法。相机外参候选在独立验证组中的平移变换链残差和旋转"
                "残差均得到改善，使坐标一致性问题能够通过数据分析定位，而不再仅依靠现场反复试调。"
            )
    technical[5]["blocks"] = [block for block in technical[5]["blocks"]
                               if block["type"] not in ("image", "note")]
    technical[5]["blocks"].append({
        "type": "callout",
        "text": "解决问题：减少轨迹内部的反复减速与非必要停顿，在保持操作路径和必要等待的前提下，改善动作平顺性和运行节拍。同配置离线对比已验证组合轨迹时长及内部低速谷数量的改善。",
    })
    for page in technical:
        for block in page["blocks"]:
            if block.get("text", "").startswith("完成由调试工作站向Jetson"):
                block["text"] = (
                    "完成调试工作站与Jetson边缘计算平台之间的运行适配，将轨迹资源、原生控制"
                    "网关、任务编排与HTTP业务接口整合为机载执行系统。通过软件检查和跨架构数值"
                    "比对，保持开发端与机载端的轨迹表达和插值结果一致，实现调试成果向机载运行"
                    "环境的迁移。"
                )

    sections = [
        {
            "title": "一、调试工作概述",
            "blocks": [
                {"type": "p", "text": "围绕G1机器人在固定工位开展拉环拨转、滑轨推拉和起落架杆件操控，完成了上肢操控系统的技术集成与调试。相关工作涵盖机器人建模、视觉定位、柔顺示教、运动规划、轨迹优化、执行防护和机载部署，打通了从目标识别到动作执行的技术流程。"},
                {"type": "p", "text": "调试重点是解决复杂操作中的实际问题，包括工具坐标与视觉坐标不一致、机械臂逆解跳变、末端负载变化、示教轨迹抖动与停顿、手臂和手型时序不匹配，以及开发环境与机载环境存在差异等。针对这些问题，已建立对应的算法方法、调试工具和可复用动作资源。"},
            ],
        },
        {
            "title": "二、整体技术路线",
            "blocks": [
                {"type": "p", "text": "整体采用“感知与建模—任务规划—轨迹优化—受控执行—数据记录”的分层结构。视觉和机器人状态提供输入，统一模型负责坐标与工具关系，规划及优化模块生成连续动作，执行端负责状态检查、控制输出和异常处理。"},
                {"type": "image", "asset": "architecture.png", "caption": "上肢操作任务技术链路"},
                {"type": "p", "text": "任务生成采用视觉规划与实测示教两条路线：视觉规划根据目标位置、操作方向和机器人运动能力生成路径；实测示教保留人工确定的有效操作几何，再进行平顺处理和受限调速。两类路线统一使用动作资源和执行接口。"},
                {"type": "p", "text": "规划、预览和执行采用一致的轨迹数据。执行前核查实际起点、状态时效和运动限值，执行过程中记录机器人状态与任务结果，使调试问题能够定位到具体动作和技术版本。"},
            ],
        },
        {
            "title": "三、核心技术与解决的问题",
            "blocks": [block for page in technical for block in page["blocks"]],
        },
        {
            "title": "四、典型任务调试成果",
            "blocks": [
                {"type": "h2", "text": "（一）拉环拨转"},
                {"type": "p", "text": "围绕接近、手型准备、插入、拨动、退出和返回等阶段，完成视觉任务规划与实测示教回放两类流程。通过固定采集目标、区分旋转轴心与圆弧参考、筛选连续逆解以及复用记录动作，解决目标定义混用、腕部姿态变化、逆解分支切换和退出路径受限等问题。"},
                {"type": "p", "text": "已形成左拨与右拨动作单元、准备位与记录起点衔接、完整组合预览和受控执行能力。左臂完整记录回放和机载组合执行均已有成功结束的动作记录，完成了从示教资源到组合执行的流程贯通。"},
                {"type": "h2", "text": "（二）滑轨推拉"},
                {"type": "p", "text": "针对推进与拉出方向分别组织接近、接触和退出记录，建立张开、推进握持和拉出握持等手型资源。通过录制端点提取、受限衔接、手型等待和当前起点核查，解决不同方向路径不一致、手型切换时机不清以及动作资源重复拟合等问题。"},
                {"type": "p", "text": "完成记录缓存、原始路径约束下的平顺拟合和局部时间优化。离线候选对比显示，拉出接近与退出段的时长得到缩短，退出段内部停顿得到消除；同时建立几何修改与连续导数的独立校核方法，使路径调整和节拍优化能够分别评价。"},
                {"type": "h2", "text": "（三）起落架杆件操控"},
                {"type": "p", "text": "将准备位接近、上下行操作、握持松手和退出组织为独立动作单元，通过循环次数、前置下放和末轮回收方式映射业务路径，解决单行程、往返周期和完整业务流程之间容易混淆的问题。"},
                {"type": "p", "text": "完成局部去停顿、连续拼接、分块重复和控制进度校核。同配置离线对比中，准备位接近单元的内部低速谷得到消除，组合轨迹时长得到缩短。配套任务协议已完成机载部署，服务运行及四类组合配方加载核验通过，形成可按业务要求组织的动作体系。"},
            ],
        },
    ]
    return sections


def build(sections):
    doc = Document()
    configure_styles(doc)
    doc.sections[0].different_first_page_header_footer = False
    normal = doc.styles["Normal"]
    normal.font.size = Pt(11)
    normal.paragraph_format.line_spacing = 1.32
    normal.paragraph_format.space_after = Pt(6)
    for name, size in (("Heading 1", 15), ("Heading 2", 12), ("Heading 3", 11)):
        style = doc.styles[name]
        style.font.size = Pt(size)
        style.font.color.rgb = RGBColor.from_string("253444")
        style.paragraph_format.space_before = Pt(11 if name == "Heading 1" else 8)
        style.paragraph_format.space_after = Pt(5)
    doc.core_properties.title = TITLE
    doc.core_properties.subject = "上肢操作调试中采用的技术及解决的问题"

    header = doc.sections[0].header.paragraphs[0]
    set_run_font(header.add_run(TITLE), 9, color="60717D")
    footer = doc.sections[0].footer.paragraphs[0]
    footer.alignment = WD_ALIGN_PARAGRAPH.CENTER
    add_field(footer, "PAGE")
    title = doc.add_paragraph()
    title.alignment = WD_ALIGN_PARAGRAPH.CENTER
    title.paragraph_format.space_after = Pt(12)
    set_run_font(title.add_run(TITLE), 18, bold=True, east_asia="黑体")
    for section in sections:
        paragraph(doc, section["title"], style="Heading 1")
        for block in section["blocks"]:
            kind = block["type"]
            if kind in ("p", "callout"):
                p = paragraph(doc, block["text"])
                p.paragraph_format.first_line_indent = Pt(22)
                if kind == "callout":
                    p.paragraph_format.first_line_indent = Pt(0)
                    p.runs[0].bold = True
            elif kind in ("h2", "h3"):
                paragraph(doc, block["text"], style="Heading 2" if kind == "h2" else "Heading 3")
            elif kind == "table":
                add_table(doc, block)
            elif kind == "image":
                p = doc.add_paragraph()
                p.alignment = WD_ALIGN_PARAGRAPH.CENTER
                p.paragraph_format.keep_with_next = True
                p.add_run().add_picture(str(HERE / "assets" / block["asset"]), width=Cm(14.6))
                p = doc.add_paragraph()
                p.alignment = WD_ALIGN_PARAGRAPH.CENTER
                set_run_font(p.add_run(block["caption"]), 9, color="60717D")
            else:
                raise ValueError(kind)
    doc.save(OUTPUT)


def verify(sections):
    with ZipFile(OUTPUT) as archive:
        assert archive.testzip() is None
        xml = archive.read("word/document.xml").decode("utf-8")
        assert 'w:type="page"' not in xml
    doc = Document(OUTPUT)
    body = "\n".join([p.text for p in doc.paragraphs] + [
        cell.text for table in doc.tables for row in table.rows for cell in row.cells
    ])
    for section in sections:
        assert section["title"] in body
        for block in section["blocks"]:
            if "text" in block:
                assert block["text"] in body, block["text"][:40]
            if block["type"] == "table":
                for row in block["rows"]:
                    assert all(value in body for value in row)
    forbidden = ("五、量化成果", "六、技术特色", "汇报摘要", "交付成果清单",
                 "待完善事项", "编制日期", "资料周期", "/home/syf", "总体成果结论")
    assert not any(text in body for text in forbidden)
    assert not re.search(r"20\d{2}(?:年|[-/])\d{1,2}", body)
    assert not re.search(r"\d{1,2}月\d{1,2}日", body)
    assert len(doc.tables) == 1
    assert len(doc.inline_shapes) == 1
    assert [p.text for p in doc.paragraphs if p.style.name == "Heading 1"] == [
        section["title"] for section in sections
    ]
    terms = ("SVD", "ChArUco", "YOLOv8", "导纳", "雅可比", "RRTConnect", "Ruckig",
             "Hermite", "Bernstein", "SLSQP", "Jetson", "SHA-256")
    assert all(term in body for term in terms)
    result = {
        "file": OUTPUT.name,
        "content_verified": True,
        "calendar_dates_removed": True,
        "chapters_5_6_removed": True,
        "cover_summary_toc_removed": True,
        "major_sections": len(sections),
        "technical_sections": 8,
        "editable_tables": len(doc.tables),
        "embedded_figures": len(doc.inline_shapes),
        "text_characters": len(body),
        "sha256": hashlib.sha256(OUTPUT.read_bytes()).hexdigest(),
    }
    (HERE / "revised_validation.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(OUTPUT)


if __name__ == "__main__":
    original = json.loads((HERE / "report_content.json").read_text(encoding="utf-8"))
    sections = revised_sections(original)
    build(sections)
    verify(sections)
