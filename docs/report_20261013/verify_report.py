"""Verify rendered pagination and content, and generate a visual overview."""

import hashlib
import json
import re
import subprocess
import unicodedata
from xml.etree import ElementTree

from PIL import Image, ImageDraw, ImageFont

from build_report import HERE, ROOT, STEM, LATIN_FONT_FILE


def normalize(text):
    return re.sub(r"\s+", "", unicodedata.normalize("NFKC", text))


def main():
    content = json.loads((HERE / "report_content.json").read_text(encoding="utf-8"))
    pdf = HERE / (STEM + ".pdf")
    info = subprocess.check_output(["pdfinfo", str(pdf)], text=True)
    pages_count = int(re.search(r"^Pages:\s+(\d+)", info, re.M).group(1))
    assert pages_count == len(content["pages"]) + 1, info
    extracted = subprocess.check_output(["pdftotext", "-layout", str(pdf), "-"], text=True)
    pages = extracted.split("\f")
    pages = [page for page in pages if page.strip()]
    assert len(pages) == pages_count
    bbox = ElementTree.fromstring(subprocess.check_output(
        ["pdftotext", "-bbox-layout", str(pdf), "-"], text=True))
    namespace = {"pdf": "http://www.w3.org/1999/xhtml"}
    bbox_pages = bbox.findall(".//pdf:page", namespace)
    assert len(bbox_pages) == pages_count
    missing = []
    for number, page in enumerate(content["pages"], 2):
        actual = normalize(pages[number - 1])
        # PDF layout extraction interleaves columns when table cells wrap.
        cell_blocks = [normalize("".join(block.itertext())) for block in
                       bbox_pages[number - 1].findall(".//pdf:block", namespace)]
        assert normalize(page["title"]) in actual, (number, page["title"])
        assert normalize(f"第{number}页") in actual, (number, "footer")
        for block in page["blocks"]:
            if "text" in block and normalize(block["text"]) not in actual:
                missing.append((number, block["text"][:50]))
            if block["type"] == "table":
                for row in block["rows"]:
                    for value in row:
                        if (normalize(value) not in actual and
                                not any(normalize(value) in text for text in cell_blocks)):
                            missing.append((number, value))
    assert not missing, missing

    toc_lines = {normalize(line) for line in pages[2].splitlines()}
    assert normalize("汇报摘要2") in toc_lines
    for number, page in enumerate(content["pages"], 2):
        if number <= 3:
            continue
        labels = []
        if page.get("kicker"):
            if page["kicker"].startswith("01"):
                labels.append(page["title"])
            labels.append(next(block["text"] for block in page["blocks"]
                               if block["type"] == "h2"))
        else:
            labels.append(page["title"])
        for label in labels:
            assert normalize(label + str(number)) in toc_lines, (label, number)

    sources = {
        "dring_live": "logs/dring_torque_review_20260920/REPORT.md",
        "dring_jetson": "deployment/rail_jetson/LATENCY_MOTOR_REVIEW.md",
        "calibration": "logs/camera_c_analysis_20260913_153025/REPORT.md",
        "optimization": "deployment/rail_jetson/INTERNAL_STOP_OPTIMIZATION_V5.md",
        "rail": "docs/trajectory_only_results_20260929.md",
        "parity": "deployment/rail_jetson/README.md",
        "protocol": "deployment/rail_jetson/GEAR_PROTOCOL_UPDATE.md",
    }
    numbers = {
        "dring_live": ["两轮", "成功结束"],
        "dring_jetson": ["SUCCEEDED", "74.605"],
        "calibration": ["11.717", "5.591", "1.688", "1.169"],
        "optimization": ["68.465", "55.523", "82.883", "74.605", "78 项"],
        "rail": ["22.007150", "17.000000", "7.758369"],
        "parity": ["10,799", "1e-10", "16 segments"],
        "protocol": ["58/58", "26/26", "Deployed to Jetson"],
    }
    for name, source in sources.items():
        text = (ROOT / source).read_text(encoding="utf-8")
        assert all(number in text for number in numbers[name]), source

    render_dir = HERE / "assets/rendered_pages"
    thumbs = []
    for number in range(1, pages_count + 1):
        image = Image.open(render_dir / f"page-{number:02}.png").convert("RGB")
        assert image.width > 600 and image.height >= 1000
        assert image.convert("L").getextrema()[0] < 100
        image.thumbnail((355, 502))
        thumbs.append(image)
    overview = Image.new("RGB", (1510, 2200), "#E8ECEF")
    draw = ImageDraw.Draw(overview)
    for index, image in enumerate(thumbs):
        x, y = 20 + index % 4 * 375, 15 + index // 4 * 550
        overview.paste(image, (x, y))
        draw.text((x + 160, y + 510), str(index + 1),
                  font=ImageFont.truetype(LATIN_FONT_FILE, 20), fill="#253444")
    overview_path = HERE / "assets/report_contact_sheet.png"
    overview.save(overview_path)
    validation_path = HERE / "validation.json"
    result = json.loads(validation_path.read_text(encoding="utf-8"))
    result.update(rendered_pages=pages_count, rendered_content_verified=True,
                  toc_page_numbers_verified=True, source_numbers_verified=True)
    result["artifacts_sha256"][pdf.name] = hashlib.sha256(pdf.read_bytes()).hexdigest()
    validation_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in result.items() if not key.endswith("sha256")}, ensure_ascii=False, indent=2))
    print(overview_path)


if __name__ == "__main__":
    main()
