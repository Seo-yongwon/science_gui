"""캡처 이미지를 파일명·경로 규칙에 따라 HTML 보고서에 자동 반영."""
from __future__ import annotations

import html
import os
import re
from datetime import datetime

DEFAULT_HTML_NAME = 'mission_report.html'

# (파일명 정규식, 섹션 제목, save_dir 기준 하위 폴더 또는 None=루트)
_CAPTURE_RULES: tuple[tuple[re.Pattern[str], str, str | None], ...] = (
    (re.compile(r'^soil_.*\.png$', re.I),        '토양',       None),
    (re.compile(r'^cashe_.*\.png$', re.I),       '캐시',       None),
    (re.compile(r'^pano_live_.*\.png$', re.I),  '파노 스트림', None),
    (re.compile(r'^front_.*\.png$', re.I),      '전면',       None),
    (re.compile(r'^back_.*\.png$', re.I),       '후면',       None),
    (re.compile(r'^merged_.*\.png$', re.I),     '전체 합성',  None),
    (re.compile(r'^panorama_.*\.png$', re.I), '파노라마',   'panorama'),
    (re.compile(r'^npk_graph_.*\.png$', re.I), 'NPK 그래프', 'npk'),
    (re.compile(r'^spectrum_.*\.png$', re.I), '분광기',    'spectrometer'),
)

_SECTION_ORDER = [
    '토양', '캐시', '파노 스트림', '전면', '후면', '전체 합성', '파노라마', 'NPK 그래프', '분광기',
]


def _categorize(save_dir: str, filepath: str) -> str | None:
    save_dir = os.path.abspath(os.path.expanduser(save_dir))
    filepath = os.path.abspath(filepath)
    if not filepath.startswith(save_dir + os.sep) and filepath != save_dir:
        return None

    rel = os.path.relpath(filepath, save_dir)
    parts = rel.split(os.sep)
    basename = os.path.basename(filepath)
    subdir = parts[0] if len(parts) > 1 else None

    for pattern, section, expected_subdir in _CAPTURE_RULES:
        if expected_subdir is not None:
            if subdir != expected_subdir:
                continue
        elif len(parts) > 1:
            continue
        if pattern.match(basename):
            return section
    return None


def _collect_images(save_dir: str) -> dict[str, list[tuple[str, float]]]:
    save_dir = os.path.abspath(os.path.expanduser(save_dir))
    grouped: dict[str, list[tuple[str, float]]] = {s: [] for s in _SECTION_ORDER}

    if not os.path.isdir(save_dir):
        return grouped

    for dirpath, _dirnames, filenames in os.walk(save_dir):
        for name in filenames:
            if not name.lower().endswith('.png'):
                continue
            full = os.path.join(dirpath, name)
            section = _categorize(save_dir, full)
            if section is None:
                continue
            rel = os.path.relpath(full, save_dir).replace(os.sep, '/')
            grouped[section].append((rel, os.path.getmtime(full)))

    for items in grouped.values():
        items.sort(key=lambda x: x[1], reverse=True)
    return grouped


def _fmt_time(ts: float) -> str:
    return datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')


def _render_html(save_dir: str, grouped: dict[str, list[tuple[str, float]]]) -> str:
    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    save_dir_disp = html.escape(os.path.abspath(os.path.expanduser(save_dir)))

    sections: list[str] = []
    total = 0
    for title in _SECTION_ORDER:
        items = grouped.get(title, [])
        if not items:
            continue
        total += len(items)
        cards = []
        for rel, mtime in items:
            rel_esc = html.escape(rel)
            cards.append(
                f'<figure class="card">'
                f'<a href="{rel_esc}" target="_blank">'
                f'<img src="{rel_esc}" alt="{rel_esc}" loading="lazy">'
                f'</a>'
                f'<figcaption>{rel_esc}<br><time>{_fmt_time(mtime)}</time></figcaption>'
                f'</figure>'
            )
        sections.append(
            f'<section id="{html.escape(title)}">'
            f'<h2>{html.escape(title)} <span class="count">{len(items)}</span></h2>'
            f'<div class="grid">{"".join(cards)}</div>'
            f'</section>'
        )

    body = (
        '\n'.join(sections)
        if sections
        else '<p class="empty">아직 등록된 캡처 이미지가 없습니다.</p>'
    )

    return f'''<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>미션 캡처 보고서</title>
  <style>
    :root {{
      --bg: #1e1e2e; --surface: #11111b; --border: #313244;
      --text: #cdd6f4; --muted: #6c7086; --accent: #89b4fa;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0; padding: 24px; background: var(--bg); color: var(--text);
      font-family: "Segoe UI", "Noto Sans", "Ubuntu", sans-serif;
    }}
    header {{
      margin-bottom: 24px; padding-bottom: 16px; border-bottom: 1px solid var(--border);
    }}
    h1 {{ margin: 0 0 8px; font-size: 1.6rem; }}
    .meta {{ color: var(--muted); font-size: 0.9rem; line-height: 1.5; }}
    section {{ margin-bottom: 32px; }}
    h2 {{
      margin: 0 0 12px; font-size: 1.1rem; color: var(--accent);
      display: flex; align-items: center; gap: 8px;
    }}
    .count {{
      background: var(--border); color: var(--text); font-size: 0.75rem;
      padding: 2px 8px; border-radius: 999px;
    }}
    .grid {{
      display: grid; grid-template-columns: repeat(auto-fill, minmax(260px, 1fr)); gap: 16px;
    }}
    .card {{
      margin: 0; background: var(--surface); border: 1px solid var(--border);
      border-radius: 10px; overflow: hidden;
    }}
    .card img {{
      display: block; width: 100%; height: 180px; object-fit: cover; background: #000;
    }}
    .card figcaption {{
      padding: 10px 12px; font-size: 0.78rem; color: var(--muted); word-break: break-all;
    }}
    .card time {{ color: #a6adc8; }}
    .empty {{ color: var(--muted); }}
  </style>
</head>
<body>
  <header>
    <h1>미션 캡처 보고서</h1>
    <p class="meta">
      저장 위치: {save_dir_disp}<br>
      마지막 갱신: {now} · 총 {total}장
    </p>
  </header>
  {body}
</body>
</html>
'''


def rebuild_html_report(
    save_dir: str,
    html_name: str = DEFAULT_HTML_NAME,
) -> str | None:
    """save_dir 아래 이미지를 스캔해 HTML 보고서를 재생성한다."""
    save_dir = os.path.abspath(os.path.expanduser(save_dir))
    os.makedirs(save_dir, exist_ok=True)

    grouped = _collect_images(save_dir)
    content = _render_html(save_dir, grouped)
    html_path = os.path.join(save_dir, html_name)

    with open(html_path, 'w', encoding='utf-8') as f:
        f.write(content)
    return html_path


def register_capture(
    save_dir: str,
    image_path: str,
    html_name: str = DEFAULT_HTML_NAME,
) -> str | None:
    """저장된 이미지를 HTML 보고서에 반영한다. (전체 재생성)"""
    if not os.path.isfile(image_path):
        return None
    if _categorize(save_dir, image_path) is None:
        return None
    return rebuild_html_report(save_dir, html_name)
