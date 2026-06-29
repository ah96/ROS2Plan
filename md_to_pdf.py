#!/usr/bin/env python3
"""Convert ros2_migration.md to a styled PDF using markdown + weasyprint."""

import sys
import markdown
from weasyprint import HTML, CSS

def convert(md_path, out_path):
    with open(md_path, encoding='utf-8') as f:
        md_text = f.read()

    html_body = markdown.markdown(
        md_text,
        extensions=['tables', 'fenced_code', 'toc', 'attr_list']
    )

    css = CSS(string="""
        @import url('https://fonts.googleapis.com/css2?family=Source+Sans+Pro:wght@400;600;700&family=Source+Code+Pro&display=swap');

        @page {
            size: A4;
            margin: 2.2cm 2.4cm 2.5cm 2.4cm;
            @bottom-center {
                content: counter(page) " / " counter(pages);
                font-family: 'Source Sans Pro', sans-serif;
                font-size: 9pt;
                color: #888;
            }
        }

        body {
            font-family: 'Source Sans Pro', Georgia, serif;
            font-size: 10.5pt;
            line-height: 1.55;
            color: #1a1a1a;
        }

        /* --- Headings --- */
        h1 {
            font-size: 20pt;
            font-weight: 700;
            color: #1c3557;
            border-bottom: 2.5px solid #1c3557;
            padding-bottom: 6px;
            margin-top: 0;
            margin-bottom: 18px;
            page-break-after: avoid;
        }

        h2 {
            font-size: 14pt;
            font-weight: 700;
            color: #1c3557;
            border-bottom: 1px solid #c8d8ea;
            padding-bottom: 4px;
            margin-top: 28px;
            margin-bottom: 10px;
            page-break-after: avoid;
        }

        h3 {
            font-size: 11.5pt;
            font-weight: 600;
            color: #2c4a6a;
            margin-top: 20px;
            margin-bottom: 6px;
            page-break-after: avoid;
        }

        h4 {
            font-size: 10.5pt;
            font-weight: 600;
            color: #3a5a7a;
            margin-top: 14px;
            margin-bottom: 4px;
        }

        /* --- Paragraphs and lists --- */
        p { margin: 0 0 8px 0; }

        ul, ol {
            margin: 6px 0 10px 0;
            padding-left: 22px;
        }

        li { margin-bottom: 3px; }

        /* --- Code --- */
        code {
            font-family: 'Source Code Pro', 'Courier New', monospace;
            font-size: 8.5pt;
            background: #f4f6f8;
            border: 1px solid #dce3ea;
            border-radius: 3px;
            padding: 1px 4px;
            color: #c7254e;
        }

        pre {
            background: #f4f6f8;
            border: 1px solid #dce3ea;
            border-left: 4px solid #1c3557;
            border-radius: 4px;
            padding: 10px 14px;
            margin: 8px 0 14px 0;
            overflow: auto;
            page-break-inside: avoid;
        }

        pre code {
            background: none;
            border: none;
            padding: 0;
            color: #1a1a1a;
            font-size: 8pt;
            line-height: 1.45;
        }

        /* --- Tables --- */
        table {
            border-collapse: collapse;
            width: 100%;
            margin: 10px 0 16px 0;
            font-size: 9.5pt;
            page-break-inside: avoid;
        }

        thead tr {
            background: #1c3557;
            color: #ffffff;
        }

        th {
            padding: 7px 10px;
            text-align: left;
            font-weight: 600;
        }

        td {
            padding: 6px 10px;
            border-bottom: 1px solid #dde4ed;
        }

        tbody tr:nth-child(even) {
            background: #f4f7fb;
        }

        /* --- Horizontal rule --- */
        hr {
            border: none;
            border-top: 1.5px solid #c8d8ea;
            margin: 22px 0;
        }

        /* --- Blockquotes --- */
        blockquote {
            border-left: 4px solid #1c3557;
            margin: 10px 0 10px 0;
            padding: 6px 14px;
            background: #f0f4f9;
            color: #444;
            font-style: italic;
        }

        /* --- Links (convert to plain for print) --- */
        a { color: #1c6ea4; text-decoration: none; }

        /* --- Table of contents --- */
        .toc { margin-bottom: 24px; }
    """)

    html_full = f"""<!DOCTYPE html>
<html lang="en">
<head><meta charset="utf-8"><title>ROSPlan ROS2 Migration</title></head>
<body>{html_body}</body>
</html>"""

    HTML(string=html_full, base_url='.').write_pdf(out_path, stylesheets=[css])
    print(f"Written: {out_path}")

if __name__ == '__main__':
    md  = sys.argv[1] if len(sys.argv) > 1 else 'ros2_migration.md'
    out = sys.argv[2] if len(sys.argv) > 2 else 'ros2_migration.pdf'
    convert(md, out)
