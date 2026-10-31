#!/usr/bin/env python3
"""Render the local Markdown report to a Japanese PDF with embedded IPA font."""
import argparse
from html import escape
from pathlib import Path
import re
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image


def clean(text):
    return escape(re.sub(r'\[([^]]+)\]\(([^)]+)\)', r'\1 (\2)', text).replace('`', '').replace('**', ''))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, default=Path('REPORT.md'))
    parser.add_argument('--output', type=Path, default=Path('REPORT.pdf'))
    args = parser.parse_args()
    pdfmetrics.registerFont(TTFont('IPA', '/usr/share/fonts/opentype/ipafont-gothic/ipag.ttf'))
    body = ParagraphStyle('body', fontName='IPA', fontSize=9, leading=15, wordWrap='CJK', spaceAfter=7)
    heading = ParagraphStyle('heading', parent=body, fontSize=13, leading=19, spaceBefore=13,
                             keepWithNext=True)
    caption = ParagraphStyle('caption', parent=body, fontSize=8, leading=12, spaceAfter=0)
    story, table, figures = [], [], []

    def flush_figures():
        while figures:
            pair = figures[:2]
            del figures[:2]
            column_width = (A4[0] - 80) / len(pair)
            cells = []
            for label, relative_path in pair:
                path = args.input.resolve().parent / relative_path
                picture = Image(str(path), mask='auto')
                scale = min((column_width - 12) / picture.imageWidth, 175 / picture.imageHeight)
                picture.drawWidth = picture.imageWidth * scale
                picture.drawHeight = picture.imageHeight * scale
                picture.hAlign = 'CENTER'
                cells.append([picture, Spacer(1, 6), Paragraph(clean(label), caption)])
            block = Table([cells], colWidths=[column_width] * len(pair), splitByRow=0)
            block.setStyle(TableStyle([('VALIGN', (0, 0), (-1, -1), 'TOP'),
                                       ('TOPPADDING', (0, 0), (-1, -1), 6),
                                       ('BOTTOMPADDING', (0, 0), (-1, -1), 6)]))
            story.extend([block, Spacer(1, 8)])

    def flush():
        if table:
            width = A4[0] - 80
            rows = [[Paragraph(clean(cell), body) for cell in row] for row in table]
            col_widths = [width / len(table[0])] * len(table[0])
            criteria_table = len(table[0]) == 2 and table[0][0] == '観点'
            if criteria_table:
                # Three Japanese glyphs at the body font size, plus cell padding.
                label_width = pdfmetrics.stringWidth('創造性', body.fontName, body.fontSize) + 12
                col_widths = [label_width, width - label_width]
            block = Table(rows, colWidths=col_widths, repeatRows=1,
                          splitByRow=0 if criteria_table else 1)
            block.setStyle(TableStyle([('GRID', (0, 0), (-1, -1), .3, colors.grey),
                                       ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#e6edf4')),
                                       ('VALIGN', (0, 0), (-1, -1), 'TOP')]))
            story.extend([block, Spacer(1, 10)])
            table.clear()
    for line in args.input.read_text().splitlines():
        figure = re.fullmatch(r'!\[([^]]*)\]\(([^)]+)\)', line.strip())
        if figure:
            flush()
            figures.append(figure.groups())
            continue
        if not line.strip():
            flush()
            continue
        flush_figures()
        if line.startswith('|'):
            cells = [cell.strip() for cell in line.strip('|').split('|')]
            if not all(re.fullmatch(r'[:\-\s]+', cell) for cell in cells):
                table.append(cells)
            continue
        flush()
        if line.startswith('#'):
            story.append(Paragraph(clean(line.lstrip('# ')), heading))
        else:
            story.append(Paragraph(clean(line.lstrip('> ')), body))
    flush()
    flush_figures()
    document = SimpleDocTemplate(str(args.output), pagesize=A4, rightMargin=40, leftMargin=40,
                                 topMargin=35, bottomMargin=35, title='Physical AI 応用1 最終課題', author='chocopan',
                                 subject='双腕ロボットROBOTIS AI Worker SG2を使ったsim環境でのホワイトボードの掃除')
    document.build(story)
    print(args.output)


if __name__ == '__main__':
    main()
