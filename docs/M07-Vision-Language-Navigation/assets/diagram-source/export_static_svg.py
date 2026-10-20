#!/usr/bin/env python3
"""Export the delivered HTML's unchanged SVG geometry as a standalone image."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ASSETS = Path(__file__).resolve().parent.parent


def export(name):
    html = (ASSETS / (name + '.html')).read_text()
    svg = re.search(r'<svg\b[^>]*>.*?</svg>', html, re.S).group()
    # Only the actual first stylesheet; later script literals are not styles.
    css = re.search(r'<style[^>]*>(.*?)</style>', html, re.S).group(1)
    dimensions = re.search(r'viewBox="0 0 ([\d.]+) ([\d.]+)"', svg)
    width, height = dimensions.groups()
    svg = re.sub(r'<svg\b', f'<svg xmlns="http://www.w3.org/2000/svg" data-theme="light" width="{width}" height="{height}" style="width:{width}px;height:{height}px"', svg, count=1)
    position = svg.index('>') + 1
    svg = svg[:position] + '<style><![CDATA[' + css + ']]></style><rect width="100%" height="100%" fill="#f8fafc"/>' + svg[position:]
    ET.fromstring(svg)
    (ASSETS / (name + '.svg')).write_text(svg)


if __name__ == '__main__':
    for name in ['01-closed-loop', '02-model-io', '03-rvq', '05-async']:
        export(name)
