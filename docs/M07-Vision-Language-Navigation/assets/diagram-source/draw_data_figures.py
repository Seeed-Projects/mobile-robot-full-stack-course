#!/usr/bin/env python3
"""Deterministic vector figures from frozen, genuine wheeled-robot records."""
import html
import json
import math
from pathlib import Path

COURSE = Path(__file__).resolve().parents[2]
REPO = COURSE.parents[1]
EVIDENCE = COURSE / 'code/evidence'
ASSETS = COURSE / 'assets'


def text(x, y, value, size=18, color='#334155', anchor='start'):
    return f'<text x="{x}" y="{y}" font-size="{size}" fill="{color}" text-anchor="{anchor}">{html.escape(str(value))}</text>'


def line(x1, y1, x2, y2, color='#64748b', width=2, dash=''):
    return f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="{width}" stroke-dasharray="{dash}"/>'


def begin(title, width=1100, height=550):
    return [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" role="img"><title>{html.escape(title)}</title>',
            '<style>text{font-family:system-ui,"PingFang SC","Microsoft YaHei",sans-serif} .path{fill:none;stroke-linejoin:round;stroke-linecap:round}</style>',
            f'<rect width="{width}" height="{height}" rx="16" fill="#f8fafc"/>', text(35, 45, title, 25, '#0f172a')]


def world(local, root):
    x, y, yaw = root
    return [[x + math.cos(yaw)*p[0] - math.sin(yaw)*p[1],
             y + math.sin(yaw)*p[0] + math.cos(yaw)*p[1], yaw+p[2]] for p in local]


def polyline(points, convert, color, width=3, dash=''):
    coords = ' '.join(f'{x:.3f},{y:.3f}' for x, y in map(convert, points))
    return f'<polyline class="path" points="{coords}" stroke="{color}" stroke-width="{width}" stroke-dasharray="{dash}"/>'


def coordinates():
    record = json.loads((EVIDENCE/'prediction-01/raw.json').read_text())
    local = record['response']['data']['actions']['actions']
    root = record['capture']['pose']
    actual = world(local, root)
    image = begin('采集时的局部累计路径 → 固定世界路径')
    for x, title in [(30, '局部坐标：x前向、y左向'), (585, '世界坐标：使用采集时根位姿')]:
        image += [f'<rect x="{x}" y="90" width="485" height="340" rx="12" fill="white" stroke="#cbd5e1"/>', text(x+20, 125, title, 20)]
    left = lambda p: (95 + p[0]*245, 300-p[1]*245)
    right = lambda p: (650+(p[0]-5.45)*245, 335-(p[1]-15.20)*245)
    image += [line(95, 300, 455, 300), line(95, 365, 95, 175), text(450, 330, 'x / m'), text(105, 175, 'y / m')]
    image.append(polyline([[0,0,0]]+local, left, '#2563eb'))
    image += [line(650, 360, 1025, 360), line(650, 360, 650, 165), text(995, 390, 'X / m'), text(660, 170, 'Y / m')]
    image.append(polyline([root]+actual, right, '#2563eb'))
    for p, convert in [([0,0,0], left), (root, right)]:
        x,y=convert(p);image.append(f'<circle cx="{x}" cy="{y}" r="6" fill="#0f172a"/>')
    x,y=right(root);image.append(text(x-10,y-35,'根位姿',17,'#0f172a','end'))
    camera=record['capture']['camera_position'];cx,cy=right(camera)
    image += [f'<circle cx="{cx}" cy="{cy}" r="5" fill="white" stroke="#f97316" stroke-width="2"/>',text(745,400,'橙圈为相机；不能替代路径原点',16,'#c2410c')]
    first=actual[0]
    image += [text(35,475,'十点已经累计；下游只做一次刚体变换，不再次逐点积分。',20),
              text(35,510,f'首点世界坐标：({first[0]:.6f}, {first[1]:.6f}, {first[2]:.6f})',18), '</svg>']
    (ASSETS/'04-coordinates.svg').write_text('\n'.join(image))
    return first


def paths():
    rows=[json.loads(s) for s in (EVIDENCE/'actual-state.jsonl').read_text().splitlines() if s]
    installed=json.loads((EVIDENCE/'installed-paths.json').read_text())
    predictions=[]
    for directory in sorted(EVIDENCE.glob('prediction-*')):
        record=json.loads((directory/'raw.json').read_text())
        predictions.append([record['capture']['pose']]+world(record['response']['data']['actions']['actions'], record['capture']['pose']))
    image=begin('同一次历史任务：模型预测、安装参考、实际轨迹',1100,650)
    all_points=[p for route in predictions for p in route]+[r['pose'] for r in rows]
    xmin=math.floor(min(p[0] for p in all_points)*4)/4-.05
    xmax=math.ceil(max(p[0] for p in all_points)*4)/4+.05
    ymin=math.floor(min(p[1] for p in all_points)*4)/4-.05
    ymax=math.ceil(max(p[1] for p in all_points)*4)/4+.05
    scale=min(900/(xmax-xmin),360/(ymax-ymin))
    left=100+(900-(xmax-xmin)*scale)/2
    bottom=490
    convert=lambda p:(left+(p[0]-xmin)*scale,bottom-(p[1]-ymin)*scale)
    for i in range(math.ceil(xmin*2),math.floor(xmax*2)+1):
        X=i/2;x,_=convert([X,ymin]);image += [line(x,bottom-(ymax-ymin)*scale,x,bottom,'#e2e8f0',1),text(x,520,f'{X:.2f}',15,anchor='middle')]
    for i in range(math.ceil(ymin*4),math.floor(ymax*4)+1):
        Y=i/4;_,y=convert([xmin,Y]);image += [line(left,y,left+(xmax-xmin)*scale,y,'#e2e8f0',1),text(left-13,y+5,f'{Y:.2f}',15,anchor='end')]
    # Draw all real raw predictions; the rejected fifth remains a distinct red layer.
    for points in predictions[:-1]: image.append(polyline(points,convert,'#60a5fa',7,'12 7'))
    for item in installed:image.append(polyline(item['world_path'],convert,'#f59e0b',3,'5 5'))
    image.append(polyline(predictions[-1],convert,'#dc2626',3,'10 7'))
    image.append(polyline([r['pose'] for r in rows],convert,'#0f766e',4))
    sx,sy=convert(rows[0]['pose']);ex,ey=convert(rows[-1]['pose'])
    image += [f'<circle cx="{sx}" cy="{sy}" r="7" fill="#0f172a"/>', text(sx,sy-20,'起点',18,anchor='middle'),
              f'<circle cx="{ex}" cy="{ey}" r="7" fill="#0f766e"/>',text(ex+15,ey+35,'实际停止',18,'#0f766e')]
    for x,color,label,dash in [(40,'#60a5fa','模型预测','12 7'),(300,'#f59e0b','安装参考','5 5'),(560,'#0f766e','实际轨迹',''),(820,'#dc2626','第五条拒绝','10 7')]:
        image += [line(x,575,x+40,575,color,4,dash),text(x+50,581,label,18)]
    image += [text(40,620,'历史旧版无局部修复，已接受的模型路线与安装参考重合。X、Y单位：米。',18),'</svg>']
    (ASSETS/'06-paths.svg').write_text('\n'.join(image))


if __name__=='__main__':
    print('First world point:', coordinates())
    paths()
