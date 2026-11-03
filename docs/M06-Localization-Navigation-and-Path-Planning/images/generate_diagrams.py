#!/usr/bin/python3
"""Generate editable draw.io, SVG and PNG course diagrams from the same geometry.

Uses Pillow for local PNG export when draw.io desktop is unavailable.
All diagrams are authored course illustrations, never measured experiment data.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
COLORS = {'blue': ('#e7effa', '#4674a5'), 'teal': ('#e4f3ef', '#25856c'),
          'amber': ('#fff2d9', '#bb8430'), 'gray': ('#eef1f5', '#647487')}


def diagram(name, title, subtitle, nodes, edges, note, width=1240, height=590):
    scale = 2
    img = Image.new('RGB', (width*scale, height*scale), '#ffffff')
    draw = ImageDraw.Draw(img)
    svg = ET.Element('svg', xmlns='http://www.w3.org/2000/svg', width=str(width), height=str(height), viewBox=f'0 0 {width} {height}')
    ET.SubElement(svg, 'rect', width='100%', height='100%', fill='#ffffff')
    defs = ET.SubElement(svg, 'defs')
    marker = ET.SubElement(defs, 'marker', id='arrow', markerWidth='9', markerHeight='9', refX='8', refY='4', orient='auto', markerUnits='userSpaceOnUse')
    ET.SubElement(marker, 'path', d='M0,0 L8,4 L0,8 z', fill='#65758a')
    mx = ET.Element('mxfile', host='drawio', version='26.0.0')
    graph = ET.SubElement(ET.SubElement(mx, 'diagram', name=title), 'mxGraphModel', pageWidth=str(width), pageHeight=str(height))
    cells = ET.SubElement(graph, 'root'); ET.SubElement(cells,'mxCell',id='0');ET.SubElement(cells,'mxCell',id='1',parent='0')
    def label(text, x, y, size=18, color='#203247', center=False):
        font = ImageFont.truetype(FONT, size*scale)
        draw.text((int(x*scale),int(y*scale)),text,font=font,fill=color,anchor='mm' if center else 'lt')
        e=ET.SubElement(svg,'text',x=str(x),y=str(y if center else y+size),fill=color,**{'font-family':'DejaVu Sans, sans-serif','font-size':str(size),'text-anchor':'middle' if center else 'start'})
        if center:e.set('dominant-baseline','middle')
        e.text=text
    label(title, 34, 24, 27)
    label(subtitle, 34, 65, 15, '#65758a')
    index={n[0]:n for n in nodes}
    for j,edge in enumerate(edges):
        source,target,text,*route=edge
        a,b=index[source],index[target]
        if route:
            points=route[0]
        else:
            ax,ay,aw,ah=a[2:6]; bx,by,bw,bh=b[2:6]
            if abs((ax+aw/2)-(bx+bw/2)) < 10:
                points=[(ax+aw/2,ay+ah),(bx+bw/2,by)] if by>ay else [(ax+aw/2,ay),(bx+bw/2,by+bh)]
            else:
                points=[(ax+aw,ay+ah/2),(bx,by+bh/2)] if bx>ax else [(ax,ay+ah/2),(bx+bw,by+bh/2)]
        draw.line([(int(x*scale),int(y*scale)) for x,y in points],fill='#65758a',width=3*scale)
        x,y=points[-1];px,py=points[-2];angle=math.atan2(y-py,x-px)
        tip=[(x,y),(x-11*math.cos(angle-.45),y-11*math.sin(angle-.45)),(x-11*math.cos(angle+.45),y-11*math.sin(angle+.45))]
        draw.polygon([(int(x*scale),int(y*scale)) for x,y in tip],fill='#65758a')
        ET.SubElement(svg,'polyline',points=' '.join(f'{x},{y}' for x,y in points),fill='none',stroke='#65758a',**{'stroke-width':'3','marker-end':'url(#arrow)'})
        cell=ET.SubElement(cells,'mxCell',id=f'e{j}',value=text,source=source,target=target,parent='1',edge='1',style='edgeStyle=orthogonalEdgeStyle;rounded=0;html=1;endArrow=block;strokeColor=#65758a;fontSize=12;labelBackgroundColor=#ffffff;')
        geom=ET.SubElement(cell,'mxGeometry',relative='1',**{'as':'geometry'})
        if len(points)>2:
            arr=ET.SubElement(geom,'Array',**{'as':'points'})
            for x,y in points[1:-1]:ET.SubElement(arr,'mxPoint',x=str(x),y=str(y))
        if text:
            mid=points[len(points)//2] if len(points)>2 else ((points[0][0]+points[1][0])/2,(points[0][1]+points[1][1])/2)
            label(text,mid[0],mid[1]-14,12,'#65758a',True)
    for key,lines,x,y,w,h,color in nodes:
        fill,stroke=COLORS[color]
        draw.rounded_rectangle((x*scale,y*scale,(x+w)*scale,(y+h)*scale),radius=12*scale,fill=fill,outline=stroke,width=2*scale)
        ET.SubElement(svg,'rect',x=str(x),y=str(y),width=str(w),height=str(h),rx='12',fill=fill,stroke=stroke,**{'stroke-width':'2'})
        for i,line in enumerate(lines):label(line,x+w/2,y+h/2+(i-(len(lines)-1)/2)*25,17 if i==0 else 14,center=True)
        cell=ET.SubElement(cells,'mxCell',id=key,value='\n'.join(lines),parent='1',vertex='1',style=f'rounded=1;whiteSpace=wrap;html=1;fillColor={fill};strokeColor={stroke};fontSize=16;fontColor=#203247;')
        ET.SubElement(cell,'mxGeometry',x=str(x),y=str(y),width=str(w),height=str(h),**{'as':'geometry'})
    label(note,34,height-36,14,'#65758a')
    for label_text,x,y in [(title,34,24),(subtitle,34,65),(note,34,height-36)]:
        cell=ET.SubElement(cells,'mxCell',id=f'text-{y}',value=label_text,parent='1',vertex='1',style='text;html=1;align=left;verticalAlign=middle;whiteSpace=wrap;strokeColor=none;fillColor=none;fontSize=18;')
        ET.SubElement(cell,'mxGeometry',x=str(x),y=str(y),width=str(width-70),height='30',**{'as':'geometry'})
    ET.indent(mx);ET.ElementTree(mx).write(ROOT/f'{name}.drawio',encoding='utf-8',xml_declaration=True)
    ET.ElementTree(svg).write(ROOT/f'{name}.svg',encoding='utf-8',xml_declaration=True)
    img.save(ROOT/f'{name}.png')


def main():
    diagram('navigation_architecture','M6 | From sensors to navigation','M3 sensing / state estimation + M5 mapping -> M6 decisions',[
        ('s',['3D LiDAR','PointCloud2'],35,140,200,80,'blue'),
        ('p',['LiDAR processing','scan + obstacle points'],300,140,220,80,'teal'),
        ('l',['SLAM / AMCL','map -> odom'],590,140,220,80,'blue'),
        ('b',['BT Navigator','goal / recovery'],915,140,270,80,'amber'),
        ('w',['Wheel + IMU','EKF -> odom'],35,340,200,80,'blue'),
        ('c',['Costmaps','static / live / inflation'],300,340,220,80,'teal'),
        ('g',['Planner Server','A* / Dijkstra / Theta*'],590,340,220,80,'amber'),
        ('v',['Controller Server','DWB / MPPI / RPP'],915,340,270,80,'amber')],
        [('s','p',''),('p','l',''),('p','c',''),('w','l','TF',[(235,365),(265,365),(265,255),(700,255),(700,220)]),
         ('c','g',''),('c','v','live costs',[(450,420),(450,465),(875,465),(875,400),(915,400)]),('g','v','path'),('b','g','goal',[(1040,220),(1040,275),(700,275),(700,340)]),
         ('b','v',''),('v','w','cmd_vel / feedback',[(1050,420),(1050,490),(135,490),(135,420)])],
         'SLAM and AMCL run in separate phases. Simulator ground truth is evaluation-only.')
    diagram('mapping_workflow','6.1 | Map -> save -> relocalize -> navigate','Keep mapping and localization mutually exclusive',[
        ('a',['Start mapping','run_mapping.sh'],35,145,230,80,'blue'),
        ('b',['Survey / loop closure','live scan + EKF'],340,145,230,80,'teal'),
        ('c',['Save observed map','pgm + yaml + pose graph'],650,145,240,80,'blue'),
        ('d',['Stop mapping','Ctrl+C terminal A'],965,145,235,80,'gray'),
        ('h',['Navigate','Nav2 active'],35,355,230,85,'amber'),
        ('g',['Check localization','5 AMCL updates + scan fit'],340,355,250,85,'teal'),
        ('f',['Seed / rotate','relocalize --pose ...'],650,355,240,85,'blue'),
        ('e',['Reload saved map','run_navigation.sh'],965,355,235,85,'blue')],
        [('a','b',''),('b','c',''),('c','d',''),('d','e',''),('e','f',''),('f','g',''),('g','h','pass')],
        'If localization fails: inspect map / TF / scan / seed; keep navigation inactive.')
    diagram('planning_loop','6.2 | Global route and local control','A global path describes a route; a controller repeatedly chooses velocities',[
        ('a',['Goal + saved map','known walls / shelves'],40,135,240,85,'blue'),
        ('b',['Global search','A* / Dijkstra / Theta*'],370,135,250,85,'amber'),
        ('c',['Reference path','map coordinates'],755,135,250,85,'teal'),
        ('f',['Robot motion','cmd_vel -> chassis'],40,350,240,85,'blue'),
        ('e',['Local controller','sample / score / execute'],370,350,250,85,'amber'),
        ('d',['Live local costmap','new LiDAR obstacles'],755,350,250,85,'teal')],
        [('a','b',''),('b','c',''),('c','e','reference',[(880,220),(880,280),(500,280),(500,350)]),('d','e',''),('e','f',''),
         ('f','d','new sensor observations',[(160,435),(160,490),(880,490),(880,435)])],
        'In this lab, the global map stays static. Local detection handles the cone added after mapping.',width=1100)
    diagram('lidar_detection','6.3 | LiDAR obstacle detection pipeline','Geometric detection is implemented; persistent tracking / prediction are extensions',[
        ('a',['Timestamped cloud','sensor frame -> base'],35,140,200,80,'blue'),
        ('b',['ROI + voxel grid','invalid / self / range'],300,140,220,80,'teal'),
        ('c',['Ground RANSAC','near-horizontal plane'],590,140,220,80,'teal'),
        ('e',['Obstacle cloud','non-ground points'],915,140,270,80,'teal'),
        ('d',['Height-slice scan','SLAM / AMCL / clearing'],300,350,220,85,'blue'),
        ('k',['Local costmap','mark + clear + inflate'],590,350,220,85,'amber'),
        ('f',['3D cluster boxes','Euclidean KD-tree'],915,350,270,85,'amber')],
        [('a','b',''),('b','c',''),('c','e',''),('e','f','clusters'),('b','d','projection'),
         ('d','k','clear'),('e','k','mark',[(985,220),(985,275),(700,275),(700,350)])],
        'All non-ground points mark the local costmap; cluster-size thresholds affect boxes only.')
    diagram('multi_robot','6.4 | Multi-robot communication and coordination','Design exercise: no completed dual-J501 formation experiment is claimed',[
        ('a',['Shared map / mission','consistent origin + frames'],435,130,330,80,'blue'),
        ('b',['Leader: robot1','localization + Nav2'],40,315,300,100,'teal'),
        ('c',['Coordinator','relative offset -> goal'],435,315,330,100,'amber'),
        ('d',['Follower: robot2','localization + Nav2'],860,315,300,100,'teal')],
        [('a','b','map',[(435,170),(190,170),(190,315)]),('a','d','map',[(765,170),(1010,170),(1010,315)]),
         ('b','c','leader pose'),('c','d','NavigateToPose'),
         ('d','c','status / watchdog',[(1010,415),(1010,475),(600,475),(600,415)])],
        'Discovery Server discovers peers; data still needs network reachability. Each robot owns its stop logic.')

    # Scientific illustrations: formulas, not measured experimental output.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.patches import Circle, FancyArrowPatch
    fig,axes=plt.subplots(1,2,figsize=(11,4.4))
    d=np.linspace(0,1.2,500);r=.27
    for k in [1.,3.,6.]:
        costs=np.where(d<=r,253,252*np.exp(-k*(d-r)));costs[d>.65]=0
        axes[0].plot(d,costs,label=f'k = {k:g}')
    axes[0].set(xlabel='Distance from obstacle (m)',ylabel='Inflation cost',title='Inflation radius = 0.65 m (illustration)');axes[0].legend()
    t=np.linspace(0,2,50)
    for omega in [-.55,-.25,0.,.25,.55]:
        x=.25*t if omega==0 else .25/omega*np.sin(omega*t)
        y=np.zeros_like(t) if omega==0 else .25/omega*(1-np.cos(omega*t))
        axes[1].plot(x,y,label=f'w={omega:g}')
    axes[1].add_patch(Circle((.42,.02),.065,color='#e68a31',label='Obstacle'))
    axes[1].set(xlabel='Robot forward X (m)',ylabel='Robot left Y (m)',aspect='equal',title='Sampled local trajectories (illustration)');axes[1].legend(fontsize=8)
    fig.tight_layout();fig.savefig(ROOT/'planning_principles.png',dpi=170);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,4.5))
    theta=.65;l=np.array([2.5,1.8]);R=np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
    offset=np.array([-1.2,0.]);f=l+R@offset
    for p,col,label in [(l,'#25856c','Leader'),(f,'#4674a5','Follower target')]:
        ax.scatter(*p,s=180,color=col,zorder=3);ax.annotate(label,p,xytext=(8,15),textcoords='offset points')
        v=R@np.array([.55,0]);ax.arrow(*p,*v,width=.025,color=col,length_includes_head=True)
    ax.plot([l[0],f[0]],[l[1],f[1]],'--',color='#bb8430',label='Desired offset: -d in leader frame')
    ax.set(xlim=(.7,3.5),ylim=(.2,3),aspect='equal',xlabel='Shared map X (m)',ylabel='Shared map Y (m)',title='Leader-Follower geometry (design illustration)');ax.legend(loc='upper left');ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(ROOT/'formation_geometry.png',dpi=170);plt.close(fig)


if __name__=='__main__':main()
