#!/usr/bin/env python3
"""Build educational SlowFast GIFs/posters offline. No model, GPU or navigation.

Scene artwork comes from image_gen; all frame choices and feature arithmetic are
computed deterministically. Feature grids and RGB observations are illustrations.
"""
from __future__ import annotations
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps, ImageSequence

HERE = Path(__file__).resolve().parent
W,H,FPS,SECONDS=1200,675,10,16
BG='#f5f6f2'; INK='#193c40'; MUTED='#60767a'; EDGE='#dce4df'
COLORS={'anchor':'#687681','long':'#8665a9','mid':'#427eae','fast':'#087f7a','current':'#d27336'}
ORDER=['anchor','long','mid','fast','current']
FONT_PATHS=[Path('/System/Library/Fonts/Supplemental/Arial.ttf'),Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')]
BOLD_PATHS=[Path('/System/Library/Fonts/Supplemental/Arial Bold.ttf'),Path('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf')]
FONT_CACHE={}

def font(size,bold=False):
    key=(size,bold)
    if key not in FONT_CACHE:
        paths=BOLD_PATHS if bold else FONT_PATHS
        candidate=next((p for p in paths if p.is_file()),None)
        if candidate is None: raise RuntimeError('Install Arial or DejaVu Sans, or edit FONT_PATHS.')
        FONT_CACHE[key]=ImageFont.truetype(str(candidate),size)
    return FONT_CACHE[key]

def text(d,xy,value,size=20,color=INK,bold=False,anchor=None):
    d.text(xy,str(value),font=font(size,bold),fill=color,anchor=anchor)

def panel(d,box,fill='white',outline=EDGE,radius=14,width=1):
    d.rounded_rectangle(box,radius=radius,fill=fill,outline=outline,width=width)

def clamp(t):return min(1.,max(0.,t))
def fade(a,b,k):
    def rgb(c):return tuple(int(c[i:i+2],16) for i in (1,3,5))
    return tuple(round(x+(y-x)*clamp(k)) for x,y in zip(rgb(a),rgb(b)))

def header(d,eyebrow,title,subtitle):
    text(d,(40,21),eyebrow,14,MUTED,True)
    text(d,(40,45),title,31,INK,True)
    text(d,(40,88),subtitle,19,MUTED)

def picture(im,img,box):
    x,y,w,h=box
    # Only fit/crop generated artwork for the instructional composition.
    fitted=ImageOps.fit(img,(w,h),Image.Resampling.LANCZOS)
    mask=Image.new('L',(w,h));ImageDraw.Draw(mask).rounded_rectangle((0,0,w-1,h-1),radius=10,fill=255)
    im.paste(fitted,(x,y),mask)


def load_selector():
    path=HERE/'slowfast_reference.py'
    spec=importlib.util.spec_from_file_location('m07_slowfast_reference',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def example_data():
    cfg=json.loads((HERE/'example-config.json').read_text())
    source=HERE/'slowfast_reference.py'
    assert hashlib.sha256(source.read_bytes()).hexdigest()==cfg['slowfast_reference_sha256']
    selector=load_selector();tiers=selector.validate_slowfast_tiers(cfg['slowfast_tiers'])
    segs=selector.slowfast_segments(239,240,tiers)
    rows=[dict(name=tiers[s['tier']]['name'],frame_ids=s['frame_ids'],pool_spatial=s['pool_spatial']) for s in segs]
    index={i:r['name'] for r in rows for i in r['frame_ids']}
    counts={r['name']:len(r['frame_ids']) for r in rows}
    assert counts=={'anchor':2,'long':26,'mid':20,'fast':32,'current':2}
    return cfg,selector,tiers,rows,index,counts


def temporal(sec,scenes,index,counts,poster=False):
    im=Image.new('RGB',(W,H),BG);d=ImageDraw.Draw(im)
    header(d,'01 / TEMPORAL SELECTION','Time: choose which observations to use',
           '240 captured frames at 4 fps  |  Current frame: 239  |  About 60 seconds')
    labels=[('0','Start: fridge hidden'),('80','Earlier doorway view'),('160','Fridge in sight'),('239','Current view')]
    for j,(frame,label) in enumerate(labels):
        x=40+j*283;picture(im,scenes[j],(x,125,271,125))
        text(d,(x,258),'Frame '+frame+'  /  '+label,16,INK,True)
    tiers_shown=5 if poster or sec>=10 else max(0,min(5,int((sec-5)/1.)+1))
    reveal=['current','fast','mid','long','anchor'][:tiers_shown]
    if sec<2 and not poster:phase='Stored buffer: all 240 frames are available'
    elif sec<5 and not poster:phase='Frame age = 239 - frame index; age 0 is the newest'
    elif sec<10 and not poster:phase='Build this prediction: select '+(reveal[-1] if reveal else 'current')+' observations'
    else:phase='82 selected for this prediction; all 240 remain stored'
    text(d,(40,301),phase,22,INK,True)
    # All 240 cells stay present. Their locations are absolute frame indices.
    for f in range(240):
        row,col=divmod(f,60);x=76+col*18;y=341+row*32
        tier=index.get(f)
        age=239-f
        if f<2:base='anchor'
        elif age<=1:base='current'
        elif age<=33:base='fast'
        elif age<=89:base='mid'
        else:base='long'
        if sec<2 and not poster:fill='#d8e5e2';outline='#a9bdb9'
        elif sec<5 and not poster:fill=fade('#e6ece7',COLORS[base],.35);outline=COLORS[base]
        elif tier in reveal:fill=COLORS[tier];outline=COLORS[tier]
        else:fill='#edf0ec';outline='#d2dad4'
        d.rounded_rectangle((x,y,x+14,y+23),radius=3,fill=fill,outline=outline)
    for row in range(4):text(d,(40,346+row*32),f'{row*60:03}',13,MUTED)
    if 2<=sec<5 and not poster:
        k=round(clamp((sec-2)/3)*239);row,col=divmod(k,60);x=76+col*18;y=341+row*32
        d.rounded_rectangle((x-3,y-3,x+17,y+26),radius=5,outline=INK,width=3)
    for j,name in enumerate(ORDER):
        x=40+j*227;panel(d,(x,485,x+214,523),fill='white')
        d.ellipse((x+12,498,x+23,509),fill=COLORS[name])
        text(d,(x+32,494),name.capitalize()+': '+str(counts[name]),17,INK,True)
    panel(d,(40,541,1160,641),fill='#eaf0e9')
    if poster or sec>=10:
        text(d,(57,553),'MID CLOSE-UP: keep 2 adjacent frames, skip 4, repeat',18,INK,True)
        for k,age in enumerate(range(34,48)):
            keep=(age-34)%6 in [0,1];x=60+k*77
            d.rounded_rectangle((x,584,x+65,615),radius=5,fill=COLORS['mid'] if keep else '#dce5dc')
            text(d,(x+32,589),str(age),17,'white' if keep else MUTED,True,anchor='ma')
        text(d,(60,621),'Frame age',12,MUTED)
    else:
        text(d,(58,559),'Colored = selected     Outlined = not selected for this prediction',22,INK,True)
        text(d,(58,596),'The unselected observations are still in the stored history buffer.',19,MUTED)
    text(d,(40,655),'Illustrated observations; exact selection follows the fixed checkpoint rules.',13,MUTED)
    return im


FEATURES=np.array([[1+((r*3+c*2)%9) for c in range(8)] for r in range(8)],dtype=float)

def avg_pool(grid,factor):
    n=grid.shape[0];assert n%factor==0
    return grid.reshape(n//factor,factor,n//factor,factor).mean(axis=(1,3))


def grid(im,xy,size,values,reveal=1.,highlight=None):
    d=ImageDraw.Draw(im);x,y=xy;n=len(values);cell=size/n
    for r in range(n):
        for c in range(n):
            shown=(r*n+c)<round(reveal*n*n)
            color=fade('#dfe7e2','#147f7b',(values[r,c]-1)/8) if shown else '#e8ede7'
            d.rectangle((round(x+c*cell)+1,round(y+r*cell)+1,round(x+(c+1)*cell)-2,round(y+(r+1)*cell)-2),fill=color)
    d.rectangle((x,y,x+size,y+size),outline=INK,width=1)
    if highlight:
        r,c,k=highlight;d.rectangle((x+c*cell,y+r*cell,x+(c+k)*cell,y+(r+k)*cell),outline='#d27336',width=4)


def spatial(sec,scenes,poster=False):
    im=Image.new('RGB',(W,H),BG);d=ImageDraw.Draw(im)
    header(d,'02 / SPATIAL POOLING','Space: keep fewer feature locations',
           'After vision encoding  |  A toy 8 x 8 feature grid  |  Original RGB stays unchanged')
    picture(im,scenes[2],(40,128,240,135))
    text(d,(40,269),'Illustrated RGB observation',15,MUTED)
    text(d,(312,145),'The vision encoder produces feature vectors.',22,INK,True)
    text(d,(312,182),'Pooling averages nearby feature locations.',21,INK)
    text(d,(312,218),'Each square below represents one vector.',18,MUTED)
    # A one-channel arithmetic example, separate from the semantic artwork.
    panel(d,(923,127,1160,270),fill='white')
    text(d,(938,137),'One channel / mean',16,MUTED,True)
    vals=[[1,3],[5,7]]
    for r in range(2):
        for c in range(2):
            x=939+c*35;y=166+r*35;d.rectangle((x,y,x+33,y+33),fill='#dcebe5',outline='white')
            text(d,(x+16,y+6),vals[r][c],18,INK,True,anchor='ma')
    text(d,(1030,181),'= 4',28,INK,True)
    text(d,(938,245),'(1 + 3 + 5 + 7) / 4',14,MUTED)
    pools=[FEATURES,avg_pool(FEATURES,2),avg_pool(FEATURES,4)]
    centers=[180,598,1016]
    for j,(factor,values,cx) in enumerate(zip([1,2,4],pools,centers)):
        panel(d,(cx-140,302,cx+140,577),fill='white')
        text(d,(cx,313),'s = '+str(factor),23,INK,True,anchor='ma')
        reveal=1. if poster else (clamp((sec-2)/2) if j==0 else clamp((sec-(4 if j==1 else 8))/3))
        highlight=None
        if not poster and j==0 and 4<=sec<7:
            k=int((sec-4)/3*16)%16;highlight=(k//4*2,k%4*2,2)
        if not poster and j==0 and 8<=sec<11:
            k=int((sec-8)/3*4)%4;highlight=(k//2*4,k%2*4,4)
        grid(im,(cx-96,352),192,values,reveal,highlight)
        n=len(values);text(d,(cx,552),f'{n} x {n} = {n*n} locations',18,INK,True,anchor='ma')
    if sec<4 and not poster:phase='First encode the selected images into spatial features.'
    elif sec<8 and not poster:phase='s = 2: average 2 x 2 regions of the ORIGINAL 8 x 8 grid.'
    elif sec<12 and not poster:phase='s = 4: average 4 x 4 regions of the SAME original grid.'
    else:phase='64 -> 16 -> 4 locations: less spatial detail, fewer visual tokens.'
    text(d,(40,598),phase,21,INK,True)
    text(d,(40,630),'Ratios are relative to s = 1. Colors show toy values, not learned model activations.',16,MUTED)
    text(d,(40,655),'Illustration only; this is not JPEG downsampling or the measured model feature shape.',13,MUTED)
    return im


def combined(scenes,rows,counts):
    im=Image.new('RGB',(W,H),BG);d=ImageDraw.Draw(im)
    header(d,'03 / PUT THE TWO IDEAS TOGETHER','Fewer observations. Coarser older features.',
           '240 stored frames -> 82 selected frames -> vision encoding -> tier-specific pooling')
    for j,name in enumerate(ORDER):
        x=40+j*227;row=next(r for r in rows if r['name']==name);s=row['pool_spatial']
        panel(d,(x,137,x+214,565),fill='white',outline=COLORS[name],width=2)
        text(d,(x+107,150),name.capitalize(),22,COLORS[name],True,anchor='ma')
        scene=scenes[[0,1,2,3,3][j]];picture(im,scene,(x+12,187,190,107))
        text(d,(x+107,307),str(counts[name])+' selected frames',18,INK,True,anchor='ma')
        text(d,(x+107,339),'Pool factor s = '+str(s),18,MUTED,anchor='ma')
        grid(im,(x+43,378),128,avg_pool(FEATURES,s))
        n=8//s;text(d,(x+107,520),f'Toy grid: {n} x {n}',17,INK,True,anchor='ma')
    text(d,(40,587),'Time selection chooses images. Spatial pooling reduces encoded feature locations.',21,INK,True)
    text(d,(40,622),'Each toy grid represents one encoded time block; the pictures are teaching illustrations.',16,MUTED)
    text(d,(40,653),'Stored RGB is retained. Counts come from the fixed selector. Grid sizes are illustrative.',13,MUTED)
    return im


def encode_gif(frames,path):
    # A shared palette avoids flashing colors between GIF frames.
    palette=frames[-1].quantize(colors=192,method=Image.Quantize.MEDIANCUT)
    quantized=[f.quantize(palette=palette,dither=Image.Dither.NONE) for f in frames]
    quantized[0].save(path,save_all=True,append_images=quantized[1:],duration=100,loop=0,optimize=True,disposal=1)
    with Image.open(path) as gif:
        duration=sum(f.info.get('duration',0) for f in ImageSequence.Iterator(gif))
        assert duration==SECONDS*1000,(path,duration)
    return duration


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--assets',type=Path,required=True);args=ap.parse_args()
    assets=args.assets;assets.mkdir(parents=True,exist_ok=True)
    cfg,selector,tiers,rows,index,counts=example_data()
    with Image.open(assets/'scene-atlas.jpg') as atlas:
        scenes=[atlas.crop(box).convert('RGB') for box in cfg['scene_crops']]
    # Export exact selection data separately from the illustrative scenes.
    data={'history_frames':240,'latest_frame':239,'fps':4,'first_to_last_span_s':239/4,'selected_frames':len(index),'unselected_frames':240-len(index),'counts':counts,'segments':rows,'selected_frame_ids':sorted(index),'teaching_only':True,'timestamp_relative':cfg['timestamp_relative']}
    (assets/'selection-example.json').write_text(json.dumps(data,indent=2))
    outputs=[]
    for name,fn in [('time-selection',lambda t:temporal(t,scenes,index,counts)),('spatial-pooling',lambda t:spatial(t,scenes))]:
        frames=[fn(i/FPS) for i in range(FPS*SECONDS)]
        path=assets/(name+'.gif');duration=encode_gif(frames,path)
        fn_poster=temporal(15,scenes,index,counts,True) if name=='time-selection' else spatial(15,scenes,True)
        fn_poster.save(assets/(name+'.png'),optimize=True)
        outputs.append({'file':path.name,'size':path.stat().st_size,'dimensions':[W,H],'duration_ms':duration,'sampling_fps':FPS})
    combined(scenes,rows,counts).save(assets/'combined-example.png',optimize=True)
    (assets/'animation-manifest.json').write_text(json.dumps({'outputs':outputs,'feature_grid':'8x8 toy encoded block, not actual activations','source_config':'example-config.json','scene_source':'built-in image_gen; see scene-prompt.json','renderer':'build_visuals.py; numeric layout drawn deterministically','not_runtime_evidence':True},indent=2))
    print(json.dumps(outputs,indent=2))

if __name__=='__main__':main()
