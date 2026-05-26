#!/usr/bin/env python3
# Visual proof for the qcom-rotator: push a recognizable image through the
# hardware for each transform, convert the device output back to RGB and tile
# everything into one annotated grid.
import sys, os, re
import numpy as np
from PIL import Image, ImageDraw, ImageFont

SRC="/tmp/ro/vsrc"; OUT="/tmp/ro/vout"
W,H=640,480

# name, fmt, rot, hf, vf, crop|None
CASES=[
    ("90 CW",       "nv12",  90,0,0,None),
    ("180",         "nv12", 180,0,0,None),
    ("270 CW",      "nv12", 270,0,0,None),
    ("hflip",       "nv12",   0,1,0,None),
    ("vflip",       "nv12",   0,0,1,None),
    ("crop",        "nv12",   0,0,0,(160,128,320,240)),
    ("crop+90",     "nv12",  90,0,0,(160,128,256,192)),
    ("RGB565 90",   "rgb565",90,0,0,None),
    ("ARGB32 180",  "argb32",180,0,0,None),
    ("NV16 90",     "nv16",  90,0,0,None),
]

def make_rgb():
    im=Image.new("RGB",(W,H)); d=ImageDraw.Draw(im)
    d.rectangle([0,0,W//2,H//2],fill=(200,40,40))      # TL red
    d.rectangle([W//2,0,W,H//2],fill=(40,170,40))      # TR green
    d.rectangle([0,H//2,W//2,H],fill=(40,60,200))      # BL blue
    d.rectangle([W//2,H//2,W,H],fill=(210,190,40))     # BR yellow
    # big white upward arrow (unambiguous orientation marker)
    cx=W//2
    d.polygon([(cx,90),(cx-80,230),(cx-30,230),(cx-30,400),
               (cx+30,400),(cx+30,230),(cx+80,230)],fill=(255,255,255))
    d.ellipse([20,20,80,80],fill=(0,0,0))              # black disc = TOP-LEFT origin
    return im

# --- format conversions (full-range BT.601) ---
def rgb2yuv(a):
    a=a.astype(np.float32)
    R,G,B=a[...,0],a[...,1],a[...,2]
    Y=0.299*R+0.587*G+0.114*B
    U=-0.169*R-0.331*G+0.5*B+128
    V=0.5*R-0.419*G-0.081*B+128
    return np.clip(np.stack([Y,U,V],-1),0,255).astype(np.uint8)

def yuv2rgb(Y,U,V):
    Y=Y.astype(np.float32);U=U.astype(np.float32)-128;V=V.astype(np.float32)-128
    R=Y+1.402*V; G=Y-0.344*U-0.714*V; B=Y+1.772*U
    return np.clip(np.stack([R,G,B],-1),0,255).astype(np.uint8)

def to_raw(fmt,rgb):
    a=np.asarray(rgb)
    if fmt in ("nv12","nv16"):
        yuv=rgb2yuv(a); Y=yuv[...,0]
        if fmt=="nv12":
            U=yuv[0::2,0::2,1]; V=yuv[0::2,0::2,2]      # 4:2:0 (decimate)
            chh=H//2
        else:
            U=yuv[:,0::2,1]; V=yuv[:,0::2,2]            # 4:2:2
            chh=H
        inter=np.empty((chh,W),np.uint8); inter[:,0::2]=U; inter[:,1::2]=V
        return Y.tobytes()+inter.tobytes()
    if fmt=="rgb565":
        R,G,B=a[...,0].astype(np.uint16),a[...,1].astype(np.uint16),a[...,2].astype(np.uint16)
        return (((R>>3)<<11)|((G>>2)<<5)|(B>>3)).astype('<u2').tobytes()
    if fmt=="argb32":
        h,w,_=a.shape
        return np.dstack([a[...,2],a[...,1],a[...,0],np.full((h,w),255,np.uint8)]).astype(np.uint8).tobytes()

def from_raw(fmt,w,h,buf):
    if fmt in ("nv12","nv16"):
        Y=np.frombuffer(buf[:w*h],np.uint8).reshape(h,w)
        chh=h//2 if fmt=="nv12" else h
        inter=np.frombuffer(buf[w*h:w*h+w*chh],np.uint8).reshape(chh,w)
        U=inter[:,0::2]; V=inter[:,1::2]
        U=np.repeat(U,2,axis=1)[:, :w]; V=np.repeat(V,2,axis=1)[:, :w]
        if fmt=="nv12": U=np.repeat(U,2,axis=0)[:h]; V=np.repeat(V,2,axis=0)[:h]
        return yuv2rgb(Y,U,V)
    if fmt=="rgb565":
        v=np.frombuffer(buf[:w*h*2],'<u2').reshape(h,w).astype(np.uint16)
        R=((v>>11)&0x1f)<<3; G=((v>>5)&0x3f)<<2; B=(v&0x1f)<<3
        return np.stack([R,G,B],-1).astype(np.uint8)
    if fmt=="argb32":
        a=np.frombuffer(buf[:w*h*4],np.uint8).reshape(h,w,4)
        return np.stack([a[...,2],a[...,1],a[...,0]],-1)

def gen():
    os.makedirs(SRC,exist_ok=True); os.makedirs(OUT,exist_ok=True)
    rgb=make_rgb(); rgb.save(f"{SRC}/source.png")
    for fmt in ("nv12","nv16","rgb565","argb32"):
        open(f"{SRC}/{fmt}.raw","wb").write(to_raw(fmt,rgb))
    print("gen ok")

def cmds():
    for name,fmt,rot,hf,vf,crop in CASES:
        tag=name.replace(" ","_").replace("+","")
        a=[tag, f"{SRC}/{fmt}.raw", f"{OUT}/{tag}.out", fmt,str(W),str(H),
           str(rot),str(hf),str(vf)]
        if crop: a+= [str(crop[0]),str(crop[1]),str(crop[2]),str(crop[3])]
        print(" ".join(a))

def render(logfile):
    log=open(logfile).read()
    src=Image.open(f"{SRC}/source.png")
    BOX=300  # thumbnails fit inside a BOX x BOX square (handles portrait rotations)
    def thumb(im):
        s=min(BOX/im.width, BOX/im.height)
        return im.resize((int(im.width*s),int(im.height*s)))
    panels=[("SOURCE 640x480",thumb(src))]
    for name,fmt,rot,hf,vf,crop in CASES:
        tag=name.replace(" ","_").replace("+","")
        blk=re.search(rf"CASE {tag}\n(.*?)(?=\nCASE |\Z)",log,re.S)
        body=blk.group(1) if blk else ""
        cap=re.search(r"CAP w=(\d+) h=(\d+)",body)
        st=re.search(r"RESULT status=(\w+)",body)
        if not cap or not st or st.group(1)!="OK":
            panels.append((f"{name} [{fmt}] FAIL",None)); continue
        ow,oh=int(cap.group(1)),int(cap.group(2))
        buf=open(f"{OUT}/{tag}.out","rb").read()
        rgb=Image.fromarray(from_raw(fmt,ow,oh,buf))
        panels.append((f"{name}  [{fmt}]  {ow}x{oh}",thumb(rgb)))
    # compose grid: 4 columns, uniform square cells (no portrait overflow)
    cols=4; pad=12; lab=20
    cell_w=BOX+pad; cell_h=BOX+lab+pad
    rows=(len(panels)+cols-1)//cols
    canvas=Image.new("RGB",(cols*cell_w+pad, rows*cell_h+pad+30),(245,245,245))
    d=ImageDraw.Draw(canvas)
    try: font=ImageFont.load_default()
    except Exception: font=None
    d.text((pad,8),"qcom-rotator HARDWARE output - one recognizable image through each transform "
                   "(white arrow = up, black disc = top-left origin)",fill=(0,0,0),font=font)
    for i,(label,im) in enumerate(panels):
        r,c=divmod(i,cols)
        x=pad+c*cell_w; y=30+pad+r*cell_h
        d.text((x,y),label,fill=(0,0,0),font=font)
        if im is not None:
            canvas.paste(im,(x,y+lab))
        else:
            d.rectangle([x,y+lab,x+BOX,y+lab+BOX],outline=(200,0,0),width=2)
    out=f"{OUT}/rotator_proof_grid.png"; canvas.save(out)
    print("wrote",out, canvas.size)

if __name__=="__main__":
    c=sys.argv[1] if len(sys.argv)>1 else "gen"
    {"gen":gen,"cmds":cmds}.get(c,lambda:render(sys.argv[2]))() if c!="render" else render(sys.argv[2])
