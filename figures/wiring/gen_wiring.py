"""Generates so101-wiring.svg: an isometric wiring diagram of the SO-101 follower bus.

Run:  python3 gen_wiring.py [output.svg]      (standard library only)
      python3 gen_wiring.py out.svg --render render.png   (labels over build_scene.py's render)

How it is built
- Everything is placed in millimetres in a 3D scene and projected by P(x, y, z).
  +x runs down-left on screen, +y runs down-right, +z is up. S is pixels per mm.
- Shapes are painted back to front (painter's algorithm), so the ORDER of the
  sections below matters: a part drawn later covers a part drawn earlier.
- box() takes three fills: top, the +y face, the +x face. Colours are plain hex
  strings next to each part; the wire colours are DATA / PWR / GND near the top.
- Each section is wrapped in a named <g> (see grp()), so the SVG can also be
  restyled directly in Figma, Illustrator or Inkscape.
See README.md for where every dimension came from.
"""
import math
S=2.9; C=math.cos(math.radians(30)); SN=0.5
OX,OY=1005,470
def P(x,y,z=0): return (OX-(x-y)*C*S, OY+((x+y)*SN-z)*S)
def pts(l): return " ".join(f"{a:.1f},{b:.1f}" for a,b in l)
out=[]
_open=[False]
def grp(name):
    """Start a named <g> so each part of the drawing can be selected and restyled on its own."""
    if _open[0]: out.append("</g>")
    out.append(f'<g id="{name}">'); _open[0]=True
def poly(l,fill,stroke="#1c1b19",sw=1.2,extra=""):
    out.append(f'<polygon points="{pts(l)}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" stroke-linejoin="round" {extra}/>')
def box(x,y,z,dx,dy,dz,top,left,right,**k):
    poly([P(x,y+dy,z),P(x+dx,y+dy,z),P(x+dx,y+dy,z+dz),P(x,y+dy,z+dz)],left,**k)
    poly([P(x+dx,y,z),P(x+dx,y+dy,z),P(x+dx,y+dy,z+dz),P(x+dx,y,z+dz)],right,**k)
    poly([P(x,y,z+dz),P(x+dx,y,z+dz),P(x+dx,y+dy,z+dz),P(x,y+dy,z+dz)],top,**k)
def cyl(x,y,z,r,h,top,side):
    rx,ry=r*S*1.2247,r*S*0.7071
    a,b=P(x,y,z); c,d=P(x,y,z+h)
    out.append(f'<path d="M{a-rx:.1f},{b:.1f} A{rx:.1f},{ry:.1f} 0 0 0 {a+rx:.1f},{b:.1f} L{c+rx:.1f},{d:.1f} L{c-rx:.1f},{d:.1f} Z" fill="{side}" stroke="#1c1b19" stroke-width="1.2"/>')
    out.append(f'<ellipse cx="{c:.1f}" cy="{d:.1f}" rx="{rx:.1f}" ry="{ry:.1f}" fill="{top}" stroke="#1c1b19" stroke-width="1.2"/>')
def wire(l,color,w=2.6):
    out.append(f'<polyline points="{pts([P(*p) for p in l])}" fill="none" stroke="#1c1b19" stroke-width="{w+1.6}" stroke-linejoin="round" stroke-linecap="round"/>')
    out.append(f'<polyline points="{pts([P(*p) for p in l])}" fill="none" stroke="{color}" stroke-width="{w}" stroke-linejoin="round" stroke-linecap="round"/>')
def text(x,y,s,size=20,anchor="start",weight=400,fill="#1c1b19",mono=False):
    fam='ui-monospace, SFMono-Regular, Menlo, monospace' if mono else 'ui-sans-serif, system-ui, -apple-system, Helvetica, Arial, sans-serif'
    out.append(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" text-anchor="{anchor}" font-weight="{weight}" fill="{fill}" font-family="{fam}">{s}</text>')
def line(a,b,color="#6b6862",w=1.3,dash=""):
    d=f'stroke-dasharray="{dash}"' if dash else ""
    out.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{color}" stroke-width="{w}" {d}/>')

DATA,PWR,GND="#fbfaf7","#c8402f","#1c1b19"
ACC="#1f6fd1"
STR=[(-1.5,GND),(0,PWR),(1.5,DATA)]

def cable(segs,color,w=2.6,outline=1.6):
    """segs: list of (p0,c1,c2,p1) 3D cubic segments, drawn as one smooth path"""
    d="M%.1f,%.1f"%P(*segs[0][0])
    for (_,c1,c2,p1) in segs: d+=" C%.1f,%.1f %.1f,%.1f %.1f,%.1f"%(*P(*c1),*P(*c2),*P(*p1))
    out.append(f'<path d="{d}" fill="none" stroke="#1c1b19" stroke-width="{w+outline}" stroke-linecap="round" stroke-linejoin="round"/>')
    out.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{w}" stroke-linecap="round" stroke-linejoin="round"/>')
PLUG=("#ebe9e3","#d2d0ca","#b9b7b1")
SW,SD,SL=24.8,32.9,45.4      # servo on its side: width along the chain, depth (horn axis) toward us, length up
ZP=SL/2-1.55                 # port height on the back face
PA,PBX=6.3,18.5              # port centres across the width
X=[100+44*i for i in range(6)]
COL3=[(-1.5,GND),(0,PWR),(1.5,DATA)]
grp("macbook-and-usb-lead")
# --- MacBook Pro 14-inch at true scale (312.6 x 221.2 mm), so only its left rear corner is in frame
LW,LD,BH=312.6,221.2,11.0; xr,yl=-85.0,-60.0; xl=xr-LW
AL=("#d6d7d9","#b9bbbe","#a2a4a8")
def rbox(x,y,z,dx,dy,dz,r,top,n=6):
    pts=[]
    for cx_,cy_,a0 in ((x+dx-r,y+dy-r,0),(x+r,y+dy-r,90),(x+r,y+r,180),(x+dx-r,y+r,270)):
        for j in range(n+1):
            t=math.radians(a0+90*j/n); pts.append((cx_+r*math.cos(t),cy_+r*math.sin(t)))
    for j in range(len(pts)):
        p,q=pts[j],pts[(j+1)%len(pts)]; nx,ny=q[1]-p[1],-(q[0]-p[0]); ln=math.hypot(nx,ny) or 1
        if nx+ny<=0: continue
        g=int(162+ (ny/ln)*24 - (nx/ln)*0); col="#%02x%02x%02x"%(g,g+2,g+6)
        poly([P(p[0],p[1],z),P(q[0],q[1],z),P(q[0],q[1],z+dz),P(p[0],p[1],z+dz)],col,stroke=col,sw=0.8)
    poly([P(px,py,z+dz) for px,py in pts],top)
rbox(xl,yl,0,LW,LD,BH,9,AL[0])
def T(u,v,z=BH+0.05): return P(xl+u*LW,yl+v*LD,z)
poly([T(.045,.11),T(.955,.11),T(.955,.56),T(.045,.56)],"#1b1c1f",sw=0.8)
for r_ in range(6):
    v0=.122+r_*.072
    for c_ in range(10,14):
        u0=.053+c_*.0646
        poly([T(u0,v0),T(u0+.056,v0),T(u0+.056,v0+.060),T(u0,v0+.060)],"#2f3136","#0f1012",0.6)
poly([T(.29,.60),T(.71,.60),T(.71,.97),T(.29,.97)],"#cfd0d3","#9a9ca0",0.9)
tilt=math.radians(20); ly_,lz_=LD*math.sin(tilt),LD*math.cos(tilt)
def Q(u,v,o=0.0): return P(xl+u*LW,yl+6-v*ly_-o,BH+v*lz_-o*0.36)
poly([Q(1,0),Q(1,1),Q(1,1,4.5),Q(1,0,4.5)],AL[2])
poly([Q(0,0),Q(1,0),Q(1,1),Q(0,1)],"#0e0f11")
poly([Q(.02,.04),Q(.98,.04),Q(.98,.97),Q(.02,.97)],"#23262d","#0e0f11",0.6)
for i_,(u0,wd) in enumerate([(.80,.10),(.76,.16),(.76,.08),(.80,.12),(.76,.17),(.80,.07)]):
    line(Q(u0+wd,.20-i_*.028),Q(u0,.20-i_*.028),"#7fb0f0",2)
# left side, back to front: MagSafe 3, two Thunderbolt (USB-C) ports, headphone jack
def slot(yc,w_,h_,fill="#17181a"):
    zc=BH*0.5; pts=[]
    for j in range(13): t=math.radians(-90+180*j/12); pts.append(P(xr,yc+w_/2-h_/2+h_/2*math.cos(t),zc+h_/2*math.sin(t)))
    for j in range(13): t=math.radians(90+180*j/12); pts.append(P(xr,yc-w_/2+h_/2+h_/2*math.cos(t),zc+h_/2*math.sin(t)))
    poly(pts,fill,"#6f7175",0.8)
TB1,TB2=yl+43,yl+59
slot(yl+22,15,3.6); slot(TB1,8.3,2.6); slot(TB2,8.3,2.6); slot(yl+74,3.5,3.5)
# USB-C lead: plug in the rear Thunderbolt port, cable on the table, plug in the board
box(xr,TB1-4.1,BH*0.5-1.2,1.6,8.2,2.4,"#d5d7da","#b4b7bb","#8f9296",sw=0.6)
box(xr+1.6,TB1-6.2,BH*0.5-3.25,19,12.4,6.5,"#4a4c50","#38393d","#2b2c2f")
cable([((xr+20.6,TB1,5.5),(xr+34,TB1,5.5),(xr+38,TB1+6,1.8),(-48,2,1.8)),((-48,2,1.8),(-40,6,1.8),(-34,11.1,3.2),(-21.5,11.1,3.2))],"#9a978f",8,1.6)
box(-21.5,4.9,0.2,20,12.4,6.5,"#4a4c50","#38393d","#2b2c2f")
box(-1.5,7.0,2.0,1.5,8.2,2.4,"#d5d7da","#b4b7bb","#8f9296",sw=0.6)
grp("driver-board-and-power-lead")
# the board cable's plug and first stretch sit behind the board, so they are drawn first
box(29,-6.5,2.7,7,5.5,3.6,*PLUG)
# --- board: Seeed Servo Driver Board for LeRobot, 42 x 25 mm, parts placed from Seeed's product photo
def disc(x,y,z,r,fill,stroke="none",sw=0.6):
    c=P(x,y,z); out.append(f'<ellipse cx="{c[0]:.1f}" cy="{c[1]:.1f}" rx="{r*S*1.2247:.1f}" ry="{r*S*0.7071:.1f}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')
def tq(x,y,dx,dy,z,fill,stroke="#0a0a0b",sw=0.5): poly([P(x,y,z),P(x+dx,y,z),P(x+dx,y+dy,z),P(x,y+dy,z)],fill,stroke,sw)
GOLD="#c9a24a"; HDR=("#2c2e32","#1f2023","#151617")
def header(y0):
    box(7,y0,1.6,17.8,2.5,8.5,*HDR)
    for k_ in range(7): tq(7.75+2.54*k_,y0+0.75,1.0,1.0,10.12,"#08080a")
def hole(hx,hy): disc(hx,hy,1.62,1.9,GOLD,"#1c1b19"); disc(hx,hy,1.62,1.15,"#f7f6f2","#1c1b19")
box(0,0,0,42,25,1.6,"#1e2023","#111214","#0a0b0c")
hole(26.5,2.6); hole(39.4,2.6)
for k_ in range(7): disc(8.25+2.54*k_,1.0,1.62,0.6,GOLD)              # far pad row, mostly behind the header
box(0,6.6,1.6,7.3,9,3.2,"#d5d7da","#b4b7bb","#8f9296")                # USB-C receptacle
header(2.25)                                                          # empty XIAO socket, far row
box(9.5,6,1.6,3,3,0.8,"#5b5f66","#3a3c40","#2a2c2f",stroke=ACC,sw=2)   # CH343P
for cx_,cy_ in ((14,10.2),(10,12.7)): box(cx_,cy_,1.6,2.9,1.6,1.0,"#3c3f44","#2a2c2f","#1d1e20",sw=0.7)
for cx_,cy_ in ((15,6.5),(17.5,7.5),(20,9),(22,6.3),(18,12)): tq(cx_,cy_,1.0,0.6,1.62,"#8a8d92","#0a0a0b",0.3)
box(28.5,-1,1.6,8,5.5,6,"#f4f3ef","#dddbd5","#c4c2bc")                 # servo port: right-angle, opens to the far edge
for k_ in range(3):
    box(30.2+2.5*k_,4.5,1.6,0.6,1.9,2.4,"#d5d7da","#b4b7bb","#8f9296",sw=0.5); disc(30.5+2.5*k_,6.9,1.62,0.55,GOLD)
tq(24.6,6.6,1.5,0.9,1.9,"#b98a5e","#3a2a1a",0.4); tq(24.6,8.0,1.5,0.9,1.9,"#b98a5e","#3a2a1a",0.4)
box(26.3,10,1.6,4.4,4.4,3,"#55585d","#3d4044","#2e3033")                # inductor
cyl(37,10,1.6,2.7,5.6,"#e3e4e6","#b9bbbf")                             # electrolytic cap
header(17.5)                                                          # near row
box(27,15.6,1.6,2.5,5,8,"#26282b","#1a1b1d","#111213")                 # jumper
box(31,13.5,1.6,7.5,7,8.5,"#58b56a","#3f9652","#2f7a40")               # screw terminal, entries face the servo end
for ty in (15.25,18.75):
    disc(34.75,ty,10.12,1.35,"#cfd2d4","#1c1b19",0.7)
    poly([P(38.5,ty-1.1,2.4),P(38.5,ty+1.1,2.4),P(38.5,ty+1.1,4.8),P(38.5,ty-1.1,4.8)],"#16361e","#16361e",0.5)
disc(2.5,22.8,1.62,0.9,GOLD,"#1c1b19",0.4)
for k_ in range(1,8): disc(2.5+2.54*k_,22.8,1.62,0.8,GOLD,"#1c1b19",0.4); disc(2.5+2.54*k_,22.8,1.62,0.35,"#111214")
hole(25.6,22.2); hole(39.4,22.4)
for ty,o,c in ((15.25,0.9,PWR),(18.75,-0.9,GND)):
    cable([((38.5,ty,3.6),(46,ty,3.6),(46+o,30,0.8),(46+o,45,2.2))],c)
wire([(46,45,2.6),(46,58,2.6)],"#2a2c30",17)                 # female barrel jack on the pigtail
wire([(46,60,2.6),(46,71,2.6)],"#3a3c40",12)                 # the supply's barrel plug
cable([((46,72,2.6),(46,84,1.5),(38,96,1.5),(38,114,6))],"#2a2c30",4.2,1.4)
grp("servos")
# --- servos: STS3215 on its side, back face toward the viewer (ports and idler wheel are on that face)
names=["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]
def vdisc(cx_,y,cz_,r,fill,sw=1.2):
    poly([P(cx_+r*math.cos(t*math.pi/24),y,cz_+r*math.sin(t*math.pi/24)) for t in range(48)],fill,sw=sw)
def socket(x0):
    poly([P(x0-3.9,SD,ZP-2.0),P(x0+3.9,SD,ZP-2.0),P(x0+3.9,SD,ZP+2.0),P(x0-3.9,SD,ZP+2.0)],"#f4f3ef",sw=1)
    poly([P(x0-3.1,SD,ZP-1.2),P(x0+3.1,SD,ZP-1.2),P(x0+3.1,SD,ZP+1.2),P(x0-3.1,SD,ZP+1.2)],"#2a2c30",sw=0.6)
def plug(x0): box(x0-3.5,SD,ZP-1.75,7,6,3.5,*PLUG,sw=1)
for i,x in enumerate(X):
    box(x,0,0,SW,SD,SL,"#3b3d42","#2a2c30","#1e2023")
    vdisc(x+SW/2,SD,SL/2+12.5,10,"#c9c7c1"); vdisc(x+SW/2,SD+1.8,SL/2+12.5,10,"#e9e8e4"); vdisc(x+SW/2,SD+1.9,SL/2+12.5,3,"#b9bbbf",0.8)
    socket(x+PA); socket(x+PBX)
    plug(x+PA)
    if i<5: plug(x+PBX)
    lx,ly=P(x+SW/2,0,SL+16); lx-=26
    out.append(f'<text x="{lx:.1f}" y="{ly:.1f}" text-anchor="middle" fill="#4a4843" font-size="17" font-family="ui-monospace, SFMono-Regular, Menlo, monospace"><tspan font-size="22" font-weight="700" fill="#1c1b19" font-family="ui-sans-serif, system-ui, sans-serif">{i+1}</tspan> {names[i]}</text>')
def drop(px,d_):   # from a plug on a servo's back face out and down onto the table
    return ((px+d_,SD+6,ZP),(px+d_,SD+13,ZP),(px+d_,SD+14,5),(px+d_,SD+21,0.8))
def rise(px,d_):
    return ((px+d_,SD+21,0.8),(px+d_,SD+14,5),(px+d_,SD+13,ZP),(px+d_,SD+6,ZP))
grp("servo-cables")
# board -> servo 1: round the end of the board on the table, then up into port A
xa=X[0]+PA
for d_,c in COL3:
    x0=32.5+d_
    cable([((x0,-8.4,4.5),(x0,-13,4.5),(x0+1,-16,1.4),(x0+3,-21,0.8)),      # out the back of the plug, down to the table
           ((x0+3,-21,0.8),(x0+6,-29,0.8),(52-d_,-24,0.8),(52-d_,-4,0.8)),   # round the end of the board
           ((52-d_,-4,0.8),(52-d_,22,0.8),(xa+d_-14,SD+46,0.8),(xa+d_,SD+21,0.8)),
           rise(xa,d_)],c)
# servo -> servo: each cable lies in a loop on the table
for i in range(5):
    b_=X[i]+PBX; a_=X[i+1]+PA
    for d_,c in COL3:
        cable([drop(b_,d_),((b_+d_,SD+21,0.8),(b_+d_+3,SD+44,0.8),(a_+d_-3,SD+44,0.8),(a_+d_,SD+21,0.8)),rise(a_,d_)],c)

grp("power-supply")
box(26,112,0,24,40,16,"#3b3d42","#2a2c30","#1e2023")
grp("labels")
# --- labels
text(1460,532,"MacBook Pro (to scale)",20,"end",weight=600)
text(1460,556,"/dev/cu.usbmodem…",16,"end",fill="#4a4843",mono=True)
up=P(xr+2,TB1-6,BH); text(up[0]-74,up[1]+92,"USB-C",18,"end",fill="#4a4843")
bp=P(4,25,0); text(bp[0]+30,bp[1]+70,"servo driver board",20,weight=600); line((bp[0]+50,bp[1]+50),(bp[0]-4,bp[1]+8))
jp=P(46,58,2.6); text(jp[0]+10,jp[1]+70,"barrel jack",16,"middle",fill="#4a4843"); line((jp[0]+8,jp[1]+53),(jp[0]+2,jp[1]+13))
pp=P(38,152,0); text(pp[0],pp[1]+34,"power supply",20,"middle",weight=600)
text(pp[0],pp[1]+57,"5–12 V DC, barrel plug into a two-wire pigtail",16,"middle",fill="#4a4843")

grp("chip-inset")
# --- inset: CH343P as a QFN16, 3 x 3 mm body, 0.5 mm pitch, leadless (WCH datasheet package)
ix,iy,iw,ih=50,40,600,380
chip=P(11,7.5,2.4)
line(chip,(ix+iw,iy+ih-60),ACC,1.6,"5 4")
out.append(f'<rect x="{ix}" y="{iy}" width="{iw}" height="{ih}" rx="14" fill="#ffffff" stroke="{ACC}" stroke-width="1.8"/>')
text(ix+22,iy+36,"CH343P, the only chip between Mac and servos",19,weight=600)
k=42; ccx,ccy=ix+300,iy+92; CH=0.75
def Pc(x,y,z=0): return (ccx+(x-y)*C*k, ccy+((x+y)*0.5-z)*k)
def cq(l,fill,stroke="#1c1b19",sw=1.2): poly([Pc(*p) for p in l],fill,stroke,sw)
cq([(0,3,0),(3,3,0),(3,3,CH),(0,3,CH)],"#26282c")
cq([(3,0,0),(3,3,0),(3,3,CH),(3,0,CH)],"#1a1b1e")
cq([(0,0,CH),(3,0,CH),(3,3,CH),(0,3,CH)],"#34363b")
pads=[0.75,1.25,1.75,2.25]
for p in pads:
    cq([(p-.13,3,0),(p+.13,3,0),(p+.13,3,.22),(p-.13,3,.22)],"#d9d7d2",sw=0.8)   # pins 1-4
    cq([(3,p-.13,0),(3,p+.13,0),(3,p+.13,.22),(3,p-.13,.22)],"#bdbbb5",sw=0.8)   # pins 5-8
d=Pc(0.42,2.58,CH); out.append(f'<ellipse cx="{d[0]:.1f}" cy="{d[1]:.1f}" rx="6" ry="3.5" fill="#8f9296"/>')
o=Pc(0.55,1.72,CH)
out.append(f'<text transform="matrix(0.866,0.5,-0.866,0.5,{o[0]:.1f},{o[1]:.1f})" font-size="24" font-weight="700" fill="#e9e8e4" font-family="ui-monospace, SFMono-Regular, Menlo, monospace">CH343P</text>')
# serial side: pin 4 TXD, pin 5 RXD meet at the near corner
tx=Pc(2.25,3,0.02); rx_=Pc(3,2.25,0.02); by=iy+ih-92
line(tx,(tx[0],by),"#6b6862",3); line(rx_,(rx_[0],by),"#6b6862",3)
text(tx[0]-10,by-22,"TXD ↓",15,"end",fill="#4a4843",mono=True); text(rx_[0]+10,by-22,"↑ RXD",15,"start",fill="#4a4843",mono=True)
out.append(f'<rect x="{ccx-62}" y="{by}" width="124" height="46" rx="6" fill="#f1efe9" stroke="#1c1b19" stroke-width="1.3"/>')
text(ccx,by+29,"2 buffers",16,"middle",600)
line((ccx-62,by+23),(ix+150,by+23),"#1c1b19",7); line((ccx-63,by+23),(ix+151,by+23),DATA,4)
text(ix+22,by+19,"one data wire",17,weight=600); text(ix+22,by+41,"to all six servos",15,fill="#4a4843")
# USB side: pin 7 UD+, pin 8 UD-
for p,lab,dy in ((1.25,"D+",16),(0.75,"D−",-4)):
    a_=Pc(3,p,0.02); line(a_,(ix+iw-150,a_[1]),"#9a978f",3)
    text(ix+iw-140,a_[1]+dy,lab,15,"start",fill="#4a4843",mono=True)
u=Pc(3,1.0,0.02); text(ix+iw-100,u[1]-4,"USB-C",17,weight=600); text(ix+iw-100,u[1]+17,"to Mac",15,fill="#4a4843")
text(ix+22,iy+66,"QFN16 package, 3 × 3 mm",15,fill="#4a4843")

grp("legend")
# --- legend
gx,gy=900,880
text(gx,gy,"The 3 wires in every cable",19,weight=600)
for k,(c,lab,sub) in enumerate([(DATA,"data","half-duplex UART, 1 Mbps, shared by all six servos"),(PWR,"power","from the board's screw terminal"),(GND,"ground","")]):
    y=gy+34+k*30
    out.append(f'<line x1="{gx}" y1="{y-6}" x2="{gx+46}" y2="{y-6}" stroke="#1c1b19" stroke-width="6.5" stroke-linecap="round"/><line x1="{gx}" y1="{y-6}" x2="{gx+46}" y2="{y-6}" stroke="{c}" stroke-width="4.5" stroke-linecap="round"/>')
    text(gx+60,y,lab,18,weight=600); text(gx+130,y,sub,16,fill="#4a4843")
text(gx,gy+132,"Servos 45.4 × 24.8 × 32.9 mm, shown back face out, from the SO-101 model.",14,fill="#6b6862")
text(gx,gy+152,"Board 42 × 25 mm, laid out from Seeed's product photo.",14,fill="#6b6862")
text(gx,gy+172,"Mac: 14-inch MacBook Pro, to scale.",14,fill="#6b6862")

W,H=1480,1080
svg=f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" role="img" aria-label="Isometric view of a Mac connected by USB-C to the SO-101 servo driver board, whose CH343P chip feeds one three-wire cable daisy-chained through six servos, each lying with its back face out to show its two ports and the plugged cables"><defs><clipPath id="cv"><rect width="{W}" height="{H}" rx="18"/></clipPath></defs><rect width="{W}" height="{H}" rx="18" fill="#f7f6f2"/><g clip-path="url(#cv)">'+"\n".join(out)+"</g></g></svg>"
import os, sys, base64
argv=sys.argv[1:]
render=None
if "--render" in argv:
    # Blender's cel-shaded render (build_scene.py) in place of the drawn parts: the same camera, so
    # every label above still lands on its part. Only the text inside the part groups is kept.
    i=argv.index("--render"); render=argv[i+1]; del argv[i:i+2]
    PARTS={"macbook-and-usb-lead","driver-board-and-power-lead","servos","servo-cables","power-supply"}
    kept,group=[],None
    for e in out:
        if e.startswith('<g id="'): group=e[7:e.index('"',7)]
        if group in PARTS and not (e.startswith("<text") or e.startswith("<g ") or e=="</g>"): continue
        kept.append(e)
    data=base64.b64encode(open(render,"rb").read()).decode()
    kind="webp" if render.endswith(".webp") else "png"
    image=f'<image href="data:image/{kind};base64,{data}" x="0" y="0" width="{W}" height="{H}"/>'
    svg=svg.replace("\n".join(out),image+"\n"+"\n".join(kept))
dst=argv[0] if argv else os.path.join(os.path.dirname(os.path.abspath(__file__)),"so101-wiring.svg")
open(dst,"w").write(svg); print("wrote",dst)
