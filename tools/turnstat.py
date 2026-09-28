import re,sys,statistics as st
f=sys.argv[1]
rows=[]
for l in open(f,encoding='utf-8'):
    m=re.match(r'\s*([\d.]+) (\S+)\s+(A|-) a=(-?[\d.]+) r=(-?\d+) v=(-?[\d.]+) vA=(-?[\d.]+) vB=(-?[\d.]+) x=(-?\d+) m=(-?\d+) yr=(-?\d+) h=(-?[\d.]+) uL=(-?\d+) uR=(-?\d+)',l)
    if m: rows.append(m.groups())
eps=[];cur=None
for r in rows:
    if r[1] in ('a','d'):
        if not cur or cur[0]!=r[1]: cur=[r[1],[]]; eps.append(cur)
        cur[1].append(r)
    else: cur=None
allv=[];allx=[]
for k,e in eps:
    if len(e)<15: continue
    t=e[5:]
    v=[float(x[5]) for x in t]; xx=[float(x[8]) for x in t]
    allv+=v
    print(k,e[0][0],f'{len(e)*0.1:.1f}s','yr %.0f'%st.mean(float(x[10]) for x in t),'A %.1f B %.1f'%(st.mean(float(x[6]) for x in t),st.mean(float(x[7]) for x in t)),'v sd %.1f  x range %d..%d  a sd %.1f'%(st.pstdev(v),min(xx),max(xx),st.pstdev(float(x[3]) for x in t)))
if allv: print('ALL v sd %.2f'%st.pstdev(allv))
