import re,sys,statistics as st
# разбор остановок после W/S: качание корпуса и откат после отпускания клавиши
f=sys.argv[1]
rows=[]
for l in open(f,encoding='utf-8'):
    m=re.match(r'\s*([\d.]+) (\S+)\s+(A|-) a=(-?[\d.]+) r=(-?\d+) v=(-?[\d.]+) vA=(-?[\d.]+) vB=(-?[\d.]+) x=(-?\d+) m=(-?\d+)',l)
    if m: rows.append((float(m[1]),m[2],float(m[4]),int(m[5]),float(m[6])))
for i in range(1,len(rows)):
    k=rows[i-1][1]
    if k in ('w','s') and rows[i][1]=='.':
        j=i-1
        while j>0 and rows[j][1]==k: j-=1
        if rows[i-1][0]-rows[j][0]<1.0: continue
        t0=rows[i][0]; win=[r for r in rows[i:] if r[0]-t0<2.0]
        v0=rows[i-1][4]; back=min(r[4] for r in win) if v0>0 else max(r[4] for r in win)
        print(f'{k} @{t0:6.2f} v0 {v0:5.1f}  откат {back:5.1f}  |r|max {max(abs(r[3]) for r in win):4d}  r sd {st.pstdev(r[3] for r in win):5.1f}  a sd {st.pstdev(r[2] for r in win):.2f}')
