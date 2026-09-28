import numpy as np
from scipy.optimize import minimize
SD='' + __import__('os').path.dirname(__import__('os').path.abspath(__file__)) + '/'
exec(open(SD+'design.py').read().split('best=None')[0])
LO=np.array([0.7,0.6,0.7,0.5]); HI=np.array([1.6,1.2,1.6,3.0])
def score2(sc):
    pen=np.sum(np.maximum(0,LO-sc)**2+np.maximum(0,sc-HI)**2)*1000
    return score(sc)+pen
res=[]
rng=np.random.default_rng(1)
for trial in range(200):
    x0=rng.uniform(LO,HI)
    r=minimize(score2,x0,method='Nelder-Mead',options=dict(maxiter=500,xatol=1e-3,fatol=1e-7))
    res.append(r)
res.sort(key=lambda r:r.fun)
seen=[]
for r in res:
    sc=np.clip(r.x,LO,HI)
    if any(np.max(np.abs(sc-s))<0.05 for s in seen): continue
    seen.append(sc)
    print("倍率", np.round(sc,2), "評価", round(r.fun,4))
    for d in (12,16,24):
        s=modes(Kstock*sc,d)
        osc=sorted([(abs(x.imag)/2/np.pi,-x.real/abs(x)) for x in s if abs(x)>0.5 and x.imag>=0 and abs(x.imag)/2/np.pi<40])
        print(f"   遅れ{d}ms: " + ", ".join(f"{f:.1f}Hz ζ={z:.2f}" for f,z in osc[:4]))
    if len(seen)>=4: break
