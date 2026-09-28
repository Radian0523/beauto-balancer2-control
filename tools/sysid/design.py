import numpy as np
from scipy.optimize import minimize
SD='' + __import__('os').path.dirname(__import__('os').path.abspath(__file__)) + '/'
exec(open(SD+'cl.py').read().split('Kstock=')[0])
Kstock=np.array([-36400,-5460,1320,396],float)
p=np.load(SD+'phys.npy')[1:]
def modes(K, d):
    ev=closed_loop(p,d,K); s=np.log(ev.astype(complex))/0.002
    return s
def score(sc, dlist=(12,16,24)):
    K=Kstock*sc
    J=0
    for d in dlist:           # 遅れが 12〜24 ms のどれでも良いように
        s=modes(K,d)
        for x in s:
            if abs(x)<0.5: continue
            f=abs(x.imag)/2/np.pi
            if f>40: continue
            z=-x.real/abs(x)
            J+=max(0,0.6-z)**2*10        # 減衰比 0.6 未満を強く嫌う
            if x.real>0: J+=100+x.real
    J+=0.3*max(0,sc[1]-1)**2 + 0.3*max(0,sc[0]-1.3)**2   # 傾く速さのゲインは増やしたくない (振動の元)
    return J
best=None
rng=np.random.default_rng(0)
for trial in range(300):
    x0=rng.uniform([0.6,0.4,0.3,0.3],[1.5,1.2,2.0,2.5])
    r=minimize(score,x0,method='Nelder-Mead',options=dict(maxiter=400,xatol=1e-3,fatol=1e-6))
    if best is None or r.fun<best.fun: best=r
sc=best.x; K=Kstock*sc
print("倍率 (K1 傾き, K2 傾く速さ, K3 車輪位置, K4 車輪の速さ):", np.round(sc,2), " 評価", round(best.fun,4))
print("ゲイン:", np.round(K).astype(int))
for lab,KK in (('純正',Kstock),('設計',K)):
    for d in (12,16,24):
        s=modes(KK,d)
        osc=sorted([(abs(x.imag)/2/np.pi,-x.real/abs(x)) for x in s if abs(x)>0.5 and abs(x.imag)/2/np.pi<40])
        slow=[x for x in s if abs(x)>0.5 and abs(x.imag)<1e-6]
        print(f"  {lab} 遅れ{d}ms: " + ", ".join(f"{f:.1f}Hz ζ={z:.2f}" for f,z in osc[:4]))
np.save(SD+'design.npy', sc)
