import numpy as np
SD='' + __import__('os').path.dirname(__import__('os').path.abspath(__file__)) + '/'
exec(open(SD+'cl.py').read().split('Kstock=')[0])
Kstock=np.array([-36400,-5460,1320,396],float)
pp=np.load(SD+'phys.npy')[1:]
models={'物理 (a=63)':pp, '最初の合わせ込み (a≈2)':np.load(SD+'best.npy')[1:], '物理で a を 45 に':None, '物理で a を 85 に':None}
def with_a(a):
    q=pp.copy(); q[0]=a; return q
models['物理で a を 45 に']=with_a(45); models['物理で a を 85 に']=with_a(85)
# 電流の効き (bθ, bψ) を ±30% ずらしたもの
for f in (0.7,1.3):
    q=pp.copy(); q[1]*=f; q[4]*=f; models[f'物理で電流の効き ×{f}']=q
cands={'純正':[1,1,1,1],'候補1':[1.3,0.99,1.0,1.25],'候補2':[1.05,0.86,0.72,1.07]}
for mn,p in models.items():
    line=f"{mn:22s}"
    for cn,sc in cands.items():
        ev=closed_loop(p,16,Kstock*np.array(sc)); s=np.log(ev.astype(complex))/0.002
        osc=[(-x.real/abs(x)) for x in s if abs(x)>0.5 and abs(x.imag)/2/np.pi<40 and x.imag>0.1]
        zmin=min(osc) if osc else 1.0
        stable=all(abs(ev)<1)
        line+=f" | {cn}: {'安定' if stable else '不安定'} ζmin={zmin:+.2f}"
    print(line)
