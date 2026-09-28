import numpy as np
from scipy.optimize import least_squares
SD='' + __import__('os').path.dirname(__import__('os').path.abspath(__file__)) + '/'
exec(open(SD+'fit.py').read().split('best=None')[0])   # データ読み込み・sim・resid
A_LOCK=63.0
def phys(q):
    """q = (m11, m12, m22, bf) (電流指令 1e4 あたりのトルクで割った値)。重力 G は a_lock から決める"""
    m11,m12,m22,bf=q
    Ic=m11+2*m12+m22; G=A_LOCK*Ic; det=m11*m22-m12**2
    a=m11*G/det; c=-(m11+m12)*G/det
    bt=-(m11+m12)/det; bp=Ic/det           # 入力 u (×1e-4) への係数
    # 摩擦 bf: 相対回転 ψ̇ に逆らうトルク -bf ψ̇ はモータと同じ向きに入る
    return [a, bt, -bt*bf, c, bp, -bp*bf], dict(det=det, Ic=Ic, G=G)
def res(q, delay):
    p,info=phys(q)
    if info['det']<=0 or info['Ic']<=0: return np.full(10000,1e3)[:len(resid([1,1,0,1,1,0],delay))]
    return resid(p, delay)
out=[]
for delay in (2,3,4,5):
    best=None
    for x0 in ([0.005,-0.02,0.12,0.01],[0.002,-0.01,0.05,0.005],[0.01,-0.04,0.3,0.02]):
        f=least_squares(res,x0,args=(delay,),bounds=([1e-5,-1,1e-4,0],[1,0,5,1]))
        if best is None or f.cost<best.cost: best=f
    p,info=phys(best.x); cost=np.mean(best.fun**2)
    print(f"遅れ {delay*4} ms 誤差 {cost:.3f}  m11={best.x[0]:.4f} m12={best.x[1]:.4f} m22={best.x[2]:.4f} bf={best.x[3]:.4f} | a={p[0]:.1f} bθ={p[1]:.1f} c={p[3]:.1f} bψ={p[4]:.1f} dψ={p[5]:.1f}")
    out.append((cost,delay,p))
cost,delay,p=min(out,key=lambda x:x[0])
np.save(SD+'phys.npy', np.r_[delay,p]); print('採用 遅れ', delay*4,'ms')
