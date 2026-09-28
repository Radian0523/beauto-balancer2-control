import csv, numpy as np
from scipy.optimize import least_squares
def load(f):
    r=list(csv.DictReader(open(f)))
    g=lambda k: np.array([float(x[k]) for x in r])
    return dict(t=g('t_s'), th=g('body_angle_rad'), w=g('body_rate_rad_s'), psi=g('wheel_err_rad'), dpsi=g('wheel_spd_rad_s'), u=g('current_cmd'))
D=[load('logs/run_20260927_160703.csv'), load('logs/run_20260927_161118.csv')]
dt=0.004; W=50  # 0.2 s windows
UMAX=7000.0
def sim(p, d, i0, n, delay):
    a,bt,dt_,c,bp,dp=p
    th,w,psi,dpsi = d['th'][i0], d['w'][i0], d['psi'][i0], d['dpsi'][i0]
    out=[]
    for k in range(n):
        u=np.clip(d['u'][max(0,i0+k-delay)],-UMAX,UMAX)*1e-4
        acc_t = a*th + bt*u + dt_*dpsi
        acc_p = c*th + bp*u + dp*dpsi
        th += w*dt; psi += dpsi*dt
        w += acc_t*dt; dpsi += acc_p*dt
        out.append((w,psi))
    return np.array(out)
def resid(p, delay):
    res=[]
    for d in D:
        N=len(d['t'])
        for i0 in range(0,N-W-1,W//2):
            s=sim(p,d,i0,W,delay)
            res.append((s[:,0]-d['w'][i0+1:i0+W+1])/0.5)      # 角速度 [rad/s] の重み
            res.append((s[:,1]-d['psi'][i0+1:i0+W+1])/0.05)   # 車輪角 [rad] の重み
    return np.concatenate(res)
best=None
for delay in range(0,6):
    fit=least_squares(resid,[100,-50,0,-300,300,-5],args=(delay,))
    cost=np.mean(fit.fun**2)
    print(f"遅れ {delay*4:2d} ms: 誤差 {cost:.3f}  a={fit.x[0]:8.1f} bθ={fit.x[1]:8.1f} dθ={fit.x[2]:7.2f} c={fit.x[3]:8.1f} bψ={fit.x[4]:8.1f} dψ={fit.x[5]:7.2f}")
    if best is None or cost<best[0]: best=(cost,delay,fit.x)
np.save('' + __import__('os').path.dirname(__import__('os').path.abspath(__file__)) + '/best.npy', np.r_[best[1],best[2]])
print("採用: 遅れ", best[1]*4, "ms")
