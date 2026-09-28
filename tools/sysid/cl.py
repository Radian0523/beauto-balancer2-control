import numpy as np, scipy.linalg as la
S='' + __import__('os').path.dirname(__import__('os').path.abspath(__file__)) + '/'
def plant(p):
    a,bt,dt_,c,bp,dp=p
    A=np.array([[0,1,0,0],[a,0,0,dt_],[0,0,0,1],[c,0,0,dp]],float)   # x=[θ, θ̇, ψ, ψ̇]
    B=np.array([[0],[bt],[0],[bp]],float)*1e-4                          # u = 電流指令
    return A,B
def closed_loop(p, delay_ms, K, T=0.002):
    A,B=plant(p)
    n=4; Ad=la.expm(A*T); Bd=np.linalg.solve(A, (Ad-np.eye(n))@B) if np.linalg.cond(A)<1e12 else None
    if Bd is None:  # A が特異なら数値積分
        M=la.expm(np.block([[A,B],[np.zeros((1,5))]])*T); Ad=M[:4,:4]; Bd=M[:4,4:]
    d=int(round(delay_ms/1000/T))
    # 状態: [x(4), u(k-1..k-d)]
    N=n+d
    F=np.zeros((N,N)); F[:n,:n]=Ad
    if d==0:
        F[:n,:n]=Ad+Bd@K.reshape(1,4)
    else:
        F[:n,n+d-1:n+d]=Bd              # 一番古い指令が効く
        F[n,:n]=K                       # 新しい指令 u(k) = K x(k)
        for i in range(1,d): F[n+i,n+i-1]=1
    ev=np.linalg.eigvals(F)
    return ev
Kstock=np.array([-36400,-5460,1320,396],float)
for name in ['p_d4','best']:
    v=np.load(S+name+'.npy'); d=v[0]*4; p=v[1:]
    ev=closed_loop(p,d,Kstock)
    s=np.log(ev.astype(complex))/0.002
    worst=s[np.argmax(s.real)]
    osc=[(abs(x.imag)/2/np.pi, -x.real/abs(x)) for x in s if x.imag>1 and abs(x)>1]
    osc.sort()
    print(f"{name}: 遅れ {d:.0f} ms  最も不安定な極 {worst.real:+.1f} 1/s ({abs(worst.imag)/2/np.pi:.1f} Hz)  "
          "振動する極 (Hz, 減衰比): " + ", ".join(f"{f:.1f}Hz ζ={z:.2f}" for f,z in osc[:5]))
