#!/usr/bin/env python3
"""Deterministic synthetic Phase-1 legacy-EMA versus linear-KF benchmark."""
import math
import random
import time


class KF:
    def __init__(self, x, y, p=.1, v=1.):
        self.x = [x, y, 0., 0.]; self.p = [[0.]*4 for _ in range(4)]
        self.p[0][0] = self.p[1][1] = p*p; self.p[2][2] = self.p[3][3] = v*v
    def predict(self, dt, a=1.5):
        f = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
        self.x[0] += dt*self.x[2]; self.x[1] += dt*self.x[3]
        self.p = mm(mm(f,self.p), tr(f)); q=a*a; d2=dt*dt; d3=d2*dt; d4=d2*d2
        for i in (0,1): self.p[i][i] += q*d4/4
        self.p[0][2] += q*d3/2; self.p[2][0] += q*d3/2; self.p[1][3] += q*d3/2; self.p[3][1] += q*d3/2
        self.p[2][2] += q*d2; self.p[3][3] += q*d2
    def update(self, z, r=.1):
        inn=[z[0]-self.x[0],z[1]-self.x[1]]; rr=r*r
        s00=self.p[0][0]+rr; s01=self.p[0][1]; s10=self.p[1][0]; s11=self.p[1][1]+rr; d=s00*s11-s01*s10
        inv=[[s11/d,-s01/d],[-s10/d,s00/d]]; k=[[self.p[i][0]*inv[0][j]+self.p[i][1]*inv[1][j] for j in range(2)] for i in range(4)]
        for i in range(4): self.x[i] += k[i][0]*inn[0]+k[i][1]*inn[1]
        ikh=eye()
        for i in range(4): ikh[i][0]-=k[i][0]; ikh[i][1]-=k[i][1]
        self.p=mm(mm(ikh,self.p),tr(ikh))
        for i in range(4):
            for j in range(4): self.p[i][j] += rr*(k[i][0]*k[j][0]+k[i][1]*k[j][1])


def mm(a,b): return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
def tr(a): return [[a[j][i] for j in range(4)] for i in range(4)]
def eye(): return [[float(i==j) for j in range(4)] for i in range(4)]
def rms(v): return math.sqrt(sum(x*x for x in v)/len(v)) if v else 0.


def run(name, truth, dts, missing=()):
    random.seed(7); legacy=None; kf=None; raw=[0.,0.]; lv=[0.,0.]; t=0.; le=[]; ke=[]; lve=[]; kve=[]; drops=[]
    for n, dt in enumerate(dts):
        t += dt; tx,ty,tvx,tvy=truth(t); z=(tx+random.gauss(0,.10),ty+random.gauss(0,.10))
        if legacy is None: legacy=list(z); raw=list(z); kf=KF(*z)
        else:
            # Existing tracker path: raw association prediction plus 0.55 measurement EMA velocity.
            raw[0]+=lv[0]*dt; raw[1]+=lv[1]*dt; lv=[.45*lv[i]+.55*(z[i]-raw[i])/dt for i in range(2)]
            legacy=list(z); kf.predict(min(.5,max(.01,dt)))
        if n not in missing: kf.update(z); raw=list(z)
        le.append(math.hypot(legacy[0]-tx,legacy[1]-ty)); ke.append(math.hypot(kf.x[0]-tx,kf.x[1]-ty))
        lve.append(math.hypot(lv[0]-tvx,lv[1]-tvy)); kve.append(math.hypot(kf.x[2]-tvx,kf.x[3]-tvy))
        if n in missing: drops.append(ke[-1])
    jitter_l=rms([le[i]-le[i-1] for i in range(1,len(le))]); jitter_k=rms([ke[i]-ke[i-1] for i in range(1,len(ke))])
    return name, rms(le), rms(ke), rms(lve), rms(kve), jitter_l, jitter_k, max(drops,default=0.)


def main():
    d=[.1]*60
    scenarios=[
      run('S01 static',lambda t:(0.,0.,0.,0.),d),
      run('S02 constant velocity',lambda t:(t,0.,1.,0.),d),
      run('S03 sudden motion',lambda t:(max(0,t-2),0.,0. if t<2 else 1.,0.),d),
      run('S04 sudden stop',lambda t:(min(t,2),0.,1. if t<2 else 0.,0.),d),
      run('S05 one-frame dropout',lambda t:(t,0.,1.,0.),d,{20}),
      run('S06 three-frame dropout',lambda t:(t,0.,1.,0.),d,{20,21,22}),
      run('S07 irregular dt',lambda t:(t,0.,1.,0.),[.10,.09,.12,.08,.14]*12),
      run('S08 recreation',lambda t:(t,0.,1.,0.),d),
    ]
    print('scenario,legacy_pos_rmse,kf_pos_rmse,legacy_vel_rmse,kf_vel_rmse,legacy_jitter,kf_jitter,kf_dropout_error')
    for row in scenarios: print(','.join([row[0]]+[f'{x:.4f}' for x in row[1:]]))
    print('\ntracks,legacy_us_per_frame,kf_us_per_frame')
    for count in (1,10,50,100):
        start=time.perf_counter(); filters=[KF(0,0) for _ in range(count)]
        for _ in range(300):
            for k in filters: k.predict(.1); k.update((1.,0.))
        print(f'{count},0.000,{(time.perf_counter()-start)*1e6/300:.2f}')


if __name__ == '__main__': main()
