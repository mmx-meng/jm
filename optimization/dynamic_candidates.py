"""按释放货量生成新双服务区组批，使用相应新访问顺序重建通信缺口。"""
import math
from collections import Counter
from itertools import combinations
from core import energy_leg,charge_time


def key(c):return (c['model'],tuple(c['order']),tuple(sorted((int(k),int(v)) for k,v in c['qty'].items())))


class Builder:
    def __init__(self,candidates,groups,models,terrain,coverage):
        self.candidates=candidates;self.groups=groups;self.models=models;self.terrain=terrain;self.coverage=coverage
        self.known={key(c) for c in candidates};self.generated=[]

    def generate(self,solution,released,rng,attempts=60):
        available=Counter()
        for t in solution['transport']:
            if t['candidate_id'] in released:available.update({int(k):v for k,v in t['qty'].items()})
        gs={}
        for i,q in available.items():gs.setdefault(self.groups[i]['node'],[]).append(i)
        pairs=list(combinations(gs,2));rng.shuffle(pairs);added=0
        for step in range(attempts):
            if not pairs:break
            a,b=pairs[step%len(pairs)]
            if self.terrain.leg(a,b)['d']>7500:continue
            order=[a,b] if rng.random()<.5 else [b,a]
            qty={i:rng.randint(0,available[i]) for n in order for i in gs[n]};qty={i:q for i,q in qty.items() if q}
            if any(not any(self.groups[i]['node']==n for i in qty) for n in order):continue
            for name,m in self.models.items():
                signature=(name,tuple(order),tuple(sorted(qty.items())))
                if signature in self.known:continue
                mass=sum(self.groups[i]['mass']*q for i,q in qty.items());volume=sum(self.groups[i]['volume']*q for i,q in qty.items())
                if mass>m.payload+1e-8 or volume>m.volume+1e-8:continue
                counts={n:sum(q for i,q in qty.items() if self.groups[i]['node']==n) for n in order}
                remaining=mass;energy=0.;seq=['O01']+order+['O01']
                for u,v in zip(seq,seq[1:]):
                    geo=self.terrain.leg(u,v);energy+=energy_leg(m,geo['d'],geo['up'],remaining)
                    remaining-=sum(self.groups[i]['mass']*q for i,q in qty.items() if self.groups[i]['node']==v)
                if energy>(1-m.reserve)*m.energy:continue
                duration,delivery,intervals=self.coverage.route_profile(order,m,counts)
                prep=m.prep+m.load*sum(qty.values());reqs=[];invalid=False
                for segment in intervals:
                    if segment['mask']&1:continue
                    if not segment['mask']:invalid=True;break
                    start=prep+segment['start'];end=prep+segment['end'];mask=segment['mask']
                    if reqs and abs(reqs[-1]['end']-start)<1e-7 and reqs[-1]['mask']&mask:
                        reqs[-1]['end']=end;reqs[-1]['mask']&=mask
                    else:reqs.append(dict(start=start,end=end,mask=mask))
                if invalid:continue
                latest=min(min(self.groups[i]['deadline'],self.groups[i]['due'])-prep-delivery[self.groups[i]['node']] for i in qty)
                if latest<0:continue
                c=dict(id=max(c['id'] for c in self.candidates)+1,model=name,order=order,qty=qty,mass=mass,volume=volume,
                       energy=energy,soc=1-energy/m.energy,prep=prep,duration=duration,total=math.ceil(prep+duration),
                       delivery=delivery,latest=math.floor(latest),charge=math.ceil(charge_time(1-energy/m.energy,m.full_charge)),
                       requirements=reqs,options=[],relay_start=min((r['start'] for r in reqs),default=0),
                       relay_end=max((r['end'] for r in reqs),default=0),
                       weight=sum(self.groups[i]['weight']*q for i,q in qty.items()),
                       delivery_const=sum(self.groups[i]['weight']*q*(prep+delivery[self.groups[i]['node']]) for i,q in qty.items()))
                self.candidates.append(c);self.generated.append(c);self.known.add(signature);added+=1
        return added
