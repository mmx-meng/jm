"""所有快速接受均按原始箱号、浮点航段能耗与充电时间核查。"""
from collections import Counter,defaultdict
from core import energy_leg,charge_time


def fast_check(solution,baseline,boxes,models,relay,terrain,sites,candidate_by_id,protect_weighted=True):
    failures=[];intervals=defaultdict(list);delivery={};te=0.;re=0.;tsoc=[];rsoc=[]
    def check(ok,reason):
        if not ok:failures.append(reason)
    allids=[b for t in solution['transport'] for b in t['boxes']]
    check(Counter(allids)==Counter(boxes.keys()),'box_mapping')
    rmap={r['site_index']:r for r in solution['relay']}
    for t in solution['transport']:
        m=models[t['model']];c=candidate_by_id[t['candidate_id']]
        check(t['uav'] in m.units and t['battery'] in m.batteries,'resource_type')
        mass=sum(boxes[b]['mass'] for b in t['boxes']);vol=sum(boxes[b]['volume'] for b in t['boxes'])
        check(mass<=m.payload+1e-7 and vol<=m.volume+1e-7,'capacity')
        check(set(t['order'])=={boxes[b]['node'] for b in t['boxes']},'box_mapping')
        now=t['start']+m.prep+m.load*len(t['boxes']);q=mass;e=0
        check(t['start']>=-1e-6 and abs(now-t['takeoff'])<1e-5,'timing')
        seq=['O01']+t['order']+['O01']
        for a,b in zip(seq,seq[1:]):
            geo=terrain.leg(a,b);e+=energy_leg(m,geo['d'],geo['up'],q)
            now+=geo['up']/m.up+geo['d']/m.speed+geo['down']/m.down
            if b!='O01':
                ids=[x for x in t['boxes'] if boxes[x]['node']==b]
                now+=m.handover+m.per_box*len(ids)
                for x in ids:delivery[x]=now
                q-=sum(boxes[x]['mass'] for x in ids)
        check(abs(now-t['return_time'])<1e-5,'timing');check(abs(e-t['energy'])<1e-6,'energy_formula')
        check(e<=(1-m.reserve)*m.energy+1e-7,'soc');te+=e;tsoc.append(1-e/m.energy)
        intervals[t['uav']].append((t['start'],now));intervals[t['battery']].append((t['start'],now+charge_time(1-e/m.energy,m.full_charge)))
        assigns={a['requirement_index']:a for a in t['relay_assignments']}
        for i,req in enumerate(c['requirements']):
            a=assigns.get(i);r=rmap.get(a['site_index']) if a else None
            check(r is not None,'communication')
            if r:
                check(bool(req['mask']&(1<<(r['base_site_index']+1))),'communication')
                check(r['ready']<=t['start']+req['start']+1e-6 and r['service_end']>=t['start']+req['end']-1e-6,'communication')
    for r in solution['relay']:
        s=sites[r['base_site_index']];power=relay['hover']+relay['communication']
        e=s['flight_energy']+s['link_energy']+power*(r['service_end']-r['ready'])/3600
        check(r['start']>=-1e-6 and r['takeoff']>=r['start']+relay['prep']-1e-6,'timing')
        check(abs(r['ready']-r['takeoff']-s['out']-relay['link'])<1e-5,'timing')
        check(abs(r['return_time']-r['service_end']-s['back'])<1e-5,'timing')
        check(r['service_end']>=r['ready']-1e-6,'timing')
        check(abs(e-r['energy'])<1e-6,'energy_formula');check(e<=(1-relay['reserve'])*relay['energy']+1e-7,'soc')
        re+=e;rsoc.append(1-e/relay['energy'])
        intervals[r['uav']].append((r['start'],r['return_time']+relay['turn']))
        intervals[r['component']].append((r['start'],r['return_time']+charge_time(1-e/relay['energy'],relay['full_charge'])))
    for spans in intervals.values():
        spans.sort()
        check(all(a[1]<=b[0]+1e-6 for a,b in zip(spans,spans[1:])),'resource_overlap')
    ds=solution['deliveries'];check(Counter(d['box'] for d in ds)==Counter(boxes.keys()),'box_mapping')
    for d in ds:
        check(abs(d['time']-delivery.get(d['box'],-1e9))<1e-5,'box_mapping')
        check(d['time']<=boxes[d['box']]['deadline']+1e-6,'deadline')
        check(d['time']<=boxes[d['box']]['due']+1e-6,'tardiness')
    w=sum(boxes[x]['weight'] for x in delivery)
    weighted=sum(boxes[x]['weight']*t for x,t in delivery.items())/w/60
    met=baseline['metrics']
    for key,value in [('transport_energy_kwh',te),('relay_energy_kwh',re),('total_energy_kwh',te+re)]:check(value<=met[key]+1e-8,'energy_cap')
    check(min(tsoc)>=met['min_transport_soc']-1e-9,'soc_cap');check(min(rsoc,default=1)>=met['min_relay_soc']-1e-9,'soc_cap')
    if protect_weighted:check(weighted<=met['weighted_delivery_min']+1e-8,'weighted_delivery')
    recomputed=dict(transport_energy_kwh=te,relay_energy_kwh=re,total_energy_kwh=te+re,
                    weighted_delivery_min=weighted,min_transport_soc=min(tsoc),min_relay_soc=min(rsoc,default=1),
                    overall_finish_min=max(r['return_time'] for r in solution['transport']+solution['relay'])/60)
    for field,value in recomputed.items():check(abs(solution['metrics'][field]-value)<1e-6,'metric_consistency')
    for kind,fields in [('transport',['uav','battery']),('relay',['uav','component'])]:
        for field in fields:check({r[field] for r in solution[kind]}<={r[field] for r in baseline[kind]},'resource_count')
        check(len(solution[kind])<=len(baseline[kind]),'sorties')
    return dict(passed=not failures,failures=sorted(set(failures)),transport_energy_kwh=te,relay_energy_kwh=re,
                total_energy_kwh=te+re,weighted_delivery_min=weighted)


def objective(solution):
    m=solution['metrics']
    return (m['overall_finish_min']*60,m['weighted_delivery_min'],m['total_energy_kwh'],
            sum(r['service_duration'] for r in solution['relay']),len(solution['relay']))
