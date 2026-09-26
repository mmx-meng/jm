"""释放货箱的跨架次组批与通信感知候选筛选；复用匹配航迹的连续证书。"""
from collections import Counter,defaultdict
from optimization.dynamic_candidates import key


def features(c,sites,models):
    reqs=c['requirements'];eligible=[]
    for req in reqs:eligible.append([i for i in range(len(sites)) if req['mask']&(1<<(i+1))])
    duration=sum(r['end']-r['start'] for r in reqs)
    energy_lb=max((min(sites[i]['flight_energy']+sites[i]['link_energy'] for i in ids) for ids in eligible if ids),default=0)
    return_lb=max((req['end']+min(sites[i]['back'] for i in ids) for req,ids in zip(reqs,eligible) if ids),default=0)
    return dict(communication_gap_intervals=[(r['start'],r['end']) for r in reqs],candidate_relay_positions=eligible,
                minimum_relay_duration=duration,relay_energy_lower_bound=energy_lb,
                relay_return_time_lower_bound=return_lb,
                heuristic=(c['total']+60*c['energy']+.25*return_lb+.05*c['charge'])/sum(c['qty'].values()))


def repair(current,released,candidates,baseline,sites,models,rng,limit=180,joint=True,force_new=False):
    fixed={t['candidate_id']:t for t in current['transport'] if t['candidate_id'] not in released}
    removed=[t for t in current['transport'] if t['candidate_id'] in released]
    quantities=Counter()
    for t in removed:quantities.update({int(k):int(v) for k,v in t['qty'].items()})
    budget=baseline['metrics']['transport_energy_kwh']-sum(t['energy'] for t in fixed.values())
    eligible=[];stats=Counter();byid={c['id']:c for c in candidates};feats={}
    for c in candidates:
        if c['id'] in fixed:continue
        if any(q>quantities[g] for g,q in c['qty'].items()):stats['outside_released_boxes']+=1;continue
        if c['energy']>budget+1e-8 or c['soc']<baseline['metrics']['min_transport_soc']-1e-9:
            stats['soc_energy_pruned']+=1;continue
        if c['total']>baseline['metrics']['overall_finish_min']*60+120:
            stats['time_lower_bound_pruned']+=1;continue
        f=features(c,sites,models)
        if any(not options for options in f['candidate_relay_positions']):stats['communication_pruned']+=1;continue
        feats[c['id']]=f;eligible.append(c)
    # 相同机型/货量/访问顺序的副本只保留合法重复次数。不同通信来源不做伪支配剪枝。
    multiplicity=Counter();pruned=[]
    for c in sorted(eligible,key=lambda c:(c['id'] not in released,feats[c['id']]['heuristic'])):
        signature=(c['model'],tuple(c['order']),tuple(sorted(c['qty'].items())))
        maximum=min(quantities[g]//q for g,q in c['qty'].items())
        if multiplicity[signature]>=maximum:stats['duplicate_dominance_pruned']+=1;continue
        multiplicity[signature]+=1;pruned.append(c)
    # 所有原释放模式保持可选；保留低代价以及随机多样模式。上限是算力预算而非可行性判据。
    incumbents=[byid[i] for i in released];incids=set(released)
    remaining=[c for c in pruned if c['id'] not in incids]
    remaining.sort(key=lambda c:feats[c['id']]['heuristic'])
    if len(remaining)>limit-len(incumbents):
        n=max(0,limit-len(incumbents));good=remaining[:n*2//3]
        diverse=rng.sample(remaining[n*2//3:],n-len(good));remaining=good+diverse
    pool=[byid[i] for i in fixed]+incumbents+remaining
    related={a['site_index'] for t in removed for a in t['relay_assignments']}
    existing={r['site_index'] for r in current['relay']}
    frozen=existing-related
    allowed=set(existing)
    if joint:
        # 所有20个已认证位置均可作为相关会话的替代，非相关会话位置保留。
        allowed.update(range(2*len(sites)))
    else:
        # 分离消融固定既有中继位置；运输改变仍按新候选缺口重新确定服务时段。
        frozen=existing
    oldkeys={key(t) for t in current['transport']}
    context=dict(fixed=fixed,released_sessions=related,allowed_sessions=allowed,
                 fixed_relay_positions=frozen,force_new=force_new,incumbent_ids={c['id'] for c in candidates if key(c) in oldkeys},separated=not joint)
    stats.update(pool_size=len(pool),released_sorties=len(removed),released_boxes=sum(quantities.values()),eligible_before_limit=len(pruned),
                 multistop_candidates=sum(len(c['order'])>1 for c in pool))
    return pool,context,dict(stats),{c['id']:feats.get(c['id'],features(c,sites,models)) for c in pool}
