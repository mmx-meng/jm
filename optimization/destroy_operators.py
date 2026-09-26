"""五类结构破坏算子；释放整个架次，防止遗漏强耦合货箱。"""
import math

NAMES=['critical_route','late_box','relay_coupled','battery_chain','spatial_cluster']


def destroy(name,solution,critical,nodes,rng,size=4):
    ts=solution['transport'];byid={t['id']:t for t in ts}
    ranked=[byid[r['sortie']] for r in critical['tasks']]
    if name=='critical_route':
        latest=max(ts,key=lambda t:t['return_time']);chain=sorted([t for t in ts if t['uav']==latest['uav']],key=lambda t:-t['return_time'])
        nearby=[t for t in ts if set(t['order'])&set(latest['order']) and t['id']!=latest['id']]
        p=nodes[latest['order'][0]]
        neighbours=sorted([t for t in ts if not set(t['order'])&set(latest['order'])],
                          key=lambda t:min(math.hypot(nodes[n]['x']-p['x'],nodes[n]['y']-p['y']) for n in t['order']))
        partner=rng.choice(neighbours[:min(6,len(neighbours))])
        priority=chain[:2]+nearby+[partner]+ranked
    elif name=='late_box':
        priority=sorted(ts,key=lambda t:max(t['deliveries'].values()),reverse=True)
    elif name=='relay_coupled':
        r=max(solution['relay'],key=lambda r:r['return_time']+.25*r['service_duration'])
        coupled=[t for t in ts if any(a['site_index']==r['site_index'] for a in t['relay_assignments'])]
        priority=sorted(coupled,key=lambda t:-max(t['deliveries'].values()))+ranked
    elif name=='battery_chain':
        anchor=ranked[rng.randrange(min(4,len(ranked)))];ids={anchor['battery'],anchor['uav']}
        chain=[t for t in ts if t['battery'] in ids or t['uav'] in ids]
        priority=sorted(chain,key=lambda t:abs(t['start']-anchor['start']))+ranked
    elif name=='spatial_cluster':
        anchor=ranked[rng.randrange(min(8,len(ranked)))];p=nodes[anchor['order'][0]]
        def distance(t):
            return min(math.hypot(nodes[s]['x']-p['x'],nodes[s]['y']-p['y']) for s in t['order'])
        priority=sorted(ts,key=lambda t:(distance(t),rng.random()))
    else:raise ValueError(name)
    selected=[]
    for t in priority:
        if t['candidate_id'] not in selected:selected.append(t['candidate_id'])
        if len(selected)>=size:break
    return selected
