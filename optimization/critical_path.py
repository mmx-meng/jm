"""按航行、通信保障、实体资源和充电先后建立事件DAG及剩余路径松弛。"""
from collections import defaultdict
import heapq
from core import charge_time,save_json
from validate import csv_write


def analyze(solution,models,relay,out=None):
    nodes={};edges=[];resource=defaultdict(list)
    def node(key,time,owner,kind):nodes[key]=dict(time=float(time),owner=owner,kind=kind)
    def edge(a,b,lag,kind):edges.append(dict(source=a,target=b,lag_s=float(lag),kind=kind))
    for t in solution['transport']:
        tid=t['id'];node(tid+':start',t['start'],tid,'transport_start')
        stamps=[(t['start'],tid+':start')]
        node(tid+':takeoff',t['takeoff'],tid,'takeoff');stamps.append((t['takeoff'],tid+':takeoff'))
        for a in t['relay_assignments']:
            for suffix,offset in [('gap_start',a['relative_start']),('gap_end',a['relative_end'])]:
                key=f'{tid}:{suffix}:{a["requirement_index"]}'
                node(key,t['start']+offset,tid,suffix);stamps.append((t['start']+offset,key))
        node(tid+':return',t['return_time'],tid,'transport_return');stamps.append((t['return_time'],tid+':return'))
        stamps.sort()
        for a,b in zip(stamps,stamps[1:]):edge(a[1],b[1],b[0]-a[0],'flight_or_service')
        resource[t['uav']].append((t['start'],tid+':start',tid+':return',0.))
        resource[t['battery']].append((t['start'],tid+':start',tid+':return',charge_time(t['soc'],models[t['model']].full_charge)))
    rmap={r['site_index']:r for r in solution['relay']}
    for r in solution['relay']:
        rid=r['id']
        for suffix,key in [('start','start'),('ready','ready'),('end','service_end'),('return','return_time')]:
            node(rid+':'+suffix,r[key],rid,'relay_'+suffix)
        edge(rid+':start',rid+':ready',r['ready']-r['takeoff']+relay['prep'],'relay_outbound_and_link')
        edge(rid+':ready',rid+':end',0,'relay_service_nonnegative')
        edge(rid+':end',rid+':return',r['return_time']-r['service_end'],'relay_return')
        resource[r['uav']].append((r['start'],rid+':start',rid+':return',relay['turn']))
        resource[r['component']].append((r['start'],rid+':start',rid+':return',charge_time(r['soc'],relay['full_charge'])))
    for t in solution['transport']:
        for a in t['relay_assignments']:
            rid=rmap[a['site_index']]['id'];i=a['requirement_index']
            edge(rid+':ready',f'{t["id"]}:gap_start:{i}',0,'communication_ready')
            edge(f'{t["id"]}:gap_end:{i}',rid+':end',0,'communication_required_until')
    for rid,rows in resource.items():
        rows.sort()
        for a,b in zip(rows,rows[1:]):edge(a[2],b[1],a[3],'charge' if 'BAT' in rid or 'ENE' in rid else 'resource_reuse')
    cmax=solution['metrics']['overall_finish_min']*60;node('finish',cmax,'system','finish')
    for t in solution['transport']+solution['relay']:edge(t['id']+':return','finish',0,'completion')
    outgoing=defaultdict(list);degree={k:0 for k in nodes}
    for e in edges:outgoing[e['source']].append(e);degree[e['target']]+=1
    ready=[k for k,d in degree.items() if d==0];heapq.heapify(ready);order=[]
    while ready:
        k=heapq.heappop(ready);order.append(k)
        for e in outgoing[k]:
            degree[e['target']]-=1
            if degree[e['target']]==0:heapq.heappush(ready,e['target'])
    if len(order)!=len(nodes):raise AssertionError('资源/通信事件图存在循环')
    remaining={k:0. for k in nodes};successor={}
    for k in reversed(order):
        if outgoing[k]:
            best=max(outgoing[k],key=lambda e:e['lag_s']+remaining[e['target']])
            remaining[k]=best['lag_s']+remaining[best['target']];successor[k]=best['target']
        nodes[k]['slack_s']=max(0.,cmax-nodes[k]['time']-remaining[k])
        nodes[k]['remaining_path_s']=remaining[k]
    tasks=[]
    for t in solution['transport']:
        own=[k for k in nodes if nodes[k]['owner']==t['id']]
        key=min(own,key=lambda k:nodes[k]['slack_s']);chain=[key]
        while chain[-1] in successor:chain.append(successor[chain[-1]])
        slack=nodes[key]['slack_s']
        tasks.append(dict(sortie=t['id'],candidate_id=t['candidate_id'],boxes=';'.join(t['boxes']),
                          uav=t['uav'],battery=t['battery'],services='>'.join(t['order']),start_s=t['start'],
                          return_s=t['return_time'],slack_s=slack,critical=slack<=1.,
                          relay_sessions=','.join(str(a['site_index']) for a in t['relay_assignments']),influence_chain=' -> '.join(chain)))
    tasks.sort(key=lambda t:t['slack_s'])
    res=[]
    for rid,rows in resource.items():
        res.append(dict(resource=rid,slack_s=min(nodes[r[1]]['slack_s'] for r in rows),
                        task_chain=' -> '.join(nodes[r[1]]['owner'] for r in sorted(rows))))
    result=dict(makespan_s=cmax,events=nodes,edges=edges,tasks=tasks,resources=res,
                scope='当前结构及资源顺序下的事件剩余路径松弛；不是全局最优性下界')
    if out:
        save_json(out/'critical_path.json',result);csv_write(out/'critical_tasks.csv',tasks);csv_write(out/'resource_slack.csv',res)
    return result
