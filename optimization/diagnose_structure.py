"""区分组批静态不可行与资源/通信排程难点；不用诊断模型替代真实调度。"""
import random
from collections import Counter
from ortools.sat.python import cp_model
from core import ROOT,save_json
from run_time_alns import load
from optimization.critical_path import analyze
from optimization.destroy_operators import destroy,NAMES
from optimization.dynamic_candidates import key


def main():
    base,nodes,boxes,models,relay,terrain,groups,sites,cs=load();rng=random.Random(24)
    critical=analyze(base,models,relay);results=[]
    for op in NAMES:
        released=destroy(op,base,critical,nodes,rng,6)
        qty=Counter()
        for t in base['transport']:
            if t['candidate_id'] in released:qty.update({int(k):v for k,v in t['qty'].items()})
        candidates=[c for c in cs if all(q<=qty[i] for i,q in c['qty'].items()) and c['soc']>=base['metrics']['min_transport_soc']-1e-9]
        old={key(t) for t in base['transport']};model=cp_model.CpModel();ys=[model.NewBoolVar(f'y{i}') for i in range(len(candidates))]
        for gi,count in qty.items():model.Add(sum(c['qty'].get(gi,0)*y for c,y in zip(candidates,ys))==count)
        model.Add(sum(ys)<=len(released))
        model.Add(sum(y for c,y in zip(candidates,ys) if key(c) not in old)>=1)
        cost=sum(round(c['energy']*1e6)*y for c,y in zip(candidates,ys));model.Minimize(cost)
        solver=cp_model.CpSolver();solver.parameters.max_time_in_seconds=2;solver.parameters.num_search_workers=1
        status=solver.Solve(model);budget=sum(t['energy'] for t in base['transport'] if t['candidate_id'] in released)
        result=dict(operator=op,released=released,candidate_count=len(candidates),status=solver.StatusName(status),old_energy=budget,
                    scope='仅数量、架次、SOC，放松时间/通信会话/资源，所得下界只能用于诊断')
        if status in [cp_model.FEASIBLE,cp_model.OPTIMAL]:
            result.update(minimum_energy=solver.ObjectiveValue()/1e6,energy_bound=solver.BestObjectiveBound()/1e6,
                          chosen=[c['id'] for c,y in zip(candidates,ys) if solver.Value(y)],
                          static_energy_possible=solver.ObjectiveValue()/1e6<=budget+1e-5)
        results.append(result)
    save_json(ROOT/'results/time_alns/structure_diagnostics.json',results)
    for r in results:print(r,flush=True)


if __name__=='__main__':main()
