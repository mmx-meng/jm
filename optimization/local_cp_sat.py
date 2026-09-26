"""局部结构精确修复；未释放架次固定组批、实体资源和相对先后。"""
from optimizer_v2 import solve_v2
from ortools.sat.python import cp_model


def transport_first(pool,groups,models,baseline,context,seconds,seed):
    """分离消融：先只选运输结构，随后固定它再做通信/资源排程。"""
    model=cp_model.CpModel();ys=[model.NewBoolVar(f'x{i}') for i in range(len(pool))]
    for g in groups:model.Add(sum(c['qty'].get(g['index'],0)*y for c,y in zip(pool,ys))==g['count'])
    for c,y in zip(pool,ys):
        if c['id'] in context['fixed']:model.Add(y==1)
        if c['soc']<baseline['metrics']['min_transport_soc']-1e-9:model.Add(y==0)
    model.Add(sum(ys)<=baseline['metrics']['transport_sorties'])
    model.Add(sum(round(c['energy']*1e6)*y for c,y in zip(pool,ys))<=round(baseline['metrics']['transport_energy_kwh']*1e6))
    lower=model.NewIntVar(0,50000,'transport_workload_lb')
    for name,m in models.items():model.Add(lower*len(m.units)>=sum(c['total']*y for c,y in zip(pool,ys) if c['model']==name))
    model.Minimize(lower);solver=cp_model.CpSolver();solver.parameters.max_time_in_seconds=seconds
    solver.parameters.num_search_workers=1;solver.parameters.random_seed=seed;status=solver.Solve(model)
    if status not in [cp_model.OPTIMAL,cp_model.FEASIBLE]:return None
    selected=[c for c,y in zip(pool,ys) if solver.Value(y)]
    return selected,{**context,'mandatory_ids':{c['id'] for c in selected},'force_new':False}


def solve_local(pool,groups,models,relay,sites,baseline,current,context,seconds,seed,protect_weighted=True):
    if context.get('separated'):
        stage=transport_first(pool,groups,models,baseline,context,min(1.,seconds/3),seed)
        if stage is None:return dict(feasible=False,status='TRANSPORT_STAGE_INFEASIBLE')
        pool,context=stage
    kwargs=dict(candidates=pool,groups=groups,models=models,relay=relay,base_sites=sites,
                pareto_reference=baseline,protect_individual_boxes=False,protect_weighted_delivery=protect_weighted,
                local_context=context,random_seed=seed,workers=1,quiet=True,relay_rounding_screen_kwh=.002,weighted_rounding_screen_s=2.)
    bounds={'delay':0,'makespan':int(baseline['metrics']['overall_finish_min']*60)+120}
    result=solve_v2(**kwargs,seconds=seconds,mode='makespan',bounds=bounds,hint=current)
    if not result.get('feasible'):return result
    stages=[dict(mode='makespan',status=result['status'],value=result['objective'],wall_s=result['wall_s'])]
    bounds['makespan']=result['optimization_values']['makespan']
    # 词典序次级修复不改变主目标上界；时间预算独立记入日志。
    refined=solve_v2(**kwargs,seconds=min(1.,seconds/3),mode='weighted_arrival',bounds=bounds,hint=result)
    if refined.get('feasible'):
        result=refined;stages.append(dict(mode='weighted_arrival',status=result['status'],value=result['objective'],wall_s=result['wall_s']))
    result['local_stages']=stages
    return result
