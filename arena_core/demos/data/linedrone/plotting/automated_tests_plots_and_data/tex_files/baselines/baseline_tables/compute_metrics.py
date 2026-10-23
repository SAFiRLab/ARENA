"""Per-run metrics for the baseline table, from the raw benchmark CSVs (load.py), nothing hard-coded:
success rate, planning time, best duration and energy of the front, clearances of the safest solution, and the Pareto
front quality (HV ratio, IGD+) of every run against the reference front of the map (load.pareto_quality)."""
import argparse
import json
from load import *
_parser=argparse.ArgumentParser(description='Per-run metrics of the baseline tables (table_data.json)')
_parser.add_argument('--arena-folder',default='arena_g300_rrt10',help='ARENA runs in baselines/<map>/native_res (variants)')
_parser.add_argument('--output',default='table_data.json',help='JSON written (read by make_compact_tables.py --data)')
ARGS=_parser.parse_args()
configure(ARGS.arena_folder)
NAMES={'ARENA':'ARENA (ours)','MOARkino':'MOAR-3D kino.\\ sweep','KinoAstar':'Fast-Planner~\\cite{zhou2019}','KinoRRT':'Kino.\\ RRT$^*$',
       'Astar':'A$^*$','RRT':'RRT$^*$','MOAR':'MOAR-3D~\\cite{petit2024} sweep','NSGA':'Ahmed and Deb~\\cite{ahmed2011}','Apex':'A$^*$pex~\\cite{zhang2022}','PORRT':'PO-RRT$^*$~\\cite{batonisashvili2026}'}
BUDGET={'PORRT','KinoRRT'}                       # planning time = fixed budget
DETERMINISTIC_CLR={'Apex','MOAR','MOARkino'}     # fronts of many solutions: clearances of the safest one from the export run
SINGLE_PATH={'Astar','RRT'}                      # one solution per run: no front quality

def lst(x): x=np.asarray(x,float); return float(x[0]) if np.ptp(x)<1e-9 else x.tolist()   # identical runs -> scalar

def success(env,p):
    if 'st' not in F[(env,p)]: return 100.0*report(env,p).groupby('Id')['Feasible'].max().mean()
    s=status(env,p); cols=['Completed','Constraints respected','Safe','Physically feasible and safe']
    return 100.0*min((s[c]>0).mean() for c in cols)          # lowest of the shares

out={}
for env in ['s42','cl3']:
    out[env]={}
    for p in NAMES:
        if (env,p) not in F:
            raise SystemExit(f'No data for {p} in {env}')
        d={'succ':round(success(env,p))}
        t=plan_time(env,p) if p in ('MOAR','MOARkino') else report(env,p).groupby('Id')['Planing Time'].first()/1e9
        d['time']=[float(np.median(t)),'budget'] if p in BUDGET else lst(t)
        if 'sol' in F[(env,p)]:
            b=best_of_front(sol(env,p)); d.update({k:lst(b[k]) for k in b})
        else:
            f=front(env,p).groupby('Id'); d['dur']=lst(f['Time cost'].min()); d['energy']=lst(f['Energy cost'].min()/1000)
            if p in DETERMINISTIC_CLR:
                b=best_of_front(export_sol(env,p)); d['minclr'],d['meanclr']=lst(b['minclr']),lst(b['meanclr'])
            else:
                c=report(env,p).groupby('Id')['Closest Obstacle Distance']; d['minclr']=lst(c.min()); d['meanclr']=lst(c.mean())
        if p in SINGLE_PATH: d['hv'],d['igd']=None,None
        else:
            q=pareto_quality(env)[p]; d['hv'],d['igd']=lst(q['hv']),lst(q['igd'])
        d['n_runs']=int(len(report(env,p)['Id'].unique()))
        out[env][NAMES[p]]=d
def tag(v):
    if isinstance(v,list) and len(v)==2 and v[1]=='budget': return {'budget':v[0]}
    if isinstance(v,list): return {'runs':v}                            # per-run (or per-search) values
    return v                                                            # deterministic scalar / None
for env in out:
    for p,d in out[env].items():
        for k in list(d):
            if k not in ('succ','n_runs','src'): d[k]=tag(d[k])
json.dump(out,open(ARGS.output,'w'),indent=1)
for env in out:
    for p,d in out[env].items():
        print(env, f'{p[:16]:16s}', 'runs',d.get('n_runs','-'), 'SR',d['succ'], ' '.join(f'{k}:{"list" if isinstance(v,list) and k!="time" else type(v).__name__}' for k,v in d.items() if k not in('succ','n_runs','src')))
