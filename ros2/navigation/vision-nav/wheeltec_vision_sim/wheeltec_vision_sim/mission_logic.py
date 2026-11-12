"""Fault assertions consume observations, never infer success from injection."""
SCENARIOS=('motion','route','vision','imu','depth','pause','old_goal','obstacle')

def assess_fault(samples,onset,recovery,deadline=.55,require_latch=False):
    during=[s for s in samples if onset+deadline<=s['wall']<=recovery]
    after=[s for s in samples if recovery+.5<=s['wall']<=recovery+1.5]
    def zero(s):return abs(s['safe_v'])<1e-5 and abs(s['safe_w'])<1e-5 and s.get('safe_age_s',1.) is not None and s.get('safe_age_s',1.)<.3
    result=dict(observed=bool(during),safe_zero=bool(during) and all(zero(s) for s in during),
                collisions=sum(s.get('contacts',0) for s in samples))
    result['contact_evidence']=bool(samples) and any(s.get('contacts_observed',False) for s in samples)
    if require_latch:
        result['old_goal_blocked']=bool(after) and all(zero(s) and s.get('new_goal_required',False) for s in after)
    result['passed']=result['observed'] and result['safe_zero'] and result['contact_evidence'] and not result['collisions'] and result.get('old_goal_blocked',True)
    return result

def resource_snapshot(root_pid=None):
    import os
    from pathlib import Path
    from resource import getrusage,RUSAGE_SELF
    rows=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:
            pid=int(entry.name)
            text=(entry/'stat').read_text();rest=text[text.rfind(')')+2:].split()
            rows.append(dict(pid=pid,ppid=int(rest[1]),cpu_ticks=int(rest[11])+int(rest[12]),rss_bytes=int(rest[21])*os.sysconf('SC_PAGE_SIZE')))
        except (OSError,ValueError,IndexError):pass
    usage=getrusage(RUSAGE_SELF)
    selected={root_pid or os.getpid()}
    while True:
        descendants={row['pid'] for row in rows if row['ppid'] in selected}
        if descendants<=selected:break
        selected|=descendants
    return dict(processes=[row for row in rows if row['pid'] in selected],resource_root_pid=root_pid or os.getpid(),clock_ticks_per_second=os.sysconf('SC_CLK_TCK'),
        runner_user_s=usage.ru_utime,runner_system_s=usage.ru_stime,runner_max_rss_kib=usage.ru_maxrss)
