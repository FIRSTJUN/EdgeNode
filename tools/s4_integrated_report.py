#!/usr/bin/env python3
"""Summarize completed, independently restored MORAI trials; never a live bag."""
import csv
import hashlib
import json
import statistics
import subprocess
from pathlib import Path


ROOT=Path('/workspace/local_data/s4_integrated_lattice_20261117')


def completed_trial(path):
    summary=json.loads((path/'summary.json').read_text())
    rows=[json.loads(s) for s in (path/'samples.jsonl').read_text().splitlines()]
    audit=json.loads((path/'live_audit.json').read_text()) if (path/'live_audit.json').exists() else {}
    spec=summary.get('trial_spec',{})
    entry=detect=None;previous=None;stops=0;stop_seconds=0.;reasons={};strategies=set();moving_armed=False
    errors=[];speeds=[];times=[];targets=[];xy=[];last_ns=None;validation=[];natural=returns=0
    for row in rows:
        t=row.get('t',row.get('elapsed_s',0.));v=row['values'];d=v.get('/planning/avoidance_debug',{})
        e=v.get('/ego_vehicle_status',{});speed=e.get('speed_kmh',0.);target=v.get('/planning/target_speed',0.)
        state=d.get('state');stage=d.get('stage');active=stage in ('AVOID_ENTRY','AVOID_HOLD','RETURN')
        if detect is None and state=='DETECT':detect=t
        if entry is None and state=='AVOID_ENTRY':entry=t
        if entry is not None and active and previous is not None:
            if speed>1.:moving_armed=True
            if speed<.2 and moving_armed:stops+=1;moving_armed=False
            if target==0.:stop_seconds+=max(0.,t-previous[0])
        key=(state,d.get('reason'))
        if previous is None or key!=previous[2]:
            if state in ('SENSOR_HOLD','BLOCKED'):reasons[str(key)]=reasons.get(str(key),0)+1
        previous=(t,speed,key)
        strategy=d.get('selected_strategy')
        if strategy:strategies.add(strategy)
        natural=max(natural,d.get('natural_rejoins',0));returns=max(returns,d.get('return_commits',0))
        if d.get('validation_elapsed_ms') is not None:validation.append(d['validation_elapsed_ms'])
        if '/control/path_error' in v:errors.append(v['/control/path_error'])
        times.append(t);speeds.append(speed);targets.append(target)
        if 'x' in e:xy.append((e['x'],e['y']))
    passed=bool(summary.get('success'))
    verification=summary.get('verification','MORAI_LIVE')
    row=dict(trial=path.name,verification=verification,approach_kmh=spec.get('approach_speed_kmh',60),
        follow_distance_m=spec.get('follow_distance_m'),obstacle_yaw_deg=spec.get('obstacle_yaw_deg'),
        path_completed=passed and verification=='MORAI_LIVE',continuous_pass=passed and stops==0 and not reasons and verification=='MORAI_LIVE',
        error=summary.get('error',''),restored=summary.get('restored',summary.get('stop_confirmed')),
        collision_samples=sum(bool(r['values'].get('/collision_data')) for r in rows),
        detection_to_entry_s=entry-detect if entry is not None and detect is not None else None,
        stops_after_entry=stops,zero_target_after_entry_s=stop_seconds,
        max_tracking_error_m=max(errors,default=None),mean_tracking_error_m=statistics.mean(errors) if errors else None,
        peak_speed_kmh=max(speeds,default=None),natural_rejoins=natural,return_commits=returns,
        selected_strategies=';'.join(sorted(strategies)),
        validation_p95_ms=sorted(validation)[int(.95*(len(validation)-1))] if validation else None,
        minimum_observed_lidar_gap_m=audit.get('minimum_observed_lidar_swept_gap_m'),
        minimum_actor_gap_m=audit.get('minimum_static_actor_box_swept_gap_m'),
        forbidden_boundary_contacts=audit.get('forbidden_boundary_contact_samples'),
        stop_reasons=json.dumps(reasons,ensure_ascii=False))
    if rows:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,(ax,bx)=plt.subplots(1,2,figsize=(12,4))
        if xy:ax.plot(*zip(*xy),label='MORAI Ego')
        global_path=path/'global-path.json'
        if global_path.exists() and xy:
            ref=json.loads(global_path.read_text());xmin=min(p[0] for p in xy)-15;xmax=max(p[0] for p in xy)+15
            ymin=min(p[1] for p in xy)-15;ymax=max(p[1] for p in xy)+15
            ax.plot([p[0] for p in ref],[p[1] for p in ref],alpha=.5,label='Saved global path')
            ax.set(xlim=(xmin,xmax),ylim=(ymin,ymax))
        ax.set(xlabel='Map X (m)',ylabel='Map Y (m)',title=path.name);ax.legend();ax.set_aspect('equal',adjustable='box')
        bx.plot(times,speeds,label='Actual');bx.step(times,targets,label='Target',where='post')
        bx.set(xlabel='Time (s)',ylabel='Speed (km/h)');bx.legend();bx.grid(alpha=.3)
        fig.tight_layout();fig.savefig(path/'trajectory-speed.png',dpi=160);plt.close(fig)
    return row


def main():
    rows=[completed_trial(p.parent) for p in sorted(ROOT.glob('*/summary.json'))]
    if not rows:return
    with (ROOT/'experiment_summary.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    comparison=[]
    by_name={r['trial']:r for r in rows}
    before=by_name.get('phaseA10_lane_cruise')
    for name,after in by_name.items():
        if before and name.startswith('phaseA10_') and name!=before['trial']:
            for metric in ('detection_to_entry_s','stops_after_entry','zero_target_after_entry_s','max_tracking_error_m','validation_p95_ms'):
                comparison.append(dict(before=before['trial'],after=name,metric=metric,
                    before_value=before[metric],after_value=after[metric],
                    scope='Same link/start/station; independent live runs; compare failure status as well'))
    with (ROOT/'before_after_comparison.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=['before','after','metric','before_value','after_value','scope']);w.writeheader();w.writerows(comparison)
    files=subprocess.check_output(['git','diff','--name-only'],cwd='/workspace',text=True).splitlines()
    files+=subprocess.check_output(['git','ls-files','--others','--exclude-standard'],cwd='/workspace',text=True).splitlines()
    manifest={s:hashlib.sha256((Path('/workspace')/s).read_bytes()).hexdigest() for s in files if (Path('/workspace')/s).is_file()}
    (ROOT/'changed_files.json').write_text(json.dumps(manifest,indent=2)+'\n')
    branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip()
    head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
    report=['# 통합 Lattice 개발 진행 기록','',f'브랜치: `{branch}` / 기준 커밋: `{head}`','',
        '완료 판정: 전체 요구사항 미완료. 아래 실주행 통과 여부와 연속 주행 여부를 따로 기록한다.',
        '', '## 실제 MORAI 검증','',
        '| 실험 | 경로 통과 | 연속 주행 | 진입 후 정지 | 최대 추종 오차(m) | 실패 사유 |',
        '|---|---|---|---:|---:|---|']
    for r in rows:
        report.append(f"| {r['trial']} | {r['path_completed']} | {r['continuous_pass']} | {r['stops_after_entry']} | {r['max_tracking_error_m']} | {r['error']} |")
    report+=['','## 확인한 원인과 변경','',
        '- 기존 장면 기준선은 회피 후 센서 HOLD와 재검증으로 120초 안에 복귀하지 못했다.',
        '- 명확한 차선에서는 단일 판단기가 LANE_STATIC만 실행한다. 유효한 진입 경로를 고정하고 후속 MGeo 중심선을 추종하며 실제 그래프 연결로 자연 합류한다.',
        '- 차선 끝/분기 전 복귀 탐색을 별도 프로세스에서 수행하고 현재 경로를 최신 관측으로 검증하며 유지한다.',
        '- 첫 차선 유지 실험에서 장애물과 상태는 신선했으나 KEEP_LAST(1) 두 토픽의 짝이 손실되어 센서 시간 초과가 반복됐다. 큐 확대 시험은 오래된 콜백 누적으로 실패했다.',
        '- MORAI 연속 주행 모드에서 상태와 footprint를 단일 프레임으로 전달한다. digest, 출처 시간, 재시작, fault history, 통신 유실 검사를 유지한다.',
        '- 정상 주행 속도 계획에 곡률과 제동 거리를 반영하는 추종 가속도 예산을 추가했다. 직선 60km/h 목표를 보존한다.',
        '', '## 오프라인 검증과 미검증 구분','',
        '- XML/TXT 테스트 기록은 오프라인 검증이다. 실제 MORAI 성공을 대신하지 않는다.',
        '- LiDAR 연결 성분을 타일과 분리해 추적하며 /planning/traffic_tracks에 속도·신뢰도를 발행하는 진단 기능을 추가했다. 동적 주행 정책은 아직 비활성화 상태이며, 추월/빠른 후방 차량 합류 구현 및 실주행은 미완료다.',
        '- 10 및 15km/h 차선 유지와 자연 합류, 새 장애물 전방 38.9m에서 배경 복귀 탐색, 차선 끝 전 복귀를 각각 실주행으로 통과했다. 해당 성공 시험의 회피 구간 목표 속도 0 지속 시간은 0초다.',
        '- S자 340m 구간은 NORMAL 설정 30/40/50/60 조건을 실제 주행했다. 60 설정 최고 실측은 55.5km/h였으며, 커브에서는 선행 감속한다. 설정 속도를 구간 전체 실측 속도라고 해석하지 않는다.',
        '- 교차로 Offset은 원본 A3 단일 도로면과 검증 B2 구간을 사용한다. 구멍/미관측 영역을 채우지 않는다. 실패 시험을 보존하며 아직 실주행 성공으로 판정하지 않는다.',
        '- 전체 18개 경유지 연속 주행, 다른 회피 차선 탐색과 복합 동적 상황은 별도 검증이 필요하다.',
        '- source-time 짝 맞춤용 입력 콜백을 계산과 분리했다. MORAI 전용 옵션은 일시적 시각 정렬 실패에서 기존 관측의 원래 0.6초 기한 안에서만 유지하며, 기한 갱신·빈 도로 처리·잘못된 데이터 무시는 하지 않는다.',
        '- 도로면 검사는 동일 cell union을 GEOS/Shapely 1.8.5.post1로 계산한다. 동적 장애물 검사를 캐시하지 않으며, 원본 순수 Python 검사는 기본 경로에 남아 있다.',
        '- road_area_outside_samples는 해당 감사 도구의 지도 증명 범위를 벗어난 표본 수다. 물리적 도로 이탈 횟수로 해석하지 않는다.',
        '- 충돌은 MORAI collision_data로, 금지 경계 접촉은 독립 차체 sweep 감사로 기록한다.',
        '', '## 산출물','',
        '- `experiment_summary.csv`, `before_after_comparison.csv`: 실험별 수치와 동일 구간 비교.',
        '- 각 실험 폴더: summary, JSONL, 닫힌 rosbag, 경로와 속도 그림. 실패 로그도 보존한다.',
        '- `changed_files.json`: 미커밋/신규 변경 파일 SHA-256.',
        '', '## 다음 우선순위','',
        '1. 교차로 Offset 배치에서 source-time 처리와 RViz 후보 발행 지연을 제거하고 실제 통과 검증.',
        '2. LiDAR 추적 오차 검증 및 시간 궤적을 이용한 추월/합류 판단 구현.',
        '3. 원래 차선도 막힌 재회피와 다양한 배치 검증.',
        '4. 복합 배치와 전체 저장 경로 연속 주행.']
    (ROOT/'report_ko.md').write_text('\n'.join(report)+'\n')
    print(json.dumps(rows,ensure_ascii=False))


if __name__=='__main__':main()
