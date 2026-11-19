from dataclasses import replace
from test_static_obstacle_planner import setup,enter,acquire


def ready():
    p,point,obstacle=setup();enter(p,point,obstacle);acquire(p,point,3.5)
    p.config=replace(p.config,transition_lengths_m=(20.,24.))
    return p


def test_individually_safe_return_is_rejected_when_whole_maneuver_validator_fails():
    p=ready();calls=[]
    def reject(candidate):calls.append(candidate.transition_length_m);return 'COMPLETE_PP:TEST_REJECTION'
    p._complete_return_validator=reject
    assert p._generate_return((45.,3.5,0.),3.,0.,{})=='NO_SAFE_RETURN'
    assert calls and not p._return_path and p.return_commits==0
    assert all(c.rejection for c in p.candidates)


def test_another_return_length_is_tried_after_whole_maneuver_rejection():
    p=ready();calls=[]
    def validate(candidate):
        calls.append(candidate.transition_length_m)
        return 'COMPLETE_PP:TEST_REJECTION' if len(calls)==1 else ''
    p._complete_return_validator=validate
    assert p._generate_return((45.,3.5,0.),3.,0.,{})==''
    assert len(calls)==2 and p.selected.transition_length_m==calls[-1]
    assert p._return_path and p.candidates[0].rejection=='COMPLETE_PP:TEST_REJECTION'


def test_cached_components_still_require_each_new_whole_maneuver_validation():
    p=ready();p._return_component_cache={}
    p._complete_return_validator=lambda candidate:''
    assert p._generate_return((45.,3.5,0.),3.,0.,{})==''
    previous=p.return_commits;seen=[]
    def reject(candidate):seen.append(candidate);return 'COMPLETE_PP:CHANGED_ENTRY'
    p._complete_return_validator=reject
    assert p._generate_return((45.,3.5,0.),3.,0.,{})=='NO_SAFE_RETURN'
    assert seen and p.return_commits==previous
    assert p._entry_context.get('return_component_cache_hits',0)>0


def test_new_observation_invalidates_cached_return_component_proof():
    p=ready();p._return_component_cache={}
    assert p._generate_return((45.,3.5,0.),3.,0.,{})==''
    previous=p.return_commits
    _,_,obstacle=setup();p.remembered+= (tuple(obstacle(45.,95.,-10.,10.)),)
    assert p._generate_return((45.,3.5,0.),3.,0.,{})=='NO_SAFE_RETURN'
    assert p.return_commits==previous and all(c.rejection for c in p.candidates)


def test_local_lane_probe_reuses_components_but_pickle_drops_scratch_proofs():
    import pickle
    from edgenode_planning.lane_candidate_planner import LaneCandidatePlanner
    p=ready();p.__class__=LaneCandidatePlanner
    p._return_component_cache={};p._complete_offsets={}
    candidate=p.selected
    p._complete_return_proof(candidate,p.pair,3.)
    assert p._return_component_cache
    p._complete_return_proof(candidate,p.pair,3.)
    assert p._entry_context.get('return_component_cache_hits',0)>0
    restored=pickle.loads(pickle.dumps(p))
    assert not restored._return_component_cache
