from types import SimpleNamespace
from unittest.mock import patch
import pytest
from edgenode_planning.entry_search import EntrySearch
from edgenode_planning.return_search import ReturnSearch
from edgenode_planning.search_budget import check,SearchBudgetExceeded


def test_budget_failure_is_bounded_uncertainty_and_does_not_leak_into_live_validation():
    snapshot=SimpleNamespace(generated_transition_count=0,_worker_search_budget_sec=.1,_entry_context={})
    snapshot._search_entry=lambda:check(snapshot)
    snapshot._block=lambda reason:dict(blocked=True,reason=reason)
    with patch('time.perf_counter',side_effect=[1.,2.,2.]):
        result,_,generated,elapsed=EntrySearch._search(snapshot,())
    assert result==dict(blocked=True,reason='ENTRY_SEARCH_BUDGET_EXHAUSTED')
    assert generated==0 and elapsed==1000.
    assert snapshot._entry_context['search_budget']['classification']=='COMPUTATION_LIMIT_NOT_PHYSICAL_IMPOSSIBILITY'
    assert snapshot._search_deadline_sec is None
    check(snapshot)


def test_return_budget_is_not_adopted_as_a_valid_return():
    snapshot=SimpleNamespace(generated_transition_count=0,_worker_search_budget_sec=.1)
    snapshot._generate_return=lambda *args:check(snapshot)
    with patch('time.perf_counter',side_effect=[1.,2.,2.]):
        reason,*_=ReturnSearch._search(snapshot,(0.,0.,0.),10.,0.)
    assert reason=='RETURN_SEARCH_BUDGET_EXHAUSTED'
    assert snapshot._search_deadline_sec is None
