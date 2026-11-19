"""Cooperative worker budget; exhaustion is uncertainty, never infeasibility."""
import time


class SearchBudgetExceeded(RuntimeError):
    pass


def check(planner):
    deadline=getattr(planner,'_search_deadline_sec',None)
    if deadline is not None and time.perf_counter()>=deadline:
        raise SearchBudgetExceeded('SEARCH_BUDGET_EXHAUSTED')
