from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s4_final_mission_audit import unexpected_empty_stamps


def test_same_cycle_explicit_stop_survives_cross_topic_receipt_order():
    assert not unexpected_empty_stamps([(1_000_000_000,True),(2_000_000_000,True)],
        [1_003_000_000,1_998_000_000])


def test_old_stop_cannot_excuse_an_empty_moving_path():
    assert unexpected_empty_stamps([(1_000_000_000,True)], [900_000_000])==[1_000_000_000]
    assert unexpected_empty_stamps([(1_000_000_000,True)], [])==[1_000_000_000]


def test_an_already_explicit_safety_state_stays_separate_from_unexplained_loss():
    assert unexpected_empty_stamps([(1_000_000_000,False),(2_000_000_000,True)], [])==[2_000_000_000]
