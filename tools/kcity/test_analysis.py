"""Geometry/time checks for reported driving measurements; no ROS graph."""
import math
import numpy as np
import pytest

from analyze_drive import after_startup, map_projection, steering_metrics


def test_reference_alignment_projects_segments_including_corner_heading():
    result=map_projection([[0,0,0],[10,0,0],[10,10,0]],np.array([[5,1],[12,7]]))
    np.testing.assert_allclose(result,[[5,1,0],[17,2,math.pi/2]])


def test_saturation_duration_and_rates_exclude_unobserved_gaps():
    result=steering_metrics(np.array([0,.05,.1,3.,3.05]),np.array([.7,.7,.6,.7,.7]),.7)
    assert result['duration_sec']==pytest.approx(.15)
    assert result['longest_saturation_sec']==pytest.approx(.1)
    assert result['rate_abs_per_sec']['max']==pytest.approx(2.)
    assert result['total_variation']==pytest.approx(.1)


def test_startup_exclusion_restarts_at_each_drive_enable():
    groups={'/planning/drive_status':[dict(t=t,value=v) for t,v in
        [(0.,'DISABLED'),(1.,'ACTIVE'),(2.,'ACTIVE'),(5.,'DISABLED'),(6.,'ACTIVE')]]}
    assert after_startup(groups,np.array([1.,3.9,4.1,6.,9.1]),3.).tolist()==[False,False,True,False,True]
