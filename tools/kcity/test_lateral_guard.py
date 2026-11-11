import math
import pytest
from test_live_test import tool


def test_lateral_estimator_uses_kmh_to_mps_and_keeps_turn_sign():
    assert tool.lateral_accel_estimate({'/ego_vehicle_status':{'speed':36.},'/Imu':{'gyro_z':-.2}})==-2.
    assert tool.lateral_accel_estimate({'/ego_vehicle_status':{'speed':0.},'/Imu':{'gyro_z':.2}})==0.
    assert tool.lateral_accel_estimate({}) is None
    assert tool.lateral_accel_estimate({'/ego_vehicle_status':{'speed':36.},'/Imu':{'gyro_z':math.nan}}) is None


@pytest.mark.parametrize('value',['nan','inf','0','-1','11'])
def test_bad_guard_cannot_start_a_trial(value):
    with pytest.raises(SystemExit):
        tool.parse_args(['--run','--max-speed-kmh','60','--max-lateral-accel-mps2',value])


def test_lateral_guard_does_not_change_the_selected_sixty_ceiling():
    args=tool.parse_args(['--run','--max-speed-kmh','60','--max-lateral-accel-mps2','3.0'])
    assert args.max_speed_kmh==60.
    assert args.max_lateral_accel_mps2==3.


def test_enabled_guard_rejects_a_stale_or_nonfinite_imu():
    latest={'/ego_vehicle_status':{'speed':60.},'/Imu':{'gyro_z':.1,'age':.01}}
    times={'/Imu':100.}
    text='lateral guard requires fresh finite IMU and speed'
    assert text not in tool.immediate_faults(latest,times,100.,60.,3.)
    assert text in tool.immediate_faults(latest,times,100.3,60.,3.)
    latest['/Imu']['gyro_z']=math.nan
    assert text in tool.immediate_faults(latest,times,100.,60.,3.)
    assert text not in tool.immediate_faults(latest,times,100.,60.)
