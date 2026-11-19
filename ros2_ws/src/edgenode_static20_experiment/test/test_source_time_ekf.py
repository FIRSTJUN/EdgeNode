import math
import numpy as np
import pytest
from edgenode_localization.ekf import EKF
from edgenode_static20_experiment.source_time_ekf import SourceTimeEKF

T=10**12
def make(history=.8):
    f=SourceTimeEKF(EKF(gps_position_std_m=.05),history_sec=history)
    f.begin('imu',T);f.initialize(0.,0.,0.,v=10.);f.end();return f
def imu(f,s,yaw=0.,gyro=0.):
    f.begin('imu',T+round(s*1e9),gyro=gyro)
    f.predict(gyro,.1);f.update_yaw(yaw);f.end()
def gps(f,s,x,y=0.):
    f.begin('gps',T+round(s*1e9));f.update_position(x,y);f.end()

def test_future_gps_is_not_applied_before_its_epoch_or_double_propagated():
    f=make();imu(f,.1);gps(f,.15,1.5)
    assert f.last_output_ns==T+100_000_000 and f.get_state()[0]==pytest.approx(1.)
    imu(f,.2)
    assert f.get_state()[0]==pytest.approx(2.,abs=1e-8)
    assert f.last_output_ns==T+200_000_000

def test_delayed_gps_replay_matches_source_ordered_processing():
    ordered=make();late=make()
    for f in (ordered,late):imu(f,.1,.02,.2)
    gps(ordered,.15,1.5,.01);imu(ordered,.2,.04,.2)
    imu(late,.2,.04,.2);gps(late,.15,1.5,.01)
    for f in (ordered,late):imu(f,.3,.06,.2)
    np.testing.assert_allclose(late.get_state(),ordered.get_state(),atol=1e-12)
    np.testing.assert_allclose(late.get_covariance(),ordered.get_covariance(),atol=1e-12)

def test_vehicle_velocity_is_a_timestamped_measurement_without_position_truth():
    f=make();imu(f,.1);old=f.get_state().copy()
    f.vehicle_velocity(T+150_000_000,5.)
    np.testing.assert_array_equal(f.get_state(),old)
    imu(f,.2)
    assert f.get_state()[3]<5.1 and f.last_output_ns==T+200_000_000

def test_duplicates_do_not_reapply_measurement_information():
    f=make();imu(f,.1);f.vehicle_velocity(T+150_000_000,5.)
    f.vehicle_velocity(T+150_000_000,6.)
    imu(f,.2);before=f.get_state().copy();cov=f.get_covariance().copy()
    imu(f,.2)
    np.testing.assert_allclose(f.get_state(),before,atol=1e-12)
    np.testing.assert_allclose(f.get_covariance(),cov,atol=1e-12)

def test_history_is_bounded_and_old_measurements_do_not_move_the_current_state():
    f=make(history=.3)
    for i in range(1,21):imu(f,i*.1)
    before=f.get_state().copy();gps(f,.1,100.)
    assert f.dropped_before_history==1 and len(f.events)<=5
    np.testing.assert_array_equal(f.get_state(),before)
    imu(f,2.1);assert f.get_state()[0]==pytest.approx(21.)

def test_source_validation_yaw_wrap_and_covariance_remain_finite():
    f=make()
    with pytest.raises(ValueError):f.begin('imu',0)
    with pytest.raises(ValueError):f.vehicle_velocity(T,float('nan'))
    for i in range(1,31):
        f.vehicle_velocity(T+round(i*.1e9),8.)
        imu(f,i*.1,math.atan2(math.sin(i*.2),math.cos(i*.2)),2.)
    cov=f.get_covariance()
    assert np.isfinite(f.get_state()).all() and -math.pi<=f.get_state()[2]<math.pi
    np.testing.assert_allclose(cov,cov.T,atol=1e-12)
    assert np.linalg.eigvalsh(cov).min()>-1e-12

def test_stationary_source_epoch_reset_discards_the_previous_spatial_history():
    f=make();imu(f,.1);gps(f,.15,1.5);imu(f,.2)
    f.begin('gps',T+300_000_000);f.initialize(100.,200.,.5,v=0.);f.end()
    assert not f.events and not f.snapshots
    imu(f,.4,.5,0.)
    np.testing.assert_allclose(f.get_state(),[100.,200.,.5,0.],atol=1e-10)
