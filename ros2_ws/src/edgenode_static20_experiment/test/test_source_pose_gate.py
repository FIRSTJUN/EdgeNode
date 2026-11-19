from types import SimpleNamespace
from builtin_interfaces.msg import Time
from edgenode_localization.localization_node import LocalizationNode
from edgenode_static20_experiment.source_time_localization import SourceTimeLocalizationNode

def test_only_the_actual_filter_epoch_is_published_and_all_three_sources_must_be_fresh(monkeypatch):
    now=10**12;emitted=[]
    monkeypatch.setattr(LocalizationNode,'_publish_ekf_odometry',lambda self,s:emitted.append(s.sec*10**9+s.nanosec))
    n=object.__new__(SourceTimeLocalizationNode)
    n.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=now))
    n.ekf=SimpleNamespace(last_output_ns=now)
    fields=['_valid_gps_stamp_ns','_source_imu_ns','_source_velocity_ns']
    for f in fields:setattr(n,f,now)
    stamp=Time(sec=now//10**9,nanosec=now%10**9)
    n._publish_ekf_odometry(stamp);assert emitted==[now]
    for f in fields:
        setattr(n,f,now-600_000_001);n._publish_ekf_odometry(stamp)
        assert emitted==[now] and n._pose_source_reason=='SOURCE_EPOCH_STALE_OR_UNAVAILABLE'
        setattr(n,f,now)
    n.ekf.last_output_ns=now-1;n._publish_ekf_odometry(stamp)
    assert emitted==[now] and n._pose_source_reason=='FILTER_EPOCH_NOT_CURRENT_IMU'
