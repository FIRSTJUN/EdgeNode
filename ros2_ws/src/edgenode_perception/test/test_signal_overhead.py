"""Real MORAI signal returns above the measured roof must not block a lane."""
from dataclasses import replace
import json
from pathlib import Path

import numpy as np

from edgenode_perception.lidar_obstacles import LidarObstacleConfig, detect_clusters


def test_recorded_signal_housings_are_above_the_s4_vehicle_height_envelope():
    data = json.loads((Path(__file__).parent/'fixtures/kcity_signal_overhead_patch.json').read_text())
    overhead = np.asarray(data['overhead_xyz'])
    config = LidarObstacleConfig(roi_min_x=-8., roi_max_x=100., roi_min_y=-40., roi_max_y=40.,
        obstacle_height_filter_enabled=True, obstacle_max_z_m=1., oriented_footprints=True,
        ground_plane_filter_enabled=True)
    assert len(detect_clusters(overhead, config)) == 2
    assert not detect_clusters(overhead, replace(config, obstacle_max_z_m=.3))
    assert overhead[:, 2].min() - data['ego_return_top_z_m'] > .5
    # Keep returns across a low obstacle and the vehicle body/roof envelope.
    for height in (-.8, -.4, .2):
        body = np.array([(10.+i*.1, j*.1, height) for i in range(3) for j in range(3)])
        assert detect_clusters(np.r_[overhead, body], replace(config, obstacle_max_z_m=.3))
