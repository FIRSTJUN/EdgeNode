"""Independent observer recomputation of B's opt-in normal reference.

Only the experiment subprocess installs this seam. The original generic
trial's exact path assertion remains in force, using the actual read-only
profile pins and fresh map geometry. No runtime certificate or Actor input
is accepted as a replacement for the complete geometry proof.
"""
import json
import os


def install():
    raw=os.environ.get('EDGENODE_B_REFERENCE_VERIFICATION')
    if not raw:return
    settings=json.loads(raw)
    if not settings['enabled']:return
    from edgenode_planning import route_clearance
    from edgenode_planning.surveyed_road_artifact import load_artifact
    from edgenode_planning.dijkstra_planner import DijkstraPlanner
    from edgenode_static20_experiment.normal_clearance import extend_normal_cache
    original=route_clearance.correct_route
    def corrected(route,network,body,*,tracking_margin_m=.15,surveyed_centering=False):
        if not surveyed_centering:raise ValueError('B normal reference requires surveyed centering')
        network.road_corridor=network.boundaries.road_surface_corridor(network)
        network.road_corridor=load_artifact(settings['road_file'],settings['road_sha256'],network)
        # The live B node prepares its configured saved mission once, before
        # callbacks run. An alternate trial goal must not silently expand
        # that preparation to extra links. Unprepared links retain the
        # original correction pipeline and all live guards in both processes.
        planner=DijkstraPlanner('/workspace/local_data/kcity_mgeo')
        warm=planner.plan_route(settings['warm_start_link_id'],settings['warm_node_ids'])
        if warm is None:raise ValueError('B configured warm mission unavailable')
        original(warm,network,body,tracking_margin_m=tracking_margin_m,surveyed_centering=True)
        extend_normal_cache(network,body,tracking_margin_m,
            steering_curvature_limit=settings['steering_curvature_limit'])
        return original(route,network,body,tracking_margin_m=tracking_margin_m,surveyed_centering=True)
    route_clearance.correct_route=corrected
