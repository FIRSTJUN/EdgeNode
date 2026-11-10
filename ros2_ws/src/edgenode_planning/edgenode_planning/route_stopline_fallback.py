"""Explicit next-route-crossing fallback, separate from verified signal bindings."""
import math

from .stop_line_geometry import crossings


MODE = 'ROUTE_STOPLINE_FALLBACK'


def associations_for_route(config, route, route_node_ids):
    """Recompute arc for the actual route; catalog arcs are never reused.

    The catalog contains real NGII marking IDs and local XY geometry. Signal
    IDs are candidates only: ego ROI is a heuristic, not pixel-to-map proof.
    """
    if config.get('schema') != 3 or config.get('fallback_enabled') is not True:
        raise ValueError('FALLBACK_NOT_EXPLICITLY_ENABLED')
    if list(route_node_ids or []) != config['route_node_ids']:
        raise ValueError('FALLBACK_WAYPOINT_MISMATCH')
    nodes = route['node_ids']
    cursor = 0
    for waypoint in config['route_node_ids']:
        try:cursor = nodes.index(waypoint, cursor) + 1
        except ValueError:raise ValueError('FALLBACK_ROUTE_NODE_MISMATCH') from None
    if route['goal_node_id'] != config['route_node_ids'][-1]:
        raise ValueError('FALLBACK_ROUTE_GOAL_MISMATCH')
    entries = []
    for marking in config['stop_lines']:
        links = set(marking['approach_link_ids']).intersection(route['link_ids'])
        if not links:
            continue
        for hit in crossings(route['points'], marking['stop_line_xy']):
            delta = hit['route_heading_rad'] - marking['reference_route_heading_rad']
            # Directed crossing: an opposite-direction traversal is unrelated.
            if abs(math.atan2(math.sin(delta), math.cos(delta))) > math.pi / 4:
                continue
            entries.append(dict(config['ego_roi'],
                stop_line_id=marking['stop_line_id'], signal_id=None,
                association_id=MODE, controlled_link_id=sorted(links)[0],
                movement='route_color_fallback', stop_line_xy=marking['stop_line_xy'],
                stop_arc_m=hit['arc_m'], route_heading_rad=hit['route_heading_rad'],
                related_nodes=marking['related_nodes'],
                map_signal_candidates=marking['map_signal_candidates'],
                front_offset_m=0., buffer_m=config['stop_policy']['reference_stop_margin_m'],
                clearance_m=0., approach_m=config['stop_policy']['approach_m'],
                max_heading_error_deg=45.))
    if not entries:
        raise ValueError('NO_FORWARD_ROUTE_STOPLINE_CROSSINGS')
    return entries


def entry_key(entry):
    """Real ID plus route occurrence: repeated roads remain separate stops."""
    return (entry.get('stop_line_id') or entry['signal_id'], entry['stop_arc_m'])
