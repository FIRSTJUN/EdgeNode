import math
import numpy as np
from edgenode_planning.avoidance_geometry import convex_hull,inflate,swept_body
from edgenode_planning.vehicle_geometry import morai_sonata_collision_envelope


def test_compiled_hull_and_inflation_are_identical_for_all_observed_and_degenerate_solids():
    rng=np.random.default_rng(54419)
    for xy in [[],[(1.,2.)],[(1.,2.),(1.,2.)],[(0.,0.),(5.,0.)],*rng.normal(size=(80,16,2))]:
        points=list(map(tuple,xy));hull=convex_hull(points)
        assert convex_hull(points,native=True)==hull
        for margin in (0.,.1,.25,.3):assert inflate(hull,margin,native=True)==inflate(hull,margin)


def test_compiled_whole_body_sweeps_keep_the_exact_existing_continuous_enclosure():
    rng=np.random.default_rng(54420);body=morai_sonata_collision_envelope()
    for _ in range(160):
        a=tuple(rng.uniform([-300.,1000.,-math.pi],[300.,1800.,math.pi]))
        b=(a[0]+rng.uniform(-2.,2.),a[1]+rng.uniform(-2.,2.),a[2]+rng.uniform(-.2,.2))
        for c in (a,b):assert swept_body(body,a,c,native=True)==swept_body(body,a,c)
