"""Keep the original continuation/cell proof, with endpoint-partitioned strips."""
import s4_refine_surveyed_roads as refiner
from s4_partitioned_surveyed_roads import local_patches

refiner.local_patches=local_patches
if __name__=='__main__':refiner.main()
