"""Use the original all-seed builder with exact endpoint-partitioned strips."""
import sys
import s4_build_surveyed_roads as builder
from s4_partitioned_surveyed_roads import local_patches

builder.local_patches=local_patches
if __name__=='__main__':builder.main()
