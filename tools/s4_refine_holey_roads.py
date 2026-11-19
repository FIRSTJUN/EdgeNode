"""Extend actual connected MGeo evidence without extending the control path."""
import s4_refine_surveyed_roads as refiner
from s4_partitioned_surveyed_roads import local_patches
from s4_survey_islands import prepare
original=refiner.HDMapBoundaries
def boundary(*args,**kwargs):return prepare(original(*args,**kwargs))
if __name__=='__main__':
    refiner.HDMapBoundaries=boundary;refiner.local_patches=local_patches;refiner.main()
