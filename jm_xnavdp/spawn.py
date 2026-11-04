"""Seeded pre-episode JM corridor spawn; never resample based on rollout outcome."""
import random
from keti_navigation.planner import line_free, clearance

BOUNDS = (35.4, 15.8, 37.0, 16.8)
SPAWN_MARGIN = .65


def sample_spawn(obstacles, seed, bounds=BOUNDS, max_attempts=256):
    if not isinstance(seed, int) or not 0 <= seed < 2**32:
        raise ValueError('Seed must be a uint32 integer')
    if len(bounds)!=4 or bounds[0]>=bounds[2] or bounds[1]>=bounds[3] or max_attempts<1:
        raise ValueError('Invalid spawn region')
    rng=random.Random(seed)
    rejected=[]
    for attempt in range(max_attempts):
        xy=[rng.uniform(bounds[0],bounds[2]),rng.uniform(bounds[1],bounds[3])]
        if line_free(xy,xy,obstacles,SPAWN_MARGIN):
            return dict(mode='random_corridor',seed=seed,selected_xy=xy,bounds_xy=list(bounds),
                        margin_m=SPAWN_MARGIN,obstacle_distance_m=clearance(xy,obstacles),
                        candidate_count=attempt+1,rejected_xy=rejected,
                        selection_basis='pre-episode geometry only; no policy outcome filtering',
                        yaw_randomized=False,initialization_only=True)
        rejected.append(xy)
    raise RuntimeError('NO_VALID_SPAWN: keep bounds/goals unchanged, do not start episode')
