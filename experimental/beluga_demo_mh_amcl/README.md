# Beluga Multi-Hypotheses AMCL demo

## How the algorithm work

Traditional AMCL is extended by managing multiple populations of particles (each population is a hypothesis). Particles are created, destroyed, or merged based on their consistency with sensory data, choosing the best fitting hypothesis in each iteration.

The observation model is based on geometric transformations and projections (using the tf2 library to compute the map → odom → base_link → base_scan transformations), supporting both 2D (cells in a costmap) and 3D environments (voxels from an octree for example, or elevation costmaps, no evidence about this functionality).

Instead of relying on traditional uncertainty measures from the covariance matrix, the authors propose a new metric (Novel metric) that compares sensory measurements with expected values to assess consistency.

The particle update process is divided into prediction, correction, and reseeding, optimizing computational cost. The prediction phase happens frequently (100Hz), correction less so (10Hz), and reseeding rarely (0.3Hz):
- The prediction phase remains the same as the typical particle filter.
- The correction phase applies the new observation model to update the weights of the particles, using the ‘hits field’ (h) instead of the covariance matrix to compute the quality of the hypothesis.
- In the reseed phase particles with less weight (losers) are replaced by particles close to those with more weight (winners). If the distribution’s covariance is high, the number of particles is increased. If it’s low, it’s decreased to save computational cost.

A quality value is computed (value between 0 and 1) for every set of particles (every hypothesis) and a weight is computed for every particle (this remains the same). Some hypotheses can be merged if they converge to the same pose.

The output of the localization system is still a mean value and a covariance calculated from the highest quality particle set.

The number of hypotheses is parametrized, along with other new concepts like the quality thresholds or the reseeding percentage for winners.

The map-matching is done with 4 levels of resolution to improve computational cost, because the next level is only computed on those cells that are candidates from the previous level. The result is a list with the candidates sorted by quality.

### Steps for managing the hypotheses

1. Start: At the start of the robot operation, P(0,t0) (first set of particles, hypothesis) is started at the robot’s initial position, if known.
2. Creation: Periodically, a cascade map matching algorithm is used to determine which map positions the latest sensory readings could be obtained. If a position with a high match is found (high hit field), a new P(k,t) is started at this position.
3. Destruction: If Quality(P(k,t)) is less than some threshold and |Pt| > 0, then P(k,t) is considered to be wrong, and it is removed.
4. Merge: Even if two P(i,t) and P(j,t), start at different positions, they could end up converging to the same position. In this case, they are mixed in a P(k,t) containing the particles with more weight.

## Useful links

[Original Paper](https://arxiv.org/pdf/2209.07586)
