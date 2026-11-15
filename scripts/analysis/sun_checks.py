"""Pairing of /sinter/sun_error messages into sun checks.

A correction rotates the lens about the focal point, so a reading that follows an over-tolerance reading with the
focal point (nearly) where it was is a re-check after a correction ("post"); a reading after a waypoint move is the
first check at a new waypoint. With FK (times + focal positions) this works for any waypoint spacing.

Without FK, fall back to timing: Trial 2 (line ends only) re-checked 1.5-12 s after a check while the next check came
~70 s later. That fallback is wrong when waypoints are seconds apart (Trial 11: line_subdivisions, ~7.5 s per
waypoint), so pass FK whenever it is available. NaN readings (no blob, e.g. a cloud) are never used as anchors.
"""
import math

import numpy as np

RECHECK_GAP_S = 20.0
STILL_MM = 2.0
TOL_RAD = 0.002


def post_correction_indices(se, t_fk=None, pos_fk=None, gap_s=RECHECK_GAP_S, still_mm=STILL_MM, tol_rad=TOL_RAD):
    """Indices of readings that re-check right after a correction. `se` rows are (t, ..., err_rad at [5]).
    t_fk / pos_fk: FK sample times and focal-point positions in metres (N x 3), if available."""
    if t_fk is not None:
        t_fk = np.asarray(t_fk)
        pos_fk = np.asarray(pos_fk)

        def pos(x):
            return pos_fk[int(np.clip(np.searchsorted(t_fk, x), 0, len(t_fk) - 1))]

        # A re-check follows an over-tolerance reading (it triggered the correction) with no waypoint move. The
        # correction itself shifts the FK focal point ~1-1.5 mm (Trial 2); waypoint moves are >= 2.4 mm.
        same = [np.linalg.norm(pos(se[i][0]) - pos(se[i - 1][0])) * 1e3 < still_mm and se[i - 1][5] > tol_rad
                for i in range(1, len(se))]
    else:
        same = [se[i][0] - se[i - 1][0] < gap_s for i in range(1, len(se))]
    return [i for i in range(1, len(se)) if same[i - 1] and not math.isnan(se[i][5])]


def line_spans(se, t_fk=None, pos_fk=None, gap_s=RECHECK_GAP_S):
    """(post index, next index) per scan line: from a post-correction reading to the next check (the line end),
    skipping the first post (initial alignment -> lead-in) as the original parity code did. Only meaningful for
    rasters checked at line ends (Trial 2 style)."""
    post = post_correction_indices(se, t_fk, pos_fk, gap_s)
    return [(k, k + 1) for k in post[1:] if k + 1 < len(se) and not math.isnan(se[k + 1][5])]
