# Per-camera collection progress for one CalibrateCamera goal.
#
# A mono goal sends the goal camera's counters in the feedback's scalar fields
# and one of these mirroring them. A both-mode goal sends the scalar fields as
# the first camera's counters and the per-camera truth here.

# Camera key in the rig config (e.g. "left").
string camera

uint32 frames_seen        # frames received and decoded for this camera
uint32 frames_detected    # board found with exactly n_x*n_y dots
uint32 frames_accepted    # passed the pose-diversity gate
uint32 frames_duplicate   # rejected by the pose-diversity gate
uint32 frames_unpaired    # dropped: no simultaneous frame from the other camera
uint32 budget             # accepted-frame budget for this camera
float32 progress          # accepted / budget, capped at 1.0
