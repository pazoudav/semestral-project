Header header                    # frame = world_frame_id

# RANSAC estimate (source skeleton -> live skeleton)
float64 scale                    # uniform scale about the origin
float64 yaw                      # [rad] rotation about Z
geometry_msgs/Point translation  # z = 0
float64 height                   # [m] estimated object height = scale * h_base (from paths/index.yaml)

# stability, decided only from the history of the estimate itself
bool stable
int32 history_size               # estimates in the stability window so far
float64 max_delta_height         # [m] largest |height_i - height| over the window
float64 max_delta_yaw            # [rad] largest wrapped |yaw_i - yaw| over the window
float64 max_delta_translation    # [m] largest |t_i - t| over the window

# informational only, not part of the stability decision
float64 mean_residual            # [m] RMS nearest-neighbor distance live -> transformed source skeleton
float64 observed_ratio           # live skeleton top z / estimated skeleton top z

# selected precomputed path, filled whenever a path set is loaded (check `stable` before using)
int32 path_id                    # index into paths/index.yaml paths; -1 if none
float64 path_height              # [m] height bin of the selected path
float64 path_scale               # scale that path was planned at (path_height / h_base)
float64 applied_scale            # scale actually applied to the path in transform() (1.0 unless apply_residual_scale)
Waypoint[] waypoints             # full tour in order, index 0 included
