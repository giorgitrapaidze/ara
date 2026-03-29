# Index on the reference line. -1 means uninitialized.
# Closest reference line index
uint32 idx

# Longitudinal offset (progress) from x_ref[idx], y_ref[idx], e.g 0.5m
# always less than 1.1m.
# Distance along the curve from the reference point to the projection
float64 ds

# ds / ds_curv_step. Progress in terms of index general coordinates.
float64 d_idx

# Lateral offset from the reference curve
float64 n

# Heading error w.r.t the reference curve
float64 epsi

uint8 status