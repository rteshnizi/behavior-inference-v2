# The outer attribute name, e.g. "height", "transportation_mode"
string name
# "ranged" or "discrete"
string KIND_RANGED = "ranged"
string KIND_DISCRETE = "discrete"
string kind

# Populated when kind == "ranged".
# NaN encodes "absent / maximal set on that side" (no constraint on that endpoint).
float64 range_min
float64 range_max

# Populated when kind == "discrete".
string[] discrete_values
