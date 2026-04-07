# Unique string id
string id

# The timestamp associated with the update.
# Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs') # CSpell: ignore nsec, nsecs
# time-handling sugar is provided by the client library
builtin_interfaces/Time stamp

# Symbolized region predicates
rt_bi_interfaces/Predicate[] predicates
# All the detected tracklets
# Tracklet can be thought of as a special predicate (only sensors have data about it)
rt_bi_interfaces/Tracklet[] estimations

# The spatial geometry associated with this update
rt_bi_interfaces/Polygon[] polygons
# The temporal ranges associated with this update
rt_bi_interfaces/TimeInterval[] intervals

######################### Traversability #########################
# Traversability constraints on this region (from RDF store).
# Empty / NaN means unconstrained.
string[] traversability_transport_req
string traversability_max_diameter    # empty string = unconstrained
string traversability_max_height      # empty string = unconstrained

######################### Properties #########################
# The fully qualified name of the associated ROS node.
string channel

string STATIC = "S"
string DYNAMIC = "D"
string AFFINE = "A"
string SENSING = "Z"
string TARGET = "O"
string TEMPORAL = "T"
string set_type # One of the above constants
