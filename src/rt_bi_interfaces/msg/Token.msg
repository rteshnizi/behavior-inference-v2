# Unique string id
string id
# parent id, if any
string parent_id

# The timestamp associated with the update.
# Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs') # CSpell: ignore nsec, nsecs
# time-handling sugar is provided by the client library
builtin_interfaces/Time stamp

# Target attributes (reified shape; written to RDF on each propagation step)
rt_bi_interfaces/Attribute[] attributes
