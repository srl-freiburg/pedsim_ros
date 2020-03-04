# Message defining a tracked group
#

uint64      group_id        # unique identifier of the target, consistent over time

builtin_interfaces/Duration    age             # age of the group

geometry_msgs/PoseWithCovariance    center_of_gravity   # mean and covariance of the group (calculated from its person tracks)

uint64[]    track_ids       # IDs of the tracked persons in this group. See srl_tracking_msgs/TrackedPersons