#
# A list of detected features
#
std_msgs/Header header

# All arrays must be the same length or empty.
# An empty array means no data is provided. E.g. no orientation is known/relevant, no category is known/relevant, etc.

geometry_msgs/Point[]      positions
geometry_msgs/Quaternion[] orientations
uint32[]                   categories

# TODO(glpuga): Add covariance matrices for all of the above.
