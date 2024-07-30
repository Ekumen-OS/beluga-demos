#
# The expected location of the features in the world. All poses are relative to the frame_id in the header.
#
std_msgs/Header header

# All arrays must be the same length or empty.
# An empty array means no data is provided. E.g. no orientation is known/relevant, no category is known/relevant, etc.

geometry_msgs/Point[]      positions
geometry_msgs/Quaternion[] orientations
uint32[]                   categories
