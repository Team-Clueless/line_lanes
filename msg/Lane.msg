std_msgs/Header header

# Used for publishing updates to a lane.
# Points are inserted after this element, starting from 0
# negative indicates backward indexing, i.e -1 == append
int64 offset

# Only x and y co-ords are currently used
geometry_msgs/Point[] points