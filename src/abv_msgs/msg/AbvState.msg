# time of last real measurement/update (not publish time)
builtin_interfaces/Time timestamp

# true if this is fresh EKF output from a real measurement; false if
# free-running dead-reckoning has exceeded the configured bound
bool valid

AbvVec3 position
AbvVec3 velocity