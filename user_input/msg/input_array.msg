# Message for publishing human input data

int32 droneID       # for waypoint control, either a 0, 1, or 2; for ergodic, always a 5
int32 datalen       # length of array
int32[] xinput      # x-coordinate of the input in unity coordinates
int32[] yinput      # y-coordinate of the input in unity coordinates
