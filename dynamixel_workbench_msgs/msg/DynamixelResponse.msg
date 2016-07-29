int32 realtime_tick          # motor state is at this time
int32 id                     # motor id
int32 goal_position          # commanded position (in encoder units)
int32 present_position       # current position (in encoder units)
int32 goal_velocity
int32 present_velocity       # current speed (0.111 rpm per unit)
int32 voltage                # current voltage (V)
int32 temperature            # current temperature (degrees Celsius)
bool is_moving               # whether the motor is currently in motion
