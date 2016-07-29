int16 timestamps          # motor state is at this time
int8 id                     # motor id
int8 operating_mode
int8 torque
int32 goal_position          # commanded position (in encoder units)
int32 present_position       # current position (in encoder units)
int32 goal_velocity
int32 present_velocity       # current speed (0.111 rpm per unit)
int16 voltage                # current voltage (V)
int8 temperature            # current temperature (degrees Celsius)
int8 is_moving               # whether the motor is currently in motion
