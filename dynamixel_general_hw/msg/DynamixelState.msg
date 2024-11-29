# This message includes basic data of dynamixel

string name                   # Name of dynamixel
uint8  id                     # ID of dynamixel

int32  present_position       # Present position (raw value in dynamixel RAM)
int32  present_velocity       # Present velocity (raw value in dynamixel RAM)
int16  present_current        # Present current (raw value in dynamixel RAM)
uint8  present_temperature    # Present temperature (raw value in dynamixel RAM)
uint16 present_input_voltage  # Present input voltage (raw value in dynamixel RAM)
