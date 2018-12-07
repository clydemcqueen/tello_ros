# Flight data, based on Tello SDK 1.3 TODO verify these against SDK

# Battery state:
int32 battery_percent                     # Remaining battery, 0-100
float32 estimated_flight_time_remaining   # Remaining flight time, seconds
bool battery_low                          # Battery is low
bool battery_lower                        # Battery is really low (the red light is flashing)

# Flight modes:
uint8 FLIGHT_MODE_GROUND=1          # Motors off
uint8 FLIGHT_MODE_HOVER=6           # Hovering
uint8 FLIGHT_MODE_TAKING_OFF=11     # Taking off
uint8 FLIGHT_MODE_LANDING=12        # Landing
uint8 FLIGHT_MODE_SPINNING_UP=41    # Spinning up the props, will take off soon
uint8 flight_mode

# Flight time:
float32 flight_time                 # Flight time since power up, in seconds

# Position and velocity, negative numbers mean "no data":
float32 east_speed                  # meters/second
float32 north_speed                 # meters/second
float32 ground_speed                # meters/second
float32 altitude                    # Height off the ground in meters

# Equipment status:
int32 EQUIPMENT_OK=0                # Everything is OK
int32 EQUIPMENT_UNSTABLE=21         # The drone is unstable, tilted at an odd angle or upside down
int32 EQUIPMENT_TIMER_EXCEEDED=205  # No input for 15 seconds, shutting down
int32 equipment

# Temperature:
bool high_temperature               # It's getting warm in here

# ???
bool em_ground                      # ???
bool em_open                        # ???
bool em_sky                         # ???
