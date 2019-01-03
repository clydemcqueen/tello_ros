std_msgs/Header header

#######
# Raw string
#######

string raw

#######
# SDK version
#######

uint8 SDK_UNKNOWN=0
uint8 SDK_1_3=1
uint8 SDK_2_0=2
uint8 sdk

#######
# SDK 1.3+ fields
#######

int32 pitch   # pitch in degrees
int32 roll    # roll in degrees
int32 yaw     # yaw in degrees relative to startup orientation

int32 vgx     # x velocity in m/s, +forward
int32 vgy     # y velocity in m/s, +right
int32 vgz     # z velocity in m/s, +down

int32 templ   # lowest temp in C
int32 temph   # highest temp in C

int32 tof     # time of flight distance in cm
int32 h       # height in cm

int32 bat     # battery %

float32 baro  # barometric altitude in m
int32 time    # time used by motor in seconds

# Acceleration includes gravity
float32 agx   # x acceleration in mm/s^2, +forward
float32 agy   # y acceleration in mm/s^2, +right
float32 agz   # z acceleration in mm/s^2, +down

#######
# SDK 2.0+ fields
#######

int32 mid     # mission pad id, or -1 if not found
int32 x       # x coord relative to mission pad in cm, or 0 if not found
int32 y       # y coord relative to mission pad in cm, or 0 if not found
int32 z       # z coord relative to mission pad in cm, or 0 if not found