define rplidar ranger
(
sensor
(
# laser-specific properties
# rplidar_a2m7 settings for Husarion
range [ 0.0  8.0 ]
fov 360.0
samples 4000  # upto 4100 per second from RPLIDAR A2 data sheet
)

# generic model properties
color "black"
size [ 0.0725 0.041 0.0725 ] # dimensions from RPLIDAR A2 data sheet (WxDxH)
)
