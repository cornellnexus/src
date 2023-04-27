# Packet Definition

## RPI -> GUI

### Start-Up RPI Response Packet
```
{
  < Response to init form and successful robot initialization e.g. grid spacing >
}
```
### Main Packet
#### Condensed View
```
{
  "timestamp": String ISO 8601 format,
  "sensors": { Dict of sensor-related details },
  "battery": { Dict of battery-related details },
  "metrics": { Dict of robot and mission-related details }
}
```

#### Detailed View
```
{
  "timestamp": String ISO 8601 format,
  "sensors": {
    "gps": {
      "connected": Boolean
      "reading": [ Float of latitude, Float of longitude ]
    },
    "imu" {
      "connected": Boolean
      "reading": {
        "accelerometer": [ Float of x-axis value, Float of y-axis value, Float of z-axis value ],
        "magnetometer": [ Float of x-axis value, Float of y-axis value, Float of z-axis value ],
        "gyroscope": [ Float of x-axis value, Float of y-axis value, Float of z-axis value ]
        }
    },
    "break_beams" {
      half1_connected: Boolean,
      half2_connected: Boolean,
      full1_connected: Boolean,
      full2_connected: Boolean,
      half_full: Boolean,
      max_full: Boolean,
    },
    "wheel_motors": {
      // TODO: connected x 4??? doc list still relevant?
    }
    "cams" {
      "front_cam": {
        "connected": Boolean,
        "tag_id_detected": Integer/String or null  // TODO: integer or string
      },
      "back_cam": {
        "connected": Boolean,
        "tag_id_detected": Integer/String or null  // TODO
      }
    },
    "ultrasonic": { 
      // TODO: doc list still relevant? how many connected
    } 
  },
  "battery": {
    "battery_percent": Integer <0-100>,
    "low_power_mode": Boolean,
    "time_till_recharge": String HH:MM:SS
  },
  "metrics": {
    "goal_loc": {
      "global_coord": [ Float of latitude, Float of longitude ],
      "local_coord": [ Float of x position, Float of y position in grid ],
      "robot_goal_dist: Float <Distance between robot and goal node>
    },
    "state": {
      "global_coord": [ Float of latitude, Float of longitude ],
      "local_coord": [ Float of x position, Float of y position in grid ],
      "heading": Float <Direction robot is facing in degrees/radians from due north?> // TODO
    },
    "phase": String,
    "eta_to_base": String HH:MM:SS,
    "goal_nodes_completed": Integer,
    "eta_complete": String HH:MM:SS,
    ...
  }
}
```


## GUI -> RPI
### Start-Up Packet
```
{
  < Init form data >
}
```

// TODO: idk add more? do we even want more? manual mode who