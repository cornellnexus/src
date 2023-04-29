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
      "reading": {
        "latitude": Float,
        "longitude": Float
      }
    },
    "imu" {
      "connected": Boolean
      "reading": {
        "accelerometer": {
          "x-axis": Float,
          "y-axis": Float,
          "z-axis": Float
        }
        "magnetometer": {
          "x-axis": Float,
          "y-axis": Float,
          "z-axis": Float
        }
        "gyroscope": {
          "x-axis": Float,
          "y-axis": Float,
          "z-axis": Float
        }
      }
    },
    "break_beams" {
      "half1": {
        "connected": Boolean,
        "blocked": Boolean
      },
      "half2": {
        "connected": Boolean,
        "blocked": Boolean
      },
      "full1": {
        "connected": Boolean,
        "blocked": Boolean
      },
      "full2": {
        "connected": Boolean,
        "blocked": Boolean
      }
      "half_full": Boolean,
      "max_full": Boolean,
    },
    "wheel_motors": {
      "duty_cycle": Float,
      "linear_velocity": Float,
      "left_wheel_velocity": Float,
      "right_wheel_velocity": Float
    }
    "cams" {
      "front_cam": {
        "connected": Boolean,
        "tag_id_detected": String or null
      },
      "back_cam": {
        "connected": Boolean,
        "tag_id_detected": String or null
      }
    },
    "ultrasonic": { 
      "front_uls": {
        "connected": Boolean,
        "distance_to_object": Float or null
      },
      "left1_uls": {
        "connected": Boolean,
        "distance_to_object": Float or null
      },
      "left2_uls": {
        "connected": Boolean,
        "distance_to_object": Float or null
      },
      "right_uls": {
        "connected": Boolean,
        "distance_to_object": Float or null
      }
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