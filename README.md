# Line Follower Robot with PD Control

Line follower robot using 5 analog IR sensors and a PD (Proportional + Derivative) controller for real-time trajectory correction. The system includes automatic calibration, sensor normalization, line position calculation, and smart logic for sharp curves and intelligent stop.

---

## How It Works

**1. Calibration**
Runs for ~5 seconds on startup. Captures the minimum and maximum values of each sensor to normalize readings across different environments.

**2. Sensor Normalization**
Raw sensor values are mapped to a 0–100 scale, ensuring consistent behavior regardless of ambient lighting or surface variation.

**3. Line Position Calculation**
Weighted average using weights `[-2, -1, 0, 1, 2]`. If the line is lost, the last valid position is used instead of resetting to zero.

**4. PD Control**
```
error = setpoint (0) - position
output = kp * error + kd * derivative
```
Derivative is calculated using `dt = (now - lasttime) / 1000.0`.

**5. Motor Control**
Base speed is adjusted by the PD output. A minimum power threshold prevents dead zones. Supports forward and reverse direction.

---

## Special Logic

| Condition | Behavior |
|---|---|
| Strong left sensors activated | Sharp left curve |
| Strong right sensors activated | Sharp right curve |
| All sensors detect line | Wait briefly; if center sensor is lost → full stop |

---

## Hardware

- Arduino (Uno or Nano)
- 5 analog IR sensors
- Motor driver (e.g. L298N)
- 2 DC motors

---

## Parameters

| Parameter | Value |
|---|---|
| `kp` | 130 |
| `kd` | 2 |
| `base_speed` | 170 |
| Sensor weights | [-2, -1, 0, 1, 2] |
| Motor output range | -255 to 255 |
| Minimum power threshold | ±120 |

---

## Program Flow

```
setup()
├── initialization
├── pin configuration
└── calibration

loop()
└── control()
    ├── digital sensor reading
    ├── special curve/stop logic
    └── PD calculation + motor output
```

---

## License

MIT License
