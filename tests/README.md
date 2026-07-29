# Hardware Tests

This directory contains validation tests for the Ball Tracking Robot system.

The purpose of these tests is to verify individual hardware and firmware subsystems before full system integration.

---

# Test Structure

```
tests/
│
├── uart_test/
│   └── uart_test.py
│
├── motor_test/
│   └── motor_test.c
│
├── encoder_test/
│   └── encoder_test.c
│
└── pid_test/
    └── pid_test.py
```

---

# UART Communication Test

## Location

```
uart_test/uart_test.py
```

## Purpose

Verifies communication between:

- Raspberry Pi
- STM32 motor controller

Tests:

- UART wiring
- Baud rate configuration
- Command transmission
- STM32 command reception

## Hardware Required

- Raspberry Pi
- STM32 controller PCB
- UART connection

## Run

```bash
python3 uart_test.py
```

## Expected Behavior

The terminal will allow manual movement commands:

| Command | Function |
|---|---|
| F | Forward |
| B | Backward |
| L | Turn Left |
| R | Turn Right |
| S | Stop |

The robot should respond to each command.

---

# Motor Driver Test

## Location

```
motor_test/motor_test.c
```

## Purpose

Verifies the TB6612FNG motor driver interface.

Tests:

- Motor enable control
- PWM output
- Direction pins
- Left motor operation
- Right motor operation

## Hardware Required

- STM32 controller PCB
- TB6612FNG motor driver
- Motors connected

## Expected Behavior

The motors should:

1. Move forward
2. Stop
3. Move backward
4. Stop

This confirms:

- PWM signals are generated correctly
- Motor direction pins are correct
- Motor driver wiring is functional

---

# Encoder Test

## Location

```
encoder_test/encoder_test.c
```

## Purpose

Verifies wheel encoder operation.

Tests:

- Encoder power
- A/B channel wiring
- Timer encoder mode configuration
- Counting direction
- Wheel feedback

## Hardware Required

- STM32 controller PCB
- Quadrature wheel encoders

## Expected Behavior

UART output should display changing encoder counts:

```
Left: 120   Right: 118

Left: 240   Right: 236
```

When wheels rotate forward:

```
Encoder counts increase
```

When wheels rotate backward:

```
Encoder counts decrease
```

---

# PID Control Test

## Location

```
pid_test/pid_test.py
```

## Purpose

Tests motor PID control behavior before connecting the physical robot.

The simulation verifies:

- Error calculation
- Proportional response
- Integral response
- Derivative response
- System settling behavior

## Expected Behavior

The simulated motor speed should:

- Increase toward the target speed
- Reduce error over time
- Stabilize near the desired value

Example:

```
Target Speed: 100 RPM

Time     Speed
0s       0 RPM
1s       65 RPM
2s       92 RPM
3s       99 RPM
4s       100 RPM
```

---

# Test Procedure

Subsystem testing should be completed in this order:

## 1. Power Validation

Verify:

- 5V rail
- 3.3V rail
- No excessive current draw

---

## 2. UART Test

Verify:

- Raspberry Pi communication
- STM32 command reception

---

## 3. Motor Test

Verify:

- Motor direction
- PWM control
- TB6612FNG operation

---

## 4. Encoder Test

Verify:

- Encoder counts
- Direction detection
- Timer configuration

---

## 5. PID Test

Verify:

- Closed-loop speed control
- Stable motor response

---

# Future Tests

Additional validation tests planned:

- Battery voltage monitoring
- Current consumption measurement
- Ultrasonic sensor validation
- Camera tracking latency testing
- Full autonomous navigation testing
- Long-duration reliability testing