# Motor bench sketch

This sketch preserves the `M,<signed PWM>` command used during the two-motor
bench test. It is intentionally separate from `safestride_mcu`, whose binary
serial session, watchdog, E-stop, pressure dead-man and encoder checks must not
be bypassed in normal operation.

## Pin map

| Driver input | Uno pin |
|---|---:|
| Left PWM | D5 |
| Left IN1 | D6 |
| Left IN2 | D8 |
| Right PWM | D9 |
| Right IN1 | D10 |
| Right IN2 | D12 |

Connect Arduino GND to both driver control grounds. Do not power either motor
or driver from the Uno 5 V pin. For each SZH-GNP521, connect Uno PWM to the
driver `PWM`, the two direction pins to `IN1` and `IN2`, and Arduino GND to
driver `COM`. Leave the driver's `5VO` output unconnected. Confirm the terminal
labels against the exact driver revision before applying power.

Use a current-limited supply or a correctly sized fuse, keep both wheels off the
ground and begin with `M,20`. `MAX_PWM` is capped at 100 for first testing.
