# Test Protocols

This folder contains standalone test scripts for verifying each rover subsystem independently. Run these before integrating systems together to confirm each component behaves correctly in isolation.

## Structure

| File | Tests |
|------|-------|
| `test_motors.py` | Motor interface — forward, reverse, turn, stop |
| `test_sensors.py` | Lidar sensor reads |
| `test_nav.py` | A* pathfinding, slope/heightmap calculations |
| `test_pid.py` | PID controller output for known inputs |
| `test_comms.py` | Serial communication between Pi and Arduino |

## How to Use

Run any test directly:
```bash
python3 "Test Protocols/test_motors.py"
```

Each test prints PASS/FAIL for each check. Fix any failures before running the full system.
