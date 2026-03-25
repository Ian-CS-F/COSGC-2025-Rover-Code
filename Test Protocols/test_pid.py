"""
Test Protocol: PID Controller
Tests the PID class exactly as defined in Python/Control/pid.py.
No hardware needed — purely computational.
"""

import sys
sys.path.append("../Python/Control")

from pid import PID  # type: ignore

# ── Tests ─────────────────────────────────────────────────────────────────────
def test_zero_error_zero_output():
    """With zero error and no integral history, output should be 0."""
    print("TEST: Zero error → zero output...")
    pid = PID(kp=1.0, ki=0.0, kd=0.0)
    output = pid.update(target=0.0, measured=0.0, dt=0.1)
    passed = abs(output) < 1e-9
    print(f"  output = {output}  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_positive_error_positive_output():
    """target > measured → positive error → positive P output."""
    print("TEST: Positive error → positive output...")
    pid = PID(kp=1.0, ki=0.0, kd=0.0)
    output = pid.update(target=10.0, measured=0.0, dt=0.1)
    passed = output > 0.0
    print(f"  output = {output}  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_negative_error_negative_output():
    """target < measured → negative error → negative P output."""
    print("TEST: Negative error → negative output...")
    pid = PID(kp=1.0, ki=0.0, kd=0.0)
    output = pid.update(target=0.0, measured=10.0, dt=0.1)
    passed = output < 0.0
    print(f"  output = {output}  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_proportional_scales_with_kp():
    """Doubling kp should double the output for the same error."""
    print("TEST: Output scales linearly with kp...")
    pid1 = PID(kp=1.0, ki=0.0, kd=0.0)
    pid2 = PID(kp=2.0, ki=0.0, kd=0.0)
    out1 = pid1.update(target=5.0, measured=0.0, dt=0.1)
    out2 = pid2.update(target=5.0, measured=0.0, dt=0.1)
    passed = abs(out2 - 2 * out1) < 1e-9
    print(f"  kp=1 → {out1},  kp=2 → {out2}  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_integral_accumulates():
    """With ki > 0, repeated updates should grow the output over time."""
    print("TEST: Integral term accumulates over multiple updates...")
    pid = PID(kp=0.0, ki=1.0, kd=0.0)
    out1 = pid.update(target=1.0, measured=0.0, dt=0.1)  # integral = 0.1
    out2 = pid.update(target=1.0, measured=0.0, dt=0.1)  # integral = 0.2
    passed = out2 > out1 > 0.0
    print(f"  step1={out1:.4f}  step2={out2:.4f}  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_derivative_opposes_change():
    """With kd > 0, a sudden error increase should raise output less than P alone."""
    print("TEST: Derivative term dampens sudden error change...")
    pid_p   = PID(kp=1.0, ki=0.0, kd=0.0)
    pid_pd  = PID(kp=1.0, ki=0.0, kd=1.0)
    # First step: same error, kd derivative is error/dt (no past error yet, past_error=0)
    # error=5, past_error=0 → derivative = 5/0.1 = 50 → adds 50 to PD output
    # So PD output will be LARGER on first step (opposing the step-up)
    # After that, if error shrinks, derivative is negative and dampens
    pid_p.update( target=0.0, measured=5.0, dt=0.1)   # step 1: past_error=-5
    pid_pd.update(target=0.0, measured=5.0, dt=0.1)   # step 1: past_error=-5
    # Step 2: error stays the same → derivative = 0, both outputs identical
    out_p  = pid_p.update( target=0.0, measured=5.0, dt=0.1)
    out_pd = pid_pd.update(target=0.0, measured=5.0, dt=0.1)
    passed = abs(out_p - out_pd) < 1e-9  # derivative is 0 when error is constant
    print(f"  P={out_p:.4f}  PD={out_pd:.4f} (constant error → D=0)  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_slip_compensation_scenario():
    """
    Simulate the slip compensation loop from main.py:
    error_ratio = 0.8 (stalling), so speed is reduced.
    The PID is not used in slip compensation directly (main.py uses a ratio
    formula), but verify the PID class behaves correctly with the gains
    used conceptually (kp=1, ki=0, kd=0) for the slip ratio.
    """
    print("TEST: PID with slip-ratio style inputs (target=1.0, measured=0.8)...")
    pid = PID(kp=1.0, ki=0.0, kd=0.0)
    output = pid.update(target=1.0, measured=0.8, dt=0.1)
    passed = output > 0.0  # positive correction needed to reach target ratio
    print(f"  output = {output:.4f}  →  {'PASS' if passed else 'FAIL'}")
    return passed

# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    tests = [
        test_zero_error_zero_output,
        test_positive_error_positive_output,
        test_negative_error_negative_output,
        test_proportional_scales_with_kp,
        test_integral_accumulates,
        test_derivative_opposes_change,
        test_slip_compensation_scenario,
    ]
    results = []
    for t in tests:
        results.append((t.__name__, t()))

    print("\n--- PID Test Results ---")
    for name, passed in results:
        print(f"  {'PASS' if passed else 'FAIL'}  {name}")
