"""Exercises the Python module end to end. Usage: test_python.py <agents dir>"""
import sys
import time

import soarbridge
from soarbridge import Alliance, CommandType, MatchMode, RobotInputs, SoarHardwareInterface

failures = 0


def check(condition, what):
    global failures
    if not condition:
        failures += 1
        print("  FAIL:", what)


class Recorder(soarbridge.CommandHandler):
    def __init__(self):
        super().__init__()
        self.moves = []
        self.intakes = 0
        self.shots = 0

    def move(self, vx, vy, omega):
        self.moves.append((vx, vy, omega))

    def turn_to(self, heading):
        pass

    def set_elevator(self, height):
        pass

    def run_intake(self, speed):
        self.intakes += 1

    def shoot(self, execute):
        self.shots += 1


def main(agents):
    try:
        SoarHardwareInterface(productions_path="/nonexistent/agent.soar").start()
        check(False, "start() raises for a missing productions file")
    except RuntimeError:
        pass

    inputs = RobotInputs()
    inputs.mode = MatchMode.TELEOP
    inputs.alliance = Alliance.RED
    inputs.target_detected = True
    inputs.target_yaw = 10.0

    with SoarHardwareInterface(productions_path=agents + "/default.soar") as soar:
        check(soar.is_running(), "running inside the with block")

        # Manual polling.
        commands = []
        for _ in range(50):
            soar.set_inputs(inputs)
            time.sleep(0.002)
            commands = soar.poll_commands()
            if commands:
                break
        types = sorted(c.type.name for c in commands)
        check(types == ["MOVE", "RUN_INTAKE"], "move + run-intake issued, got %s" % types)
        for c in commands:
            if c.type == CommandType.MOVE:
                check(abs(c.vx - 1.0) < 1e-9 and abs(c.omega + 0.3) < 1e-9, "move parameters")
            soar.complete_command(c.id)

        # Handler dispatch.
        robot = Recorder()
        inputs.has_piece = True
        for _ in range(50):
            soar.set_inputs(inputs)
            time.sleep(0.002)
            soar.dispatch_commands(robot)
            if robot.shots:
                break
        check(robot.shots >= 1, "shoot dispatched to handler")

        for i in range(2000):
            inputs.match_time = 135.0 - i * 0.02
            inputs.has_piece = (i // 50) % 2 == 0
            soar.set_inputs(inputs)
            soar.dispatch_commands(robot)
            if i % 10 == 0:
                time.sleep(0.001)

        stats = soar.get_stats()
        check(stats.commands_rejected == 0, "no rejected commands")
        check(stats.inputs_processed > 100, "inputs processed")
        check(stats.max_latency_micros < 5000, "max latency < 5 ms, got %.0f us" % stats.max_latency_micros)
        print("  latency: mean %.1f us, max %.1f us" % (stats.mean_latency_micros, stats.max_latency_micros))

    check(not soar.is_running(), "stopped after the with block")
    print("PASSED" if not failures else "FAILED (%d)" % failures)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
