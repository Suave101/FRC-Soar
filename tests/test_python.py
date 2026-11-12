"""Exercises the Python module end to end. Usage: test_python.py <agents dir> <tests dir>"""
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


def main(agents, tests=None):
    tests = tests or agents + "/../tests"
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
    generic(tests)
    print("PASSED" if not failures else "FAILED (%d)" % failures)
    return 1 if failures else 0


def generic(tests):
    """Path inputs and named commands against tests/generic_io.soar."""
    with SoarHardwareInterface(productions_path=tests + "/generic_io.soar") as soar:
        check(not soar.set_number("a..b", 1.0), "invalid path refused")
        check(soar.get_last_warning() != "", "refused path is reported")

        reports = []
        soar.on_command("report", reports.append)
        for name in ("beam", "angle", "has-piece", "no-vision"):
            check(soar.accept_command(name), "accept " + name)

        check(soar.set_string("match.mode", "teleop"), "set_string")
        check(soar.set_int("ground-intake.beam-break", 1), "set_int")
        check(soar.set_bool("ground-intake.has-piece", True), "set_bool")
        check(soar.set_number("arm.angle", 30.5), "set_number")
        check(not soar.set_number("arm.angle.x", 1.0), "child of a value refused")
        check(soar.set_int("test.seq", 1), "set seq")
        for _ in range(50):
            soar.send_inputs()
            time.sleep(0.002)
            soar.dispatch_commands()  # beam, angle, ... have no handler -> unhandled
            if reports:
                break
        check(len(reports) == 1, "one report, got %d" % len(reports))
        if reports:
            r = reports[0]
            check(r.name == "report" and r.type == CommandType.CUSTOM, "custom command named report")
            check(r.params == {"seq": 1, "mode": "teleop", "target.x": 1.5, "target.y": -2},
                  "report params %r" % r.params)
            check(isinstance(r.params["seq"], int), "Soar int -> Python int")
            check(r.get_number("target.x") == 1.5 and r.get_string("mode") == "teleop", "accessors")

        # Polling by hand: the next burst, with ^arm removed.
        soar.remove_input("arm")
        soar.set_int("test.seq", 2)
        got = []
        for _ in range(50):
            soar.send_inputs()
            time.sleep(0.002)
            for c in soar.poll_commands():
                got.append(c)
                soar.complete_command(c.id)
            if any(c.name == "report" for c in got):
                break
        names = sorted(c.name for c in got)
        check("angle" not in names and "beam" in names, "arm removed, beam kept: %s" % names)
        beam = [c for c in got if c.name == "beam"]
        if beam:
            check(beam[0].params["value"] == 1, "beam value")

        stats = soar.get_stats()
        check(stats.commands_rejected == 2, "mystery + dup rejected, got %d" % stats.commands_rejected)
        check(stats.output_constants_ignored == 1,
              "^legacy constant counted once, got %d" % stats.output_constants_ignored)
        check(stats.commands_unhandled >= 1, "accepted-but-unhandled counted")
        check(stats.input_errors == 2, "two refused paths, got %d" % stats.input_errors)


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:3]))
