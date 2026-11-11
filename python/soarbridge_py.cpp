// pybind11 module `soarbridge` for RobotPy projects.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "soarbridge/SoarHardwareInterface.h"

namespace py = pybind11;
using namespace soarbridge;

namespace {

// Lets Python subclasses of CommandHandler receive dispatched commands.
class PyCommandHandler : public CommandHandler {
 public:
  void Move(double vx, double vy, double omega) override {
    PYBIND11_OVERRIDE_PURE_NAME(void, CommandHandler, "move", Move, vx, vy, omega);
  }
  void TurnTo(double heading) override {
    PYBIND11_OVERRIDE_PURE_NAME(void, CommandHandler, "turn_to", TurnTo, heading);
  }
  void SetElevator(double height) override {
    PYBIND11_OVERRIDE_PURE_NAME(void, CommandHandler, "set_elevator", SetElevator, height);
  }
  void RunIntake(double speed) override {
    PYBIND11_OVERRIDE_PURE_NAME(void, CommandHandler, "run_intake", RunIntake, speed);
  }
  void Shoot(bool execute) override {
    PYBIND11_OVERRIDE_PURE_NAME(void, CommandHandler, "shoot", Shoot, execute);
  }
};

}  // namespace

PYBIND11_MODULE(soarbridge, m) {
  m.doc() = "Bridge between a WPILib/RobotPy robot loop and a Soar agent";

  py::enum_<MatchMode>(m, "MatchMode")
      .value("DISABLED", MatchMode::Disabled)
      .value("AUTONOMOUS", MatchMode::Autonomous)
      .value("TELEOP", MatchMode::Teleop)
      .value("TEST", MatchMode::Test);

  py::enum_<Alliance>(m, "Alliance")
      .value("RED", Alliance::Red)
      .value("BLUE", Alliance::Blue);

  py::enum_<CommandType>(m, "CommandType")
      .value("MOVE", CommandType::Move)
      .value("TURN_TO", CommandType::TurnTo)
      .value("SET_ELEVATOR", CommandType::SetElevator)
      .value("RUN_INTAKE", CommandType::RunIntake)
      .value("SHOOT", CommandType::Shoot);

  py::class_<RobotInputs>(m, "RobotInputs", "Snapshot written to the Soar input-link")
      .def(py::init<>())
      .def_readwrite("match_time", &RobotInputs::matchTime)
      .def_readwrite("mode", &RobotInputs::mode)
      .def_readwrite("alliance", &RobotInputs::alliance)
      .def_readwrite("pose_x", &RobotInputs::poseX)
      .def_readwrite("pose_y", &RobotInputs::poseY)
      .def_readwrite("heading", &RobotInputs::heading)
      .def_readwrite("velocity_x", &RobotInputs::velocityX)
      .def_readwrite("velocity_y", &RobotInputs::velocityY)
      .def_readwrite("target_detected", &RobotInputs::targetDetected)
      .def_readwrite("target_distance", &RobotInputs::targetDistance)
      .def_readwrite("target_yaw", &RobotInputs::targetYaw)
      .def_readwrite("has_piece", &RobotInputs::hasPiece)
      .def_readwrite("elevator_height", &RobotInputs::elevatorHeight);

  py::class_<Command>(m, "Command", "One command read from the Soar output-link")
      .def_readonly("type", &Command::type)
      .def_readonly("id", &Command::id)
      .def_readonly("vx", &Command::vx)
      .def_readonly("vy", &Command::vy)
      .def_readonly("omega", &Command::omega)
      .def_readonly("heading", &Command::heading)
      .def_readonly("height", &Command::height)
      .def_readonly("speed", &Command::speed)
      .def_readonly("execute", &Command::execute);

  py::class_<CommandHandler, PyCommandHandler>(m, "CommandHandler",
      "Subclass and override move, turn_to, set_elevator, run_intake and shoot")
      .def(py::init<>());

  py::class_<BridgeStats>(m, "BridgeStats")
      .def_readonly("inputs_submitted", &BridgeStats::inputsSubmitted)
      .def_readonly("inputs_processed", &BridgeStats::inputsProcessed)
      .def_readonly("inputs_overwritten", &BridgeStats::inputsOverwritten)
      .def_readonly("decision_cycles", &BridgeStats::decisionCycles)
      .def_readonly("commands_received", &BridgeStats::commandsReceived)
      .def_readonly("commands_rejected", &BridgeStats::commandsRejected)
      .def_readonly("commands_dropped", &BridgeStats::commandsDropped)
      .def_readonly("commands_completed", &BridgeStats::commandsCompleted)
      .def_readonly("last_latency_micros", &BridgeStats::lastLatencyMicros)
      .def_readonly("max_latency_micros", &BridgeStats::maxLatencyMicros)
      .def_readonly("mean_latency_micros", &BridgeStats::meanLatencyMicros)
      .def_readonly("latency_over_5ms", &BridgeStats::latencyOver5ms);

  py::class_<SoarHardwareInterface>(m, "SoarHardwareInterface")
      .def(py::init([](const std::string& productionsPath, const std::string& agentName,
                       int decisionsPerInput, int debuggerPort, int maxPendingCommands,
                       int realtimePriority) {
             BridgeConfig config;
             config.productionsPath = productionsPath;
             config.agentName = agentName;
             config.decisionsPerInput = decisionsPerInput;
             config.debuggerPort = debuggerPort;
             config.maxPendingCommands = maxPendingCommands;
             config.realtimePriority = realtimePriority;
             return std::make_unique<SoarHardwareInterface>(config);
           }),
           py::arg("productions_path") = "", py::arg("agent_name") = "robot",
           py::arg("decisions_per_input") = 1, py::arg("debugger_port") = 0,
           py::arg("max_pending_commands") = 256, py::arg("realtime_priority") = 0)
      .def("start",
           [](SoarHardwareInterface& self) {
             bool ok;
             {
               py::gil_scoped_release release;
               ok = self.Start();
             }
             if (!ok) throw std::runtime_error(self.GetLastError());
           },
           "Start the Soar thread and load productions; raises RuntimeError on failure")
      .def("stop",
           [](SoarHardwareInterface& self) {
             py::gil_scoped_release release;
             self.Stop();
           })
      .def("is_running", &SoarHardwareInterface::IsRunning)
      .def("set_inputs", &SoarHardwareInterface::SetInputs)
      .def("poll_commands", &SoarHardwareInterface::PollCommands)
      .def("complete_command", &SoarHardwareInterface::CompleteCommand,
           py::arg("command_id"), py::arg("success") = true)
      .def("dispatch_commands", &SoarHardwareInterface::DispatchCommands)
      .def("get_stats", &SoarHardwareInterface::GetStats)
      .def("__enter__",
           [](py::object self) {
             self.attr("start")();
             return self;
           })
      .def("__exit__", [](SoarHardwareInterface& self, py::args) {
        py::gil_scoped_release release;
        self.Stop();
      });
}
