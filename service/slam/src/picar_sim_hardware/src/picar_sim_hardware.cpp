#include "picar_sim_hardware/picar_sim_hardware.hpp"

#include <cstdlib>

#include <nlohmann/json.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace picar_sim_hardware
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using json = nlohmann::json;

namespace
{
size_t collect(char * data, size_t size, size_t n, void * out)
{
  static_cast<std::string *>(out)->append(data, size * n);
  return size * n;
}
// A controller cycle is 50 ms; an HTTP round trip that takes longer than
// this is a failure, not a slow success -- the command would be stale.
constexpr long kTimeoutMs = 40;
// An unreachable robot server is NOT a hardware error here. Returning ERROR
// makes ros2_control deactivate the component for good -- which R4's first
// live run did, silently, across a routine restart of the robot server: the
// controllers still read "active" and commanded nothing. The robot server's
// watchdog is what keeps an unreachable robot safe (criterion 6), so this
// keeps trying, reports the outage, and reads zero velocity meanwhile.
}  // namespace

CallbackReturn PicarSimHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  auto it = info_.hardware_parameters.find("robot_url");
  robot_url_ = it != info_.hardware_parameters.end() ? it->second : "http://host.docker.internal:8000";
  // Same shared secret robot/server.py's require_secret() checks. Read from
  // the environment, never from the URDF, so it is never in the repo.
  if (const char * s = std::getenv("APP_SHARED_SECRET")) {
    secret_ = s;
  }
  if (const char * u = std::getenv("ROBOT_URL")) {
    robot_url_ = u;
  }
  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(logger_, "expected 2 wheel joints, got %zu", info_.joints.size());
    return CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < 2; ++i) {
    names_[i] = info_.joints[i].name;
  }
  curl_ = curl_easy_init();
  if (!curl_) {
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "robot server at %s", robot_url_.c_str());
  return CallbackReturn::SUCCESS;
}

PicarSimHardware::~PicarSimHardware()
{
  if (curl_) {
    curl_easy_cleanup(curl_);
  }
}

std::vector<hardware_interface::StateInterface> PicarSimHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < 2; ++i) {
    out.emplace_back(names_[i], hardware_interface::HW_IF_POSITION, &pos_[i]);
    out.emplace_back(names_[i], hardware_interface::HW_IF_VELOCITY, &vel_[i]);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> PicarSimHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < 2; ++i) {
    out.emplace_back(names_[i], hardware_interface::HW_IF_VELOCITY, &cmd_[i]);
  }
  return out;
}

CallbackReturn PicarSimHardware::on_activate(const rclcpp_lifecycle::State &)
{
  cmd_[0] = cmd_[1] = 0.0;
  // A body that answers "no wheels" at activation has none to drive (a phone
  // walk, a replay): refuse to come up. An unreachable server is not that --
  // read() keeps trying -- and neither is a body whose wheels are only
  // momentarily unmeasured once running (3.34; see read()), nor a motor board
  // that has not sent its first frame yet: it says `awaiting_feedback`, and
  // the plugin comes up and waits (handoff 2a; it used to refuse for good,
  // so the container had to be started after the board's first frame).
  std::string out;
  if (request("GET", "/wheels", "", out)) {
    try {
      const json w = json::parse(out);
      if (!w.value("usable", false) && !w.value("awaiting_feedback", false)) {
        RCLCPP_ERROR(logger_, "the robot server reports no wheels (usable: false) -- "
                     "not activating");
        return CallbackReturn::ERROR;
      }
    } catch (const std::exception &) {
      // A bad reply is read()'s to report; it keeps trying.
    }
  }
  return read(rclcpp::Time(), rclcpp::Duration(0, 0)) == return_type::OK ?
         CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn PicarSimHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  post_wheels(0.0, 0.0);
  return CallbackReturn::SUCCESS;
}

bool PicarSimHardware::request(const std::string & method, const std::string & path,
                               const std::string & body, std::string & out)
{
  out.clear();
  curl_easy_reset(curl_);
  const std::string url = robot_url_ + path;
  struct curl_slist * headers = nullptr;
  headers = curl_slist_append(headers, "content-type: application/json");
  headers = curl_slist_append(headers, "x-driver: ros");
  if (!secret_.empty()) {
    headers = curl_slist_append(headers, ("x-app-secret: " + secret_).c_str());
  }
  curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl_, CURLOPT_TIMEOUT_MS, kTimeoutMs);
  curl_easy_setopt(curl_, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, collect);
  curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &out);
  if (method == "POST") {
    curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, body.c_str());
  }
  const CURLcode rc = curl_easy_perform(curl_);
  long status = 0;
  curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &status);
  curl_slist_free_all(headers);
  return rc == CURLE_OK && status == 200;
}

void PicarSimHardware::note_failure(const char * what)
{
  vel_[0] = vel_[1] = 0.0;
  if (consecutive_failures_++ % 100 == 0) {
    RCLCPP_WARN(logger_, "%s to %s failed (%d in a row) -- still trying; the robot "
                "server's watchdog stops the wheels meanwhile", what, robot_url_.c_str(),
                consecutive_failures_);
  }
}

bool PicarSimHardware::post_wheels(double left, double right)
{
  std::string out;
  const json body = {{"left_rad_s", left}, {"right_rad_s", right}};
  return request("POST", "/wheels", body.dump(), out);
}

return_type PicarSimHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::string out;
  if (!request("GET", "/wheels", "", out)) {
    note_failure("GET /wheels");
    return return_type::OK;
  }
  try {
    const json w = json::parse(out);
    if (!w.value("usable", false)) {
      // 3.34: mid-run this is a body that has stopped HEARING its wheels (a
      // dropped serial link, a silent motor board) -- it refuses to move
      // them meanwhile. Returning ERROR would make ros2_control deactivate
      // this component for good, so the chain would stay dead after the
      // board recovered. Same answer as an unreachable server: hold the
      // position, report no velocity, keep trying.
      note_failure("GET /wheels (no fresh wheel feedback)");
      return return_type::OK;
    }
    for (size_t i = 0; i < 2; ++i) {
      // Joint i is whichever the URDF declared; match it by side.
      const std::string side = names_[i].rfind("left", 0) == 0 ? "left" : "right";
      pos_[i] = w[side]["position_rad"].get<double>();
      vel_[i] = w[side]["velocity_rad_s"].get<double>();
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "bad /wheels reply: %s", e.what());
    note_failure("GET /wheels (parse)");
    return return_type::OK;
  }
  if (consecutive_failures_ > 0) {
    RCLCPP_INFO(logger_, "robot server reachable again after %d failed cycles", consecutive_failures_);
  }
  consecutive_failures_ = 0;
  return return_type::OK;
}

return_type PicarSimHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Sent every cycle, zeros included: that is the heartbeat the robot
  // server's watchdog reads. If this container dies the posts stop and the
  // watchdog zeroes the wheels on its own clock (criterion 6).
  const bool left_first = names_[0].rfind("left", 0) == 0;
  const double left = left_first ? cmd_[0] : cmd_[1];
  const double right = left_first ? cmd_[1] : cmd_[0];
  if (!post_wheels(left, right)) {
    note_failure("POST /wheels");
  }
  return return_type::OK;
}

}  // namespace picar_sim_hardware

PLUGINLIB_EXPORT_CLASS(picar_sim_hardware::PicarSimHardware, hardware_interface::SystemInterface)
