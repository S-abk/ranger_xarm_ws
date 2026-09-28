// Four-wheel independent steering (4WIS) motion model for MPPI.
//
// Added to a vendored nav2_mppi_controller 1.3.12 for the Ranger Mini V3,
// whose base can drive in any direction but must first swing its steering
// knuckles: forward -> crab is a 90 deg swing, and the 4WIS controller
// (ranger_4wis_controller.py) holds every wheel back, by a common gate,
// until the slowest knuckle has nearly arrived. Omni assumes a new
// direction of travel takes effect at once; this model makes MPPI's
// rollouts obey the knuckles instead, so it learns to change direction
// early and smoothly rather than commanding jumps the base cannot follow.
//
// Per rollout step it reproduces the controller:
//  0. the base acts on each command command_delay after it is sent
//     (controller tick, simulator or driver, joint drives);
//  1. commands are clamped to the acceleration limits, as in the base
//     model, but relative to the previous COMMAND, so the knuckle lag
//     does not also throttle what is commanded;
//  2. each wheel's target direction is the body twist evaluated at the
//     wheel, folded into +/-90 deg (wheel spun in reverse past that),
//     exactly as the controller folds it: below steer_deadband the knuckle
//     holds and the wheel drives the command's component along it, and
//     within fold_hysteresis_deg of the fold the knuckle keeps its side;
//  3. each knuckle slews towards its target at no more than steer_rate;
//  4. all wheels are scaled by min over wheels of max(0, cos(error))^gate_power;
//  5. the wheel set is rate-limited as a whole to wheel_accel, by one
//     common factor, as the controller does;
//  6. the realised body twist is the least-squares fit to the four wheel
//     velocity vectors.
// The rollout's vx, vy, wz are that realised twist; cvx, cvy, cwz stay the
// commands. Critics therefore score where the base would actually go.
//
// Cost: knuckles are carried as unit vectors, so a step needs no atan2,
// sin or cos. The slew is a snap (dot >= cos(step)) or a rotation by the
// precomputed step whose sign is the cross product, and the gate is the
// dot product raised to an integer power by squaring. About 60 flops per
// wheel per step: 2000 x 56 x 4 wheels is a few milliseconds.
//
// Initial knuckle angles come from joint feedback (sensor_msgs/JointState,
// e.g. joint_state_broadcaster) when it is fresh; otherwise from the
// current velocity, or the last known angles when stopped.

#ifndef NAV2_MPPI_CONTROLLER__FOUR_WIS_MOTION_MODEL_HPP_
#define NAV2_MPPI_CONTROLLER__FOUR_WIS_MOTION_MODEL_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav2_mppi_controller/motion_models.hpp"

namespace mppi
{

class FourWISMotionModel : public MotionModel
{
public:
  FourWISMotionModel(
    ParametersHandler * param_handler, const std::string & name,
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
  {
    auto getParam = param_handler->getParamGetter(name + ".FourWISConstraints");
    getParam(half_wheelbase_, "half_wheelbase", 0.2470);
    getParam(half_track_, "half_track", 0.1852);
    // Measured in Isaac: a 90 deg swing in ~0.33 s, ~270 deg/s on average.
    getParam(steer_rate_, "steer_rate", 4.7);
    getParam(gate_power_, "gate_power", 10.0);
    getParam(wheel_accel_, "wheel_accel", 1.0);
    getParam(feedback_timeout_, "feedback_timeout", 0.5);
    getParam(command_delay_, "command_delay", 0.0);
    getParam(steer_deadband_, "steer_deadband", 0.03);
    double hyst_deg;
    getParam(hyst_deg, "fold_hysteresis_deg", 10.0);
    sin_hyst_ = static_cast<float>(std::sin(hyst_deg * M_PI / 180.0));
    getParam(joint_state_topic_, "joint_state_topic", std::string("/joint_states"));
    getParam(steer_joints_, "steer_joints", std::vector<std::string>{
      "front_left_steer_joint", "front_right_steer_joint",
      "rear_left_steer_joint", "rear_right_steer_joint"});

    const float L = half_wheelbase_, T = half_track_;
    wx_ = {L, L, -L, -L};
    wy_ = {T, -T, T, -T};
    inv_sum_r2_ = 1.0f / (4.0f * (L * L + T * T));
    int_power_ = std::abs(gate_power_ - std::round(gate_power_)) < 1e-6f ?
      static_cast<int>(std::round(gate_power_)) : -1;
    known_c_.fill(1.0f);
    known_s_.fill(0.0f);

    if (auto node = parent.lock()) {
      if (steer_joints_.size() == 4 && !joint_state_topic_.empty()) {
        sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
          joint_state_topic_, rclcpp::SensorDataQoS(),
          [this](const sensor_msgs::msg::JointState::SharedPtr msg) {onJoints(*msg);});
      }
      RCLCPP_INFO(
        node->get_logger(),
        "FourWIS motion model: steer %.2f rad/s, gate power %.1f, wheel accel %.2f m/s^2, "
        "knuckle feedback from %s", steer_rate_, gate_power_, wheel_accel_,
        sub_ ? joint_state_topic_.c_str() : "(none)");
    }
  }

  bool isHolonomic() override {return true;}

  void predict(models::State & state) override
  {
    const unsigned int batch = state.vx.shape(0);
    const unsigned int steps = state.vx.shape(1);
    if (batch == 0 || steps < 2) {
      return;
    }
    const float dt = model_dt_;
    const auto & cc = control_constraints_;
    const float max_dvx = dt * cc.ax_max, min_dvx = dt * cc.ax_min;
    const float max_dvy = dt * cc.ay_max, min_dvy = dt * cc.ay_min;
    const float max_dwz = dt * cc.az_max;
    const float step = std::min(steer_rate_ * dt, static_cast<float>(M_PI));
    const float cos_step = std::cos(step), sin_step = std::sin(step);
    const float wheel_step = wheel_accel_ * dt;

    std::array<float, 4> c0, s0;
    initialKnuckles(state, c0, s0);
    // Steps between a command and the base acting on it. Before the first
    // command takes effect, the base keeps doing what it is doing now.
    const int delay = std::max(0, static_cast<int>(std::lround(command_delay_ / dt)));

    for (unsigned int i = 0; i != batch; i++) {
      float c[4], s[4], w[4];
      const float vx0 = state.vx(i, 0), vy0 = state.vy(i, 0), wz0 = state.wz(i, 0);
      for (int k = 0; k < 4; k++) {
        c[k] = c0[k];
        s[k] = s0[k];
        // Wheel speed now: the current twist at the wheel along its knuckle.
        w[k] = (vx0 - wz0 * wy_[k]) * c[k] + (vy0 + wz0 * wx_[k]) * s[k];
      }
      float cvx_last = vx0, cvy_last = vy0, cwz_last = wz0;

      for (unsigned int j = 1; j != steps; j++) {
        float & cvx = state.cvx(i, j - 1);
        float & cvy = state.cvy(i, j - 1);
        float & cwz = state.cwz(i, j - 1);
        cvx = cvx_last > 0 ? std::clamp(cvx, cvx_last + min_dvx, cvx_last + max_dvx) :
          std::clamp(cvx, cvx_last - max_dvx, cvx_last - min_dvx);
        cvy = cvy_last > 0 ? std::clamp(cvy, cvy_last + min_dvy, cvy_last + max_dvy) :
          std::clamp(cvy, cvy_last - max_dvy, cvy_last - min_dvy);
        cwz = std::clamp(cwz, cwz_last - max_dwz, cwz_last + max_dwz);
        cvx_last = cvx;
        cvy_last = cvy;
        cwz_last = cwz;

        // The command the base is acting on at this step.
        const int src = static_cast<int>(j) - 1 - delay;
        const float ax = src >= 0 ? state.cvx(i, src) : vx0;
        const float ay = src >= 0 ? state.cvy(i, src) : vy0;
        const float aw = src >= 0 ? state.cwz(i, src) : wz0;

        float gate = 1.0f;
        float speed[4];
        for (int k = 0; k < 4; k++) {
          const float u = ax - aw * wy_[k];
          const float v = ay + aw * wx_[k];
          const float sp = std::sqrt(u * u + v * v);
          float ud, vd, spd;
          if (sp < steer_deadband_) {     // too slow to steer by: hold, drive along
            ud = c[k];
            vd = s[k];
            spd = u * c[k] + v * s[k];
          } else {
            const float inv = 1.0f / sp;
            ud = u * inv;
            vd = v * inv;
            spd = sp;
            if (ud < 0.0f) {              // past +/-90 deg: point forward, spin back
              ud = -ud;
              vd = -vd;
              spd = -spd;
            }
            // A half-turn swing across the fold for a small change of
            // direction: keep the knuckle's side, at the limit, and reverse.
            if (c[k] * ud + s[k] * vd < 0.0f && ud < sin_hyst_) {
              ud = 0.0f;
              vd = vd > 0.0f ? -1.0f : 1.0f;
              spd = -spd;
            }
          }
          const float dot = c[k] * ud + s[k] * vd;
          if (dot >= cos_step) {
            c[k] = ud;
            s[k] = vd;
          } else {
            const float sg = (c[k] * vd - s[k] * ud) >= 0.0f ? 1.0f : -1.0f;
            float nc = c[k] * cos_step - sg * s[k] * sin_step;
            float ns = s[k] * cos_step + sg * c[k] * sin_step;
            const float f = 1.5f - 0.5f * (nc * nc + ns * ns);   // keep unit length
            c[k] = nc * f;
            s[k] = ns * f;
          }
          gate = std::min(gate, gateFactor(std::max(0.0f, c[k] * ud + s[k] * vd)));
          speed[k] = spd;
        }

        float worst = 0.0f;
        float target[4];
        for (int k = 0; k < 4; k++) {
          target[k] = gate * speed[k];
          worst = std::max(worst, std::abs(target[k] - w[k]));
        }
        const float kf = worst <= wheel_step ? 1.0f : wheel_step / worst;
        float su = 0.0f, sv = 0.0f, sw = 0.0f;
        for (int k = 0; k < 4; k++) {
          w[k] += kf * (target[k] - w[k]);
          const float uu = w[k] * c[k], vv = w[k] * s[k];
          su += uu;
          sv += vv;
          sw += wx_[k] * vv - wy_[k] * uu;
        }
        state.vx(i, j) = 0.25f * su;
        state.vy(i, j) = 0.25f * sv;
        state.wz(i, j) = sw * inv_sum_r2_;
      }
    }
  }

private:
  float gateFactor(float x) const
  {
    switch (int_power_) {
      case 10: {float x2 = x * x, x4 = x2 * x2; return x4 * x4 * x2;}
      case 1: return x;
      case 2: return x * x;
      case 4: {float x2 = x * x; return x2 * x2;}
      case 0: return 1.0f;
      default:
        if (int_power_ > 0) {
          float r = 1.0f, b = x;
          for (int e = int_power_; e; e >>= 1, b *= b) {if (e & 1) {r *= b;}}
          return r;
        }
        return std::pow(x, gate_power_);
    }
  }

  void onJoints(const sensor_msgs::msg::JointState & msg)
  {
    std::array<float, 4> a;
    int found = 0;
    for (size_t n = 0; n < msg.name.size() && n < msg.position.size(); n++) {
      for (int k = 0; k < 4; k++) {
        if (msg.name[n] == steer_joints_[k]) {
          a[k] = static_cast<float>(msg.position[n]);
          found |= 1 << k;
        }
      }
    }
    if (found != 0xF) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (int k = 0; k < 4; k++) {
      fb_c_[k] = std::cos(a[k]);
      fb_s_[k] = std::sin(a[k]);
      if (fb_c_[k] < 0.0f) {fb_c_[k] = -fb_c_[k]; fb_s_[k] = -fb_s_[k];}
    }
    fb_time_ = std::chrono::steady_clock::now();
    have_fb_ = true;
  }

  void initialKnuckles(
    const models::State & state, std::array<float, 4> & c, std::array<float, 4> & s)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - fb_time_).count();
      if (have_fb_ && age < feedback_timeout_) {
        c = fb_c_;
        s = fb_s_;
        known_c_ = c;
        known_s_ = s;
        return;
      }
    }
    // No fresh feedback: the knuckles the current twist implies, or, when
    // stopped, wherever they were last known to be.
    const float vx = state.vx(0, 0), vy = state.vy(0, 0), wz = state.wz(0, 0);
    for (int k = 0; k < 4; k++) {
      const float u = vx - wz * wy_[k], v = vy + wz * wx_[k];
      const float n = std::sqrt(u * u + v * v);
      if (n > 1e-3f) {
        const float sg = u < 0.0f ? -1.0f : 1.0f;
        known_c_[k] = sg * u / n;
        known_s_[k] = sg * v / n;
      }
    }
    c = known_c_;
    s = known_s_;
  }

  float half_wheelbase_{0.247f}, half_track_{0.1852f};
  float steer_rate_{4.7f}, gate_power_{10.0f}, wheel_accel_{1.0f};
  double feedback_timeout_{0.5};
  float command_delay_{0.0f};
  float steer_deadband_{0.03f};
  float sin_hyst_{0.1736f};
  int int_power_{10};
  std::string joint_state_topic_;
  std::vector<std::string> steer_joints_;
  std::array<float, 4> wx_{}, wy_{};
  float inv_sum_r2_{1.0f};

  std::mutex mutex_;
  bool have_fb_{false};
  std::chrono::steady_clock::time_point fb_time_{};
  std::array<float, 4> fb_c_{}, fb_s_{};
  std::array<float, 4> known_c_{}, known_s_{};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__FOUR_WIS_MOTION_MODEL_HPP_
