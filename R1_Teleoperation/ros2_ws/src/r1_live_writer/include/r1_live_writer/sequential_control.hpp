#pragma once

#include <algorithm>
#include <cstdint>
#include <string>

namespace r1_live_writer
{

// Policy only: the caller authorizes each tick and acknowledges every queued
// release/zero frame. No sleeps, mode writes or robot objects live here.
class SequentialControl
{
public:
  enum class Phase {Upper, Release, AwaitWalk, Walk, Stop, Fault};
  struct Input
  {
    double now{0};
    bool motion{false};
    bool tracking{false};
    bool owns_upper{false};
    bool mode_valid{false};
    int mode{-1};
    double mode_requested_at{-1};
    bool legs_quiet{false};
    std::uint64_t state_sequence{0};
  };

  Phase phase() const {return phase_;}
  const char * name() const
  {
    switch (phase_) {
      case Phase::Upper: return "arms";
      case Phase::Release: return "releasing_arms";
      case Phase::AwaitWalk: return "waiting_811";
      case Phase::Walk: return "walking";
      case Phase::Stop: return "stopping";
      case Phase::Fault: return "fault";
    }
    return "fault";
  }
  const std::string & error() const {return error_;}
  double phase_started() const {return phase_started_;}
  void reset() {*this = SequentialControl{};}

  void update(const Input & in)
  {
    const bool intent = in.motion && in.tracking;
    if (!intent) {intent_since_ = -1;}
    else if (intent_since_ < 0) {intent_since_ = in.now;}
    const bool requested = intent && in.now - intent_since_ >= 0.08;
    const bool walking_mode = in.mode_valid && in.mode == 811 &&
      in.mode_requested_at >= phase_started_;
    switch (phase_) {
      case Phase::Upper:
        if (requested) {
          enter(in.owns_upper ? Phase::Release : Phase::AwaitWalk, in.now);
        }
        break;
      case Phase::Release:
        // Never reverse an ownership ramp when the operator changes their mind.
        if (in.now - phase_started_ > 3.0) {fail("release_timeout");}
        break;
      case Phase::AwaitWalk:
        if (walking_mode) {enter(requested ? Phase::Walk : Phase::Stop, in.now);}
        else if (in.now - phase_started_ > 3.0) {fail("fsm_811_timeout");}
        break;
      case Phase::Walk:
        if (!intent || !in.mode_valid || in.mode != 811) {
          enter(Phase::Stop, in.now);
        }
        break;
      case Phase::Stop: {
        // Stop always sends fresh zeros. A resumed stick can reuse the still
        // released walking controller without reacquiring the upper body.
        if (requested && walking_mode) {enter(Phase::Walk, in.now); break;}
        const bool new_state = in.state_sequence != last_state_;
        if (new_state) {
          last_state_ = in.state_sequence;
          if (in.legs_quiet && walking_mode) {
            if (quiet_since_ < 0) {quiet_since_ = in.now;}
            ++quiet_samples_;
          } else {quiet_since_ = -1; quiet_samples_ = 0;}
        }
        if (new_state && zero_frames_ >= 6 && quiet_samples_ >= 5 && quiet_since_ >= 0 &&
          in.now - quiet_since_ >= 0.25 && walking_mode)
        {
          enter(Phase::Upper, in.now);
          intent_since_ = -1;
        } else if (in.now - phase_started_ > 5.0) {fail("stop_feedback_timeout");}
        break;
      }
      case Phase::Fault: break;
    }
  }

  bool release_due(double now) const
  {
    return phase_ == Phase::Release && release_frames_ < 130 &&
      now - last_release_ >= 0.009;
  }
  double release_weight() const
  {
    return std::max(0.0, 1.0 - static_cast<double>(release_frames_ + 1) / 100.0);
  }
  void release_frame_sent(double now)
  {
    last_release_ = now;
    ++release_frames_;
    // Same 100-step vendor ramp and 30 zero frames as teardown, distributed
    // over timer callbacks so VR emergency events remain dispatchable.
  }
  bool release_complete(double now) const
  {
    return phase_ == Phase::Release && release_frames_ >= 130 &&
      now - last_release_ >= 0.01;
  }
  void release_finished(double now) {enter(Phase::AwaitWalk, now);}
  void zero_sent() {if (phase_ == Phase::Stop) {++zero_frames_;}}

private:
  void fail(const char * reason) {phase_ = Phase::Fault; error_ = reason;}
  void enter(Phase phase, double now)
  {
    phase_ = phase;
    phase_started_ = now;
    quiet_since_ = -1;
    quiet_samples_ = 0;
    zero_frames_ = 0;
    release_frames_ = 0;
    last_release_ = -1;
    last_state_ = 0;
  }
  Phase phase_{Phase::Upper};
  double phase_started_{0}, intent_since_{-1}, quiet_since_{-1}, last_release_{-1};
  unsigned release_frames_{0}, quiet_samples_{0}, zero_frames_{0};
  std::uint64_t last_state_{0};
  std::string error_;
};
}  // namespace r1_live_writer
