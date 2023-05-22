#ifndef PRED_HPP_INCLUDED
#define PRED_HPP_INCLUDED

#include <math.hpp>
#include <agents/agents.hpp>
#include <states/transient.hpp>
#include <actions/align_actions.hpp>
#include <actions/cohere_actions.hpp>
#include <actions/avoid_actions.hpp>
#include <actions/no_interacting_actions.hpp>
#include <actions/hunt_actions.hpp>
#include <stress/sources.hpp>
#include <model/stress_base.hpp>
#include <model/transitions.hpp>
#include <model/flight_control.hpp>
#include <model/flight.hpp>
#include <model/json.hpp>


namespace model {
  
  template <>
  struct known_color_maps<pred_tag>
  {
    static constexpr size_t size = 4;
    static constexpr const char* descr[] = {
      "none",
      "speed",
      "state",
      "banking"
    };
  };


  template <>
  struct snapshot_entry<pred_tag>
  {
    vec3 pos = vec3(0);
    vec3 dir = vec3(0);
    float speed = 0.f;
    vec3 accel = vec3(0);

    static std::istream& stream_from_csv(std::istream& is, snapshot_entry<pred_tag>& e)
    {
      char delim;
      float discard;
      is >> discard >> delim; // discard id in local variable
      is >> e.pos.x >> delim >> e.pos.y >> delim;
      is >> e.dir.x >> delim >> e.dir.y >> delim;
      is >> e.speed >> delim >> e.accel.x >> delim;
      is >> e.accel.y;
      return is;
    }

    static std::ostream& stream_to_csv(std::ostream& os, const snapshot_entry<pred_tag>& e)
    {
      char delim[] = ", ";
      os << e.pos.x << delim << e.pos.y << delim;
      os << e.dir.x << delim << e.dir.y << delim << e.speed << delim;
      os << e.accel.x << delim << e.accel.y << delim;
      return os;
    }

  };



  class Pred
  {
  public:
    using Tag = pred_tag;

    static constexpr const char* name() { return "Pred"; }

    using AP = states::package<
      states::transient<actions::package<Pred,
        actions::set_retreat<Pred>
      >>,
      states::persistent<actions::package<Pred,
        actions::select_flock<Pred>,
        actions::shadowing<Pred>
      >>,
      states::persistent < actions::package < Pred,
      actions::wiggle<Pred>,
      actions::chase_closest_prey<Pred>
      >>,
      states::persistent<actions::package<Pred,
      actions::wiggle<Pred>,
      actions::avoid_closest_prey<Pred>,
      actions::hold_current<Pred>
      >>
      >;
    using transitions = transitions::piecewise_linear_interpolator<AP::transition_matrix, 1>;

  public:
    Pred(Pred&&) = default;
    Pred(size_t idx, const json& J);
    void initialize(size_t idx, const Simulation& sim, const json& J);

    // returns next update time
    tick_t update(size_t idx, tick_t T, const Simulation& sim);
    void integrate(tick_t T, const Simulation& sim);
    void on_state_exit(size_t idx, tick_t T, const Simulation& sim);

    static float distance2(const vec3& a, const vec3& b) {
      return glm::distance2(a, b);
    }
    static float bearing_angl(const vec3& d, const vec3& a, const vec3& b) {
      return math::rad_between_xy(d, space::ofs(a, b));
    }

    ::model::instance_proxy instance_proxy(long long color_map, size_t idx, const class Simulation* sim) const noexcept;
    ::model::snapshot_entry<Tag> snapshot(const Simulation* sim, size_t idx) const noexcept;
    void snapshot(Simulation* sim, size_t idx, const snapshot_entry<Tag>& se) noexcept;
    static std::vector<snapshot_entry<Tag>> init_pop(const Simulation& sim, const json& J);

    const int& get_current_state() const noexcept { return current_state_; }

  public:
    // accessible from states
    vec3 pos;
    vec3 dir;
    float ang_vel = 0; // [ 1/s ] Only for extracting data, not used in model
    tick_t reaction_time = 0;   // [ticks]
    tick_t last_update = 0;
    tick_t copy_duration = 0;
    float speed = 0.f;            // [m/tick]
    vec3 accel;  // [m/tick ^ 2]
    vec3 force;             // reserved for physical forces  [kg * m/tick^2]
    vec3 steering;    // linear, lateral  [kg * m/tick^2]
    int target = -1;
    tick_t state_timer;    // to be copyied by neighbors
    int copy_state;    // to be copyied by neighbors

    flight::aero_info<float> ai;
	  flight::state_aero<float> sa;

  private:
    int current_state_ = 0;
    static transitions transitions_;
    AP::package_array pa_;
  };


}
#endif
