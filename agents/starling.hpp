#ifndef STARLING_HPP_INCLUDED
#define STARLING_HPP_INCLUDED

#include <istream>
#include <ostream>
#include <model/json.hpp>
#include <libs/math.hpp>
#include <libs/space.hpp>
#include <glmutils/random.hpp>
#include <glm/glm.hpp>
#include <states/transient.hpp>
#include <states/persistent.hpp>
#include <actions/align_actions.hpp>
#include <actions/cohere_actions.hpp>
#include <actions/avoid_actions.hpp>
#include <actions/predator_actions.hpp>
#include <actions/avoid_pred_actions.hpp>
#include <actions/no_interacting_actions.hpp>
#include <actions/roosting.hpp>
#include <stress/sources.hpp>
#include <model/stress_base.hpp>
#include <model/transitions.hpp>
#include <model/flight_control.hpp>
#include <model/flight.hpp>


namespace model {
  
  template <>
  struct known_color_maps<starling_tag> 
  {
    static constexpr size_t size = 7;
    static constexpr const char* descr[size] = {
      "none",
	    "idx",
      "speed",
      "banking",
      "state",
      "nnd",
	    "flock"
    };
  };


  template <>
  struct snapshot_entry<starling_tag>
  {
    vec3 pos = vec3(0);
	  vec3 dir = vec3(0);
	  float speed = 0.f;
    vec3 accel = vec3(0);
    float stress = 0.f;

    static std::istream& stream_from_csv(std::istream& is, snapshot_entry<starling_tag>& e)
    {
      char delim;
      float discard;
      is >> discard >> delim; // discard id in local variable
      is >> e.pos.x >> delim >> e.pos.y >> delim >> e.pos.z >> delim;
      is >> e.dir.x >> delim >> e.dir.y >> delim >> e.dir.z >> delim;
	    is >> e.speed >> delim >> e.accel.x >> delim; 
      is >> e.accel.y >> delim >> e.stress;
      return is;
    }

    static std::ostream& stream_to_csv(std::ostream& os, const snapshot_entry<starling_tag>& e)
    {
      char delim = ',' ; 
      os << e.pos.x << delim << e.pos.y << delim << e.pos.z << delim;
      os << e.dir.x << delim << e.dir.y << delim << e.dir.z << delim << e.speed << delim;
      os << e.accel.x << delim << e.accel.y << delim;
      os << e.stress;
      return os;
    }
  };


  class Starling
  {
  public:
    using Tag = starling_tag;

    static constexpr const char* name() { return "Starling"; }

    using AP = states::package<
      states::transient<actions::package<Starling, // normal flocking
        actions::align_n<Starling>,
		    actions::cohere_centroid_distance<Starling>,
        actions::avoid_n_position<Starling>,
        actions::copy_escape<Starling>,
        actions::wiggle<Starling>
        >>, 
        states::persistent<actions::package<Starling, // escape penalty
            actions::wiggle<Starling>,
        actions::align_n<Starling>,
        actions::cohere_centroid_distance<Starling>,
        actions::avoid_n_position<Starling>
        >>,
        states::persistent<actions::package<Starling, // escaoe twi
  	        actions::relative_roosting_persistant<Starling>, // switch to: random_t_turn_gamma_pred for unidirectional escape
            actions::align_n<Starling>,
            actions::cohere_centroid_distance<Starling>,
            actions::avoid_n_position<Starling>,
            actions::wiggle<Starling>
       >>
    >;
    using stress_accum = stress::accumulator<Starling,
      stress::predator_distance<Starling>
    >;
    using transitions = transitions::piecewise_linear_interpolator<AP::transition_matrix, 3>; // based on transition cuts of interpolation

  public:
    Starling(Starling&&) = default;
    Starling(size_t idx, const json& J);

    void initialize(size_t idx, const Simulation& sim, const json& J);

    // returns next update time
    tick_t update(size_t idx, tick_t T, const Simulation& sim);
    void integrate(tick_t T, const Simulation& sim);
    void on_state_exit(size_t idx, tick_t T, const Simulation& sim);

    ::model::instance_proxy instance_proxy(long long color_map, size_t idx, const class Simulation* sim) const noexcept;
    ::model::snapshot_entry<Tag> snapshot(const Simulation* sim, size_t idx) const noexcept;
    void snapshot(Simulation* sim, size_t idx, const snapshot_entry<Tag>& se) noexcept;
    static float distance2(const vec3& a, const vec3& b) { return glm::distance2(a, b); }
    static float bearing_angl(const vec3& d, const vec3& a, const vec3& b) { return math::rad_between_xy(d, space::ofs(a, b)); }
   
    const int& get_current_state() const noexcept { return current_state_; }

  public:
    // accessible from states:
    vec3 pos;   // [m]
    vec3 dir;
    float speed;  // [m/tick]
    float ang_vel = 0; // [ 1/s ] Only for extracting data, not used in model
    vec3 accel;  // [m/tick ^ 2]
    tick_t reaction_time = 0;   // [ticks]
    tick_t last_update = 0; 
    float stress;
    std::array<float, AP::size> tm; // transition matrix line per state change evaluation for export
    vec3 force;             // reserved for physical forces  [kg * m/tick^2]
    vec3 steering;    // linear, lateral  [kg * m/tick^2]

    tick_t copy_duration;    // for copying neighbor maneuver, how much time to stay in persistent state
    int copy_state;
    tick_t state_timer;    // to be copyied by neighbors

    flight::aero_info<float> ai;
    flight::state_aero<float> sa;
    static std::vector<snapshot_entry<Tag>> init_pop(const Simulation& sim, const json& J);

  private:
    static transitions transitions_;
    int current_state_ = 0;
    float stress_ofs_; // stress offset (individual variation)
    AP::package_array pa_;
    typename stress_accum::package_tuple sp_;
  };

}
#endif
