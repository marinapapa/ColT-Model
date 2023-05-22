#ifndef OTHER_ACTIONS_HPP_INCLUDED
#define OTHER_ACTIONS_HPP_INCLUDED

#include <model/action_base.hpp>

namespace model {
  namespace actions {

    template <typename Agent>
    class wiggle
    {
      make_action_from_this(wiggle);

    public:
      wiggle() {}
      wiggle(size_t, const json& J)
      {
		    w_ = J["w"];               // [deg/s]
      }

      void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
      }

      void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
      {
      }

      void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
        auto w = std::uniform_real_distribution<float>(-w_, w_)(reng); // [rad]
		    self->steering += glmutils::perpDot(self->dir) * w;
      }

    private:
      float w_ = 0;      // [1] 

    };
  }
}


#endif