#ifndef MODEL_STATES_PERSISTENT_HPP_INCLUDED
#define MODEL_STATES_PERSISTENT_HPP_INCLUDED

#include <model/state_base.hpp>


namespace model {
  namespace states {


    template <typename IP>
    class persistent : public state<typename IP::agent_type>
    {
      make_state_from_this(persistent);
    
    public:
      persistent(size_t idx, const json& J) :
        actions(IP::create(idx, J["actions"])), 
        duration_(static_cast<tick_t>(double(J["duration"]) / Simulation::dt())) // [tick]
      {
        effective_dur_ = duration_;
	    	sai_ = flight::create_state_aero<float>(J["aeroState"]);
        tr_ = std::max(tick_t(1), static_cast<tick_t>(double(J["tr"]) / Simulation::dt())); // [tick]
        //normalize_actions<0>();
      }

      template <size_t I>
      void check_actions_exit(const tick_t& dur, tick_t& exit_tick)
      { 
          std::get<I>(actions).check_state_exit(dur, exit_tick); 
          check_actions_exit<I + 1>(dur, exit_tick); 
      } 
      template <> 
      void check_actions_exit<action_pack::size>(const tick_t& dur, tick_t& exit_tick)
      { 
      }

      // to be used only by starling agents
      void check_state_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim) override
      {
          //if (self->copy_duration > 0.f) {
          //effective_dur_ = self->copy_duration;
          //}
          self->copy_duration = tick_t(0);
          self->copy_state = 0;
      }

      void enter(agent_type* self, size_t idx, tick_t T, const Simulation& sim) override
      {
          self->state_timer = effective_dur_;
          t_exit_ = T + effective_dur_;
          if (tr_ < 1) throw std::runtime_error("Reaction time smaller than 1");

          chain_on_entry<0>(self, idx, T, sim);
          check_actions_exit<0>(effective_dur_, t_exit_);
      }

      void resume(agent_type* self, size_t idx, size_t T, const Simulation& sim) override
      {
	       self->reaction_time = tr_;
   	     self->sa = sai_;
         self->sa.cruiseSpeed += self->ai.cruiseSpeedSd;

        chain_actions<0>(self, idx, T, sim);
        self->state_timer = t_exit_ - T;
        if (T >= t_exit_) {
            effective_dur_ = duration_;
            self->on_state_exit(idx, T, sim);
        }
      };
    public:
      tick_t t_exit_;
    protected:
      tick_t tr_;        // [tick]
      tick_t duration_;  // [tick]
      tick_t effective_dur_;
	    flight::state_aero<float> sai_; // state specific aero info
    };

  }
}

#endif
