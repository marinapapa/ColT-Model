#ifndef ROOSTING_HPP_INCLUDED
#define ROOSTING_HPP_INCLUDED

#include <model/action_base.hpp>

namespace model {
	namespace actions {

		template <typename Agent>
		class relative_roosting_persistant
		{
			make_action_from_this(relative_roosting_persistant);

		public:
			relative_roosting_persistant() {}
			relative_roosting_persistant(size_t, const json& J)
			{
				dist_to_home_ = float(J["home_dist"]);            // [m]
				angl_to_home_ = glm::radians(float(J["home_direction"]));       // [deg]
				w_ = J["w"];                       // [1]
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				// homing position relative to its flock current position
				const auto& this_flock = sim.flocks<Tag>()[sim.flock_of<Tag>(idx)];
				const auto& flock_pos = this_flock.gc();
				const auto& flock_head = math::save_normalize(this_flock.vel, vec3(0.f));
				home_pos_ = flock_pos + dist_to_home_ * math::rotate_xy(flock_head, angl_to_home_);
			}

			void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				const auto ofss = space::ofs(self->pos, home_pos_);
				const vec3 Fdir = math::save_normalize(ofss, vec3(0.f)) * w_;
				self->steering += Fdir;
			}

		private:
			vec3 home_pos_ = { 0,0,0 };  // []
			float dist_to_home_ = 0;    // [m]
			float angl_to_home_ = 0;	  // [rad]
			float w_ = 0;               // [1]
		};

  }
}


#endif