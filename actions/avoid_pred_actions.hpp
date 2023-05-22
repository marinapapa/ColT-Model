#ifndef AVOID_PRED_ACTIONS_HPP_INCLUDED
#define AVOID_PRED_ACTIONS_HPP_INCLUDED

#include <action_base.hpp>
#include <glmutils/ray.hpp>
#include <glmutils/random.hpp>
#include <model/while_topo.hpp>
#include <agents/predator.hpp>

namespace model {
  namespace actions {

	//turn random degrees within a window in given time as reaction to a predator
	template <typename Agent>
	class random_t_turn_gamma_pred
	{
		make_action_from_this(random_t_turn_gamma_pred);

	public:
		random_t_turn_gamma_pred() {}
		random_t_turn_gamma_pred(size_t, const json& J)
		{
			const float turn_mean = glm::radians(float(J["turn_mean"]));
			const float turn_sd = glm::radians(float(J["turn_sd"]));
			const float time_mean = J["time_mean"];
			const float time_sd = J["time_sd"];

			if (turn_mean <= 0.f || turn_sd <= 0.f || time_mean <= 0.f || turn_mean == 0.f) throw std::runtime_error("wrong parameters in random_t_turn");

			const float turn_alpha = (turn_mean / turn_sd) * (turn_mean / turn_sd);
			const float turn_beta = (turn_sd * turn_sd) / turn_mean ;

			const float time_alpha = (time_mean / time_sd) * (time_mean / time_sd);
			const float time_beta = (time_sd * time_sd) / time_mean;

			turn_distr_ = std::gamma_distribution<float>(turn_alpha, turn_beta);
			time_distr_ = std::gamma_distribution<float>(time_alpha, time_beta);

			turn_dur_ = static_cast<tick_t>(turn_mean / Simulation::dt());
		}
		void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
		{
			if (state_dur > turn_dur_) { state_exit_t -= (state_dur - turn_dur_); }
		}

		void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
		{
			// we want to turn turn_ radians in time_ seconds.
			auto loc_time = 0.f; // random to initialize
			auto thisturn = 0.f; // random to initialize
			do {
				loc_time = time_distr_(model::reng);
				thisturn = turn_distr_(model::reng);
			} while ( loc_time * thisturn <= 0.f ); // both not 0

			turn_dur_ = static_cast<tick_t>(static_cast<double>(loc_time) / Simulation::dt());

			auto w = thisturn / loc_time;       // required angular velocity
			r_ = self->speed / w;       // radius

			// find direction away from predator
			const auto nv = sim.sorted_view<Tag, pred_tag>(idx);

			if (nv.size())
			{
				const auto& predator = sim.pop<pred_tag>()[nv[0].idx];    // nearest predator
				const float rad_away_pred = math::rad_between_xy(predator.dir, self->dir);
				w_ = std::copysignf(1.f, rad_away_pred);
			}
			else
			{
				w_ = 0.f;
			}
		}

		void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
		{
			// Fz = m * v*v/r 
			turn_dir_ = w_ * glmutils::perpDot(self->dir);
			auto Fz = self->ai.bodyMass * self->speed * self->speed / r_;
			self->steering += Fz * turn_dir_;

		}

	private:
		float r_ = 0;
		vec3 turn_dir_;
		tick_t turn_dur_;
		std::gamma_distribution<float> turn_distr_;
		std::gamma_distribution<float> time_distr_;
		float w_ = 0;      // [1] 
	};

	template <typename Agent>
	class copy_escape
	{ // cohere by turning with all neighbors

			make_action_from_this(copy_escape);

	public:
			copy_escape() {}
			copy_escape(size_t, const json& J)
			{
					topo = J["topo"];    // [1]
					//cfov = glm::cos(glm::radians(180.0f)); // [1]
					float fov = J["fov"]; // [deg]
					cfov = glm::cos(glm::radians(180.0f - 0.5f * (360.0f - fov))); // [1]

					float maxdist = J["maxdist"];     // [m]
					maxdist2 = maxdist * maxdist;     // [m^2]
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
			}

			void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
					const auto sv = sim.sorted_view<Tag>(idx);
					const auto& flock = sim.pop<Tag>();

					tick_t time_left = 0;
					int state2copy = 0;

					for (auto it = sv.cbegin(); it != sv.cbegin() + topo; ++it) {
							if (in_fov(self, it->dist2, flock[it->idx].pos, this) && it->is_esc)
							{
									time_left = it->esc_t_left;
									state2copy = it->state;
									break;
							}
					}

					self->copy_duration = time_left;
					self->copy_state = state2copy;
			}

	public:
			int topo = 0;           // [1]
			float cfov = 0;         // [1]
			float maxdist2 = 0;     // [m^2]
	};

  }
}


#endif