#ifndef COHERE_TURN_ACTIONS_HPP_INCLUDED
#define COHERE_TURN_ACTIONS_HPP_INCLUDED

#include <model/action_base.hpp>

namespace model {
  namespace actions {

    template <typename Agent>
    class cohere_centroid_distance
    { // cohere by turning with all neighbors depending on distance to the centroid

      make_action_from_this(cohere_centroid_distance);

    public:
      cohere_centroid_distance() {}
      cohere_centroid_distance(size_t, const json& J)
      {
        topo = J["topo"];    // [1]

        float fov = J["fov"]; // [deg]
        cfov = glm::cos(glm::radians(180.0f - 0.5f * (360.0f - fov))); // [1]

        float maxdist = J["maxdist"];     // [m]
        maxdist2 = maxdist * maxdist;     // [m^2]

        //max_w_dist_ = J["max_w_dist"];     // [m]
        //min_w_dist_ = J["min_w_dist"];     // [m]

        w_ = J["w"];                       // [1]
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

        auto ofss = vec3(0.f);
        auto n = 0.f; // number of neighbors
        auto realized_topo = while_topo(sv, topo, [&](const auto& ni) {

          if (in_fov(self, ni.dist2, flock[ni.idx].pos, this))
          {
            ofss += space::ofs(self->pos, flock[ni.idx].pos);
            ++n;
            return true;
          }
          return false;
        });

        const auto w_scaled = (realized_topo) ? w_ * glm::length(ofss / n) : 0.f; // math::smootherstep(glm::length(ofss / n), min_w_dist_, max_w_dist_);
        const auto Fdir =  math::save_normalize(ofss, vec3(0.f)) * w_scaled;
        self->steering += Fdir;
      }

    public:
      int topo = 0;           // [1]
      float cfov = 0;         // [1]
      float maxdist2 = 0;     // [m^2]

    private:
      float w_ = 0;           // [1]
     // float max_w_dist_;      // [m]
     // float min_w_dist_;      // [m]
    };
  }
}

#endif