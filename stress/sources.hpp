#ifndef STRESS_SOURCES_HPP_INCLUDED
#define STRESS_SOURCES_HPP_INCLUDED

#include <model/stress_base.hpp>


namespace model {
  namespace stress {

    template <typename Agent>
    class predator_distance
    {
      make_stress_source_from_this(predator_distance);

    public:
      predator_distance() = default;
      predator_distance(size_t, const json& J)
      {
        w_ = J["w"];
        shape_ = J["distr_shape"];
      }

      void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
        auto ip = sim.sorted_view<Tag, pred_tag>(idx);
        if (!ip.empty()) {
            self->stress += w_ * std::exp( - std::sqrt(ip[0].dist2) / shape_);
        }
      }

    private:
      float w_ = 0;             // [1] stress weight
      float shape_ = 0;   // shape of exponential function
    };


    template <typename Agent>
    class neighbors_stress
    {
      make_stress_source_from_this(neighbors_stress);

    public:
      neighbors_stress() = default;
      neighbors_stress(size_t, const json& J)
      {    
        w_ = J["w"];          // [1]
        topo_ = J["topo"];     // [1]
        float fov = J["fov"];  // [deg]
        cfov_ = glm::cos(glm::radians(180.0f - 0.5f * (360.0f - fov))); // [1]
      }

      void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
        const auto sv = sim.sorted_view<Tag>(idx);
        const auto& flock = sim.pop<Tag>();

        float n_str = 0;
        auto realized_topo = while_topo(sv, topo_, [&](const auto& ni) {
          const auto offs = space::ofs(self->pos, flock[ni.idx].pos);
          if (glm::dot(self->dir, offs) > glm::sqrt(ni.dist2) * cfov_)
          {
            n_str += math::smootherstep(flock[ni.idx].stress, 0.f, 1.f);
            return true;
          }
          return false;
        });

        if (realized_topo) {
          n_str /= static_cast<float>(realized_topo);
          self->stress += w_ * n_str;
        }
      }

    private:
      float w_ = 0;     // [1] stress weight
      size_t topo_ = 0;  // [1]
      float cfov_ = 0;   // [1]
    };
  }
}
#endif
