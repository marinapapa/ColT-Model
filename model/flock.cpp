#include <queue>
#include <algorithm>
#include <libs/math.hpp>
#include <libs/space.hpp>
#include <libs/graph.hpp>
#include <glmutils/oobb.hpp>
#include <agents/agents.hpp>
#include <model/flock.hpp>
#include <model/simulation.hpp>

namespace model {

  void flock_tracker::cluster(float dd)
  {
    flock_id_.assign(proxy_.size(), no_flock);
    const auto n = proxy_.size();
    auto cc = graph::connected_components(0, static_cast<int>(n), [&](int i, int j) {
      return dd > glm::distance2(proxy_[i].pos, proxy_[j].pos);
    });
    descr_.clear();
    for (unsigned ci = 0; ci < static_cast<unsigned>(cc.size()); ++ci) {
      vpos_.clear();
      vvel_.clear();
      vec3 vel = vec3(0);
      float pol = 0.f;
      for (auto i : cc[ci]) {
        flock_id_[proxy_[i].idx] = ci;
        vpos_.emplace_back(space::ofs(proxy_[cc[ci][0]].pos, proxy_[i].pos));
        vvel_.emplace_back(proxy_[i].vel);
        vel += proxy_[i].vel;
      }
      vec3 ext;
      auto H = glmutils::oobb(static_cast<int>(cc[ci].size()), vpos_.begin(), ext);
      vel /= cc[ci].size();
      std::for_each(vvel_.begin(), vvel_.end(), [&](const auto& veli) { 
      pol += glm::dot(math::save_normalize(veli, vec3(0.f)), math::save_normalize(vel, vec3(0.f))); });
      pol /= cc[ci].size();
      H[2] += glm::vec4(proxy_[cc[ci][0]].pos, 0.f);
      descr_.push_back({ vpos_.size(), vel, pol, H, ext });
      int x = 0;
    }
  }


  void flock_tracker::track()
  {
    const auto dt = Simulation::dt();
    for (auto& fd : descr_) {
      vec3 gc = fd.gc();
      fd.H[2] = gc + dt * fd.vel;
    }
  }

}
