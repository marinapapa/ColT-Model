#ifndef DIFFUSION_OBS_HPP_INCLUDED
#define DIFFUSION_OBS_HPP_INCLUDED


#include <memory>
#include <future>
#include <deque>
#include <set>
#include <algorithm>
#include <tbb/tbb.h>
#include <hrtree/sorting/insertion_sort.hpp>
#include <model/observer.hpp>
#include <agents/agents.hpp>


namespace analysis {


  namespace diffusion {

    struct snapshot_data
    {
      model::vec3 pos;
      model::vec3 dir;
      std::vector<model::neighbor_info> ninfo;
    };

    using snapshot_t = std::vector<snapshot_data>;
    using window_t = std::deque<snapshot_t>;


    // returns Qm(t)
    inline std::vector<float> Qm(const window_t& win, size_t max_topo)
    {
      const auto T = win.size();      // time steps
      const auto N = win[0].size();   // individuals
      const auto I = max_topo;        // neighbors to consider
      auto qmt = std::vector<size_t>(T, 0);
      auto M0 = std::vector<unsigned>(I);
      auto Mt = std::vector<unsigned>(I);
      for (auto n = 0; n < N; ++n) {
        std::transform(win[0][n].ninfo.cbegin(), win[0][n].ninfo.cbegin() + max_topo, M0.begin(), [](const auto& ni) { return ni.idx; });
        hrtree::insertion_sort(M0.begin(), M0.end());
        for (auto t = 1; t < T; ++t) {
          std::transform(win[t][n].ninfo.cbegin(), win[t][n].ninfo.cbegin() + max_topo, Mt.begin(), [](const auto& ni) { return ni.idx; });
          hrtree::insertion_sort(Mt.begin(), Mt.end());
          auto it = std::set_intersection(M0.cbegin(), M0.cend(), Mt.cbegin(), Mt.cend(), Mt.begin());
          qmt[t] += std::distance(Mt.begin(), it);
        }
      }
      auto qmtd = std::vector<float>(T);
      std::transform(qmt.cbegin(), qmt.cend(), qmtd.begin(), [S = N * I](const auto& x) { return float(double(x) / S); });
      qmtd[0] = 1.0;
      return qmtd;
    }

    // returns r^2(t)
    inline std::vector<float> R(const window_t& win)
    {
      const auto T = win.size();      // time steps
      const auto N = win[0].size();   // individuals
      auto dev = std::vector<float>(T, 0);
      auto cm = std::vector<model::vec3>(T, { 0,0,0 });
      for (auto t = 0; t < T; ++t) {
        // center of mass
        const auto pc = win[t][0].pos;    // pick one
        for (auto n = 0; n < N; ++n) {
          cm[t] += space::ofs(pc, win[t][n].pos);
        }
        cm[t] = cm[t] / float(N);
      }
      for (auto t = 0; t < T; ++t) {
        for (auto n = 0; n < N; ++n) {
          const auto r0 = space::ofs(win[0][n].pos, cm[0]);
          const auto rt = space::ofs(win[t][n].pos, cm[t]);
          dev[t] += glm::distance2(rt, r0);
        }
        dev[t] /= (N * T);
      }
      return dev;
    }

    // map npos into reference frame [pos, dir]
    inline model::vec3 map(const model::vec3& dir, const model::vec3& pos, const model::vec3& npos)
    {
      const auto rpos = space::ofs(pos, npos);
      return model::vec3{ glmutils::perpDot(dir, rpos), glm::dot(dir, rpos), 0.f };
    }

    inline model::vec3 anti_rotate(const model::vec3& a, const model::vec3& b, const model::vec3& r)
    {
      const auto c = glm::dot(a, b);
      const auto s = glmutils::perpDot(a, b);
      const auto Rz = glm::mat3(
        c, s, 0,
        -s, c, 0,
        0, 0, 1
      );
      return Rz * r;
    }

    // returns Deviation^2(t) from position in formation flight
    inline std::vector<float> Dfor(const window_t& win, size_t topo)
    {
      const auto T = win.size();      // time steps
      const auto N = win[0].size();   // individuals
      auto dev = std::vector<double>(T, 0);
      for (auto n = 0; n < N; ++n) {
        const auto nid = win[0][n].ninfo[topo].idx;
        const auto npos0 = map(win[0][n].dir, win[0][n].pos, win[0][nid].pos);
        for (auto t = 1; t < T; ++t) {
          const auto npos1 = map(win[t][n].dir, win[t][n].pos, win[t][nid].pos);
          dev[t] += static_cast<double>(glm::distance2(npos0, npos1));
        }
      }
      auto res = std::vector<float>(T, 0.f);
      std::transform(dev.cbegin(), dev.cend(), res.begin(), [N = N](const auto& x) { return static_cast<float>(x / N); });
      return res;
    }

    // returns Deviation^2(t) from relative position expected with equal radii
    inline std::vector<float> Dequ(const window_t& win, size_t topo)
    {
      {
        auto r = glm::vec3(0, 0, 0);
        auto rn = glm::vec3(1, 0, 0);
        auto a = glm::vec3(0, 1, 0);
        auto b = glm::normalize(glm::vec3(1, 1, 0));

        auto rna = map(a, r, rn);
        auto rnb = map(b, r, rn);
        auto ar = anti_rotate(a, b, rna);
        int dummy = 0;
      }
      const auto T = win.size();      // time steps
      const auto N = win[0].size();   // individuals
      auto dev = std::vector<double>(T, 0);
      for (auto n = 0; n < N; ++n) {
        const auto nid = win[0][n].ninfo[topo].idx;
        const auto npos0 = map(win[0][n].dir, win[0][n].pos, win[0][nid].pos);
        for (auto t = 1; t < T; ++t) {
          const auto npos1 = map(win[t][n].dir, win[t][n].pos, win[t][nid].pos);
          const auto rpos1 = anti_rotate(win[0][n].dir, win[t][n].dir, npos0);
          dev[t] += static_cast<double>(glm::distance2(npos0, rpos1));
        }
      }
      auto res = std::vector<float>(T, 0.f);
      std::transform(dev.cbegin(), dev.cend(), res.begin(), [N = N](const auto& x) { return static_cast<float>(x / N); });
      return res;
    }

  }


template <typename Tag>
  class DiffusionObserver : public model::AnalysisObserver 
  {
  public:
    DiffusionObserver(const std::filesystem::path& out_path, const json& J) :
      AnalysisObserver(out_path, J),
      max_Qm_topo_(J["max_Qm_topo"]),
      max_D_topo_(J["max_D_topo"]),
      wsize_(model::Simulation::time2tick(double(J["window"]) / this->oi_.sample_freq))
    {}

    ~DiffusionObserver()
    {
      if (future_.valid()) future_.get();
    }

  private:
    void notify_init(const model::Simulation& sim) override
    {
      max_Qm_topo_ = std::min(sim.pop<Tag>().size(), max_Qm_topo_);
      max_D_topo_ = std::min(sim.pop<Tag>().size(), max_D_topo_);
      max_topo_ = std::max(max_D_topo_, max_Qm_topo_);
      Dfor_.resize(max_D_topo_);
      Dequ_.resize(max_D_topo_);
    }

    void notify_pre_collect(const model::Simulation& sim) override
    {
      sim.force_neighbor_info_update(true);
    }

    void notify_collect(const model::Simulation& sim) override
    {
      assert(sim.forced_neighbor_info_update());
      sim.force_neighbor_info_update(false);
      if (future_.valid()) future_.get();
      pull_data(sim);
      if (window_.size() == wsize_) {
        future_ = std::async(std::launch::async, &DiffusionObserver::analyse, this, sim.tick());
      }
    }

    void notify_save(const model::Simulation& sim) override
    {
      if (future_.valid()) future_.get();
      auto fqm = std::async(std::launch::async, &DiffusionObserver::save_Qm, this);
      auto fd = std::async(std::launch::async, &DiffusionObserver::save_R, this);
      auto fde = std::async(std::launch::async, &DiffusionObserver::save_D, this, Dfor_, "Dfor_");
      auto fdp = std::async(std::launch::async, &DiffusionObserver::save_D, this, Dequ_, "Dequ_");
      fqm.get();
      fd.get();
      fde.get();
      fdp.get();
    }

    void pull_data(const model::Simulation& sim)
    {
      const auto& pop = sim.pop<Tag>();
      if (window_.size() == wsize_) {
        // treat deque as ring-buffer
        auto oldest = std::move(window_.front());
        window_.pop_front();
        window_.emplace_back(std::move(oldest));  // ready for re-use
      }
      else {
        auto state = diffusion::snapshot_t(sim.pop<Tag>().size(), { {}, {}, std::vector<model::neighbor_info>(max_topo_) });
        window_.emplace_back(std::move(state));
      }
      auto& state = window_.back();
      for (size_t i = 0; i < pop.size(); ++i) {
        auto& pivot = state[i];
        pivot.pos = pop[i].pos;
        pivot.dir = pop[i].dir;
        auto sv = sim.sorted_view<Tag>(i);
        const auto n = std::min(sv.size(), max_topo_);
        pivot.ninfo.assign(sv.cbegin(), sv.cbegin() + n);
        std::fill(pivot.ninfo.begin() + n, pivot.ninfo.end(), model::neighbor_info{});
      }
    }

    void analyse(model::tick_t tick)
    {
      Qmt_.emplace_back(diffusion::Qm(window_, max_Qm_topo_));
      R_.emplace_back(diffusion::R(window_));
      tbb::parallel_for(0ull, max_D_topo_, [&](size_t i) {
        Dfor_[i].emplace_back(diffusion::Dfor(window_, i));
        Dequ_[i].emplace_back(diffusion::Dequ(window_, i));
      });
    }

    void save_Qm()
    {
      auto op = std::filesystem::path(full_out_path_);
      auto fp = op.parent_path();
      auto fn = fp / "Qm.csv";
      auto os = std::ofstream(fn);
      for (size_t i = 0; i < Qmt_.size(); ++i) {
        os << oi_.sample_freq * model::Simulation::tick2time(model::tick_t{ i });
        for (const auto& x : Qmt_[i]) {
          os << ',' << x;
        }
        os << '\n';
      }
    }

    void save_R()
    {
      auto op = std::filesystem::path(full_out_path_);
      auto fp = op.parent_path();
      auto fn = fp / "R.csv";
      auto os = std::ofstream(fn);
      for (size_t i = 0; i < R_.size(); ++i) {
        os << oi_.sample_freq * model::Simulation::tick2time(model::tick_t{ i });
        for (const auto& x : R_[i]) {
          os << ',' << x;
        }
        os << '\n';
      }
    }

    void save_D(const std::vector<std::vector<std::vector<float>>>& D, const std::string& prefix)
    {
      auto op = std::filesystem::path(full_out_path_);
      auto fp = op.parent_path();
      for (auto topo = 0; topo < max_D_topo_; ++topo) {
        auto fn = fp / (prefix + std::to_string(topo) + ".csv");
        auto os = std::ofstream(fn);
        for (size_t i = 0; i < D[topo].size(); ++i) {
          os << oi_.sample_freq * model::Simulation::tick2time(model::tick_t{ i });
          for (const auto& x : D[topo][i]) {
            os << ',' << x;
          }
          os << '\n';
        }
      }
    }

    diffusion::window_t window_;
    size_t max_topo_ = 0;
    size_t max_Qm_topo_ = 0;
    size_t max_D_topo_ = 0;
    double dt_;
    const size_t wsize_;
    std::future<void> future_;
    std::vector<std::vector<float>> Qmt_;                 // timeseries of Qm(t)
    std::vector<std::vector<float>> R_;                   // timeseries of R(t)
    std::vector<std::vector<std::vector<float>>> Dfor_;   // [0..max_Dev_topo] timeseries of Dfor(t)
    std::vector<std::vector<std::vector<float>>> Dequ_;   // [0..max_Dev_topo] timeseries of Dfor(t)
  };

}


#endif
