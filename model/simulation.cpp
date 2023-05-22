#include <atomic>
#include <tbb/tbb.h>
#include <hrtree/sorting/radix_sort.hpp>
#include <libs/rndutils.hpp>
#include <agents/agents.hpp>
#include <model/simulation.hpp>
#include <model/observer.hpp>


namespace model {


  thread_local rndutils::default_engine reng = rndutils::make_random_engine<>();
 
  float Simulation::dt_;

  namespace {

    using state_array = Simulation::state_array;


    template <size_t S>
    void set_snapshot(Simulation* sim, species_pop& pop, const species_snapshots& s)
    {
      const auto& ss = std::get<S>(s);
      if (!ss.empty()) {
        auto& pops = std::get<S>(pop);
        if (pops.size() != ss.size()) throw std::runtime_error("snapshot mismatch");
        for (size_t i = 0; i < pops.size(); ++i) {
          pops[i].snapshot(sim, i, ss[i]);
        }
      }
      set_snapshot<S + 1>(sim, pop, s);
    }

    template <>
    void set_snapshot<model::n_species>(Simulation* sim, species_pop&, const species_snapshots&)
    {}


    template <size_t S>
    void get_snapshot(const Simulation* sim, const species_pop& pop, species_snapshots& s)
    {
      auto& ss = std::get<S>(s);
      auto& pops = std::get<S>(pop);
      for (size_t i = 0; i < pops.size(); ++i) {
        ss.push_back(pops[i].snapshot(sim, i));
      }
      get_snapshot<S + 1>(sim, pop, s);
    }

    template <>
    void get_snapshot<model::n_species>(const Simulation* sim, const species_pop&, species_snapshots&)
    {}


    template <size_t I>
    struct init_simulation_impl
    {
      static void apply(const json& J, species_pop& pop, state_array& sa, Simulation& sim)
      {
        using agent_type = typename std::tuple_element_t<I, species_pop>::value_type;
        const auto& ji = J[agent_type::name()];
        const size_t N = ji["N"];
        auto& popi = std::get<I>(pop);
        for (size_t i = 0; i < N; ++i) {
          popi.emplace_back(i, ji);
        }
        sa[I].update_times.resize(N);
        auto ut_dist = std::uniform_int_distribution<tick_t>(0, static_cast<tick_t>(1.0 / Simulation::dt()));
        for (auto& ut : sa[I].update_times) {
          ut = ut_dist(reng);
        }
        apply_cross<0>(J, sa);
        init_simulation_impl<I + 1>::apply(J, pop, sa, sim);
        for (size_t i = 0; i < N; ++i) {
          popi[i].initialize(i, sim, ji);
        }
        // initial condition
        species_snapshots ss;
        std::get<I>(ss) = agent_type::init_pop(sim, ji);
        set_snapshot<I>(&sim, pop, ss );
      }

      template <size_t K>
      static void apply_cross(const json& J, state_array& sa)
      {
        using agent_type = typename std::tuple_element_t<K, species_pop>::value_type;
        const auto& jk = J[agent_type::name()];
        const size_t N = jk["N"];
        sa[I].SNI[K].resize(sa[I].size() * N);
        apply_cross<K + 1>(J, sa);
      }

      template <>
      static void apply_cross<n_species>(const json&, state_array&) {}
    };


    template <>
    struct init_simulation_impl<n_species>
    {
      static void apply(const json&, species_pop&, state_array&, const Simulation&) {}
    };


    void init_simulation_state(const json& J, species_pop& pop, state_array& sa, Simulation& sim)
    {
      init_simulation_impl<0>::apply(J, pop, sa, sim);
    }
  

    struct radix_sort_converter
    {
      static const int key_bytes = sizeof(float);
      const std::uint8_t* operator()(const neighbor_info& x) const { return (const std::uint8_t*) & *std::addressof(x); }
    };


    template <size_t I>
    class update_neighbor_info 
    {
    public:
      static void apply(Simulation* sim, size_t idx, state_array& sa)
      {
        apply_<0>(sim, idx, sa);
      }

    private:
      template <size_t J>
      static void apply_(Simulation* sim, size_t idx, state_array& sa)
      {
        using agent_type = typename std::tuple_element_t<I, species_pop>::value_type;
        auto& SNI = sa[I].SNI[J];
        const auto& popi = sim->pop<std::integral_constant<size_t, I>>();
        const auto& popj = sim->pop<std::integral_constant<size_t, J>>();
        const auto& uti = sa[I].update_times;
        const auto& utj = sa[J].update_times;
        auto pos = popi[idx].pos;
        auto dir = popi[idx].dir;
        //auto cur_state = popi[idx].get_current_state();
        //auto is_escaping = false;
        //tick_t time_left = 0;
        //if (!(sim->esc_states_.empty())) {
        //    if (std::find(sim->esc_states_.begin(), sim->esc_states_.end(), cur_state) != sim->esc_states_.end())
        //    {
        //        is_escaping = true;
        //        time_left = popi[idx].state_timer;
        //    }
        //}
        
        auto first = SNI.begin() + (popj.size() * idx);
        auto it = first;
        for (unsigned j = 0; j < popj.size(); ++j, ++it) {
           const auto jstate = popj[j].get_current_state();
          *it = { agent_type::distance2(pos, popj[j].pos), 
              j, 
              agent_type::bearing_angl(dir, pos, popj[j].pos),
              (std::find(sim->esc_states_.begin(), sim->esc_states_.end(), jstate) != sim->esc_states_.end()),
              jstate,
              popj[j].state_timer
          };
        }
        sa[I].RNI[J] = SNI;     // pre-sorted
#ifndef NDEBUG
        // visual studio debug build crawls trough radix sort
        std::sort(first, it, [](const auto& a, const auto& b) { return a.dist2 < b.dist2; });
#else
        hrtree::inplace_radix_sort(first, it, radix_sort_converter{});
#endif
        apply_<J + 1>(sim, idx, sa);
      }

      template <>
      static void apply_<model::n_species>(Simulation*, size_t, state_array&)
      {}
    };


    template <size_t S>
    void update_species(Simulation* sim, species_pop& pop, state_array& sa)
    {
      auto& pops = std::get<S>(pop);
      auto& uts = std::get<S>(sa).update_times;
      const auto T = sim->tick();
      const auto forced_ni_update = sim->forced_neighbor_info_update();
      tbb::parallel_for(tbb::blocked_range<size_t>(0, pops.size()), [&, sim, T](auto r) {
        for (size_t i = r.begin(); i < r.end(); ++i) {
          const auto update = uts[i] <= T;
          if (update || forced_ni_update) update_neighbor_info<S>::apply(sim, i, sa);
          if (update) uts[i] = pops[i].update(i, T, *sim);
        }
      });
      update_species<S + 1>(sim, pop, sa);
    }

    template <>
    void update_species<model::n_species>(Simulation*, species_pop&, state_array&)
    {}


    template <size_t S>
    void integrate_species(Simulation* sim, species_pop& pop, state_array& sa)
    {
      auto& pops = std::get<S>(pop);
      auto& uts = std::get<S>(sa).update_times;
      const auto T = sim->tick();
      tbb::parallel_for(tbb::blocked_range<size_t>(0, pops.size()), [&, sim, T](auto r) {
        for (size_t i = r.begin(); i < r.end(); ++i) {
          if (uts[i] != static_cast<tick_t>(-1)) {
            pops[i].integrate(T, *sim);
          }
        }
      });
      integrate_species<S + 1>(sim, pop, sa);
      std::get<S>(sa).flock_tracker.track();
    }


    template <>
    void integrate_species<model::n_species>(Simulation*, species_pop&, state_array&)
    {}

    template <size_t S>
    void integrate_species_flock(Simulation* sim, species_pop& pop, state_array& sa, float fdd)
    {
      auto& pops = std::get<S>(pop);
      auto& uts = std::get<S>(sa).update_times;
      auto& fts = std::get<S>(sa).flock_tracker;
      fts.prepare(pops.size());
      const auto T = sim->tick();
      tbb::parallel_for(tbb::blocked_range<size_t>(0, pops.size()), [&, sim, T](auto r) {
        for (size_t i = r.begin(); i < r.end(); ++i) {
          if (uts[i] != static_cast<tick_t>(-1)) {
            pops[i].integrate(T, *sim);
            fts.feed(pops[i], i);
          }
        }
      });
      integrate_species_flock<S + 1>(sim, pop, sa, fdd);
      fts.cluster(fdd);
    }

    template <>
    void integrate_species_flock<model::n_species>(Simulation*, species_pop&, state_array&, float)
    {}

  }
  

  void notify_observer(Observer* observer, Simulation::Msg msg, Simulation* self)
  {
    if (observer) {
      observer->notify(msg, *self);
    }
  }


  Simulation::Simulation(const json& J) :
    tick_(0)
  {
    dt_ = J["Simulation"]["dt"];
    float flock_threshold = J["Simulation"]["flockDetection"]["threshold"];
    flock_dd_ = flock_threshold * flock_threshold;
    flock_update_ = 0;
    flock_interval_ = time2tick(J["Simulation"]["flockDetection"]["interval"]);
    const std::vector<int> esc_states = J["Simulation"]["esc_states"];
    std::for_each(esc_states.begin(), esc_states.end(), [&](const auto& st) { esc_states_.push_back(st); });

    init_simulation_state(J, species_, state_, *this);
  }


  Simulation::~Simulation()
  {
  }

  
  void Simulation::initialize(Observer* observer, const species_snapshots& ss)
  {
    set_snapshots(ss);
    notify_observer(observer, Simulation::Initialized, this);
  }


  void Simulation::update(Observer* observer)
  {
    notify_observer(observer, PreTick, this);
    {
      std::lock_guard<std::recursive_mutex> _(mutex_);
      update_species<0>(this, species_, state_);
      if (flock_update_ == tick_) {
        integrate_species_flock<0>(this, species_, state_, flock_dd_);
        flock_update_ += flock_interval_;
      }
      else {
        integrate_species<0>(this, species_, state_);
      }
      ++tick_;
    }
    notify_observer(observer, Tick, this);
  }


  void Simulation::set_snapshots(const species_snapshots& ss)
  {
    std::lock_guard<std::recursive_mutex> _(mutex_);
    set_snapshot<0>(this, species_, ss);
  }


  species_snapshots Simulation::get_snapshots() const
  {
    std::lock_guard<std::recursive_mutex> _(mutex_);
    species_snapshots res;
    get_snapshot<0>(this, species_, res);
    return res;
  }

}
