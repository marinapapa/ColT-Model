#ifndef MODEL_SIMULATION_HPP_INCLUDED
#define MODEL_SIMULATION_HPP_INCLUDED

#include <mutex>
#include <atomic>
#include <model/json.hpp>
#include <model/flock.hpp>


namespace model {

  class Simulation
  {
  private:
    static float dt_;

  public:
    enum Msg {
      Tick = 0,
      PreTick,
      Initialized,
      Finished,
      MaxMsg
    };

  public:
    explicit Simulation(const json& J);
    ~Simulation();
    void initialize(class Observer* observer, const species_snapshots& ss);
    void set_snapshots(const species_snapshots& ss);
    species_snapshots get_snapshots() const;

    // request forced neighbor info update every tick
    void force_neighbor_info_update(bool required) const { force_ni_update_.fetch_add(required ? +1 : -1); }
    bool forced_neighbor_info_update() const { return force_ni_update_.load(std::memory_order_acquire) > 0; }

    void update(class Observer* observer);
    
    static float dt() noexcept { return dt_; }      // [s]

    tick_t tick() const noexcept { return tick_; }  // [1]
    static tick_t time2tick(double time) noexcept { return static_cast<tick_t>(time / dt_); }  // [1]
    double time() const noexcept { return static_cast<double>(dt_) * tick_; }                 // [s]
    static double tick2time(tick_t tick) noexcept { return static_cast<double>(dt_) * tick; } // [s]

    // returns const reference to population vector that
    template <typename Tag>
    const auto& pop() const noexcept 
    {
      return std::get<Tag::value>(species_);
    }

    // returns exclusive neighborhood sorted by distance
    template <typename Tag, typename OtherTag = Tag>
    neighbor_info_view sorted_view(size_t idx) const noexcept
    {
      return sorted_view_impl<Tag::value, OtherTag::value>(idx);
    }

    // returns complete neighborhood
    template <typename Tag, typename OtherTag = Tag>
    neighbor_info_view raw_view(size_t idx) const noexcept
    {
      return raw_view_impl<Tag::value, OtherTag::value>(idx);
    }

    template <typename Tag>
    const std::vector<flock_descr>& flocks() const noexcept
    {
      return std::get<Tag::value>(state_).flock_tracker.flocks();
    }

    template <typename Tag>
    flock_descr flock_info(size_t flock_id) const
    {
      return std::get<Tag::value>(state_).flock_tracker.descr(static_cast<int>(flock_id));
    }

    template <typename Tag>
    int flock_of(size_t idx) const
    {
      return std::get<Tag::value>(state_).flock_tracker.id_of(idx);
    }

    template <typename Tag>
    std::vector<int> flock_mates(size_t flock_id) const
    {
      auto fm = std::vector<int>{};
      for (size_t i = 0; i < state_[Tag::value].flock_tracker.pop_size(); ++i) {
        if (flock_of<Tag>(i) == flock_id) {
          fm.push_back(static_cast<int>(i));
        }
      }
      return fm;
    }

    // Access from foreign threads
    
    void terminate() const noexcept { terminate_.store(true, std::memory_order_release); }
    bool terminated() const noexcept { return terminate_.load(std::memory_order_acquire); }

    // calls fun for all individuals, internally synchronized
    template <typename Tag, typename Fun>
    size_t visit_all(Fun&& fun) const
    {
      std::lock_guard<std::recursive_mutex> _(mutex_);
      auto& pop = std::get<Tag::value>(species_);
      size_t n = 0;
      for (size_t i = 0; i < pop.size(); ++i) {
        fun(pop[i], i); ++n;
      }
      return n;
    }

    // calls fun for all individuals, internally synchronized
    template <typename Tag, typename Fun>
    size_t visit(Fun&& fun) const
    {
      std::lock_guard<std::recursive_mutex> _(mutex_);
      auto& pop = std::get<Tag::value>(species_);
      size_t n = 0;
      for (size_t i = 0; i < pop.size(); ++i) {
        if (std::get<Tag::value>(state_).update_times[i] != static_cast<tick_t>(-1)) {
          fun(pop[i]); ++n;
        }
      }
      return n;
    }

    // calls fun for individual idx, internally synchronized
    template <typename Tag, typename Fun>
    size_t visit(size_t idx, Fun&& fun) const
    {
      std::lock_guard<std::recursive_mutex> _(mutex_);
      auto& pop = std::get<Tag::value>(species_);
      assert(idx < pop.size());
      size_t n = 0;
      fun(pop[idx]); ++n;
      return n;
    }

  private:
    // returns exclusive neighborhood sorted by distance
    template <size_t S1, size_t S2>
    neighbor_info_view sorted_view_impl(size_t idx) const noexcept
    {
      const auto n = state_[S2].update_times.size();
      const auto first = &state_[S1].SNI[S2][idx * n];
      if constexpr (S1 == S2) {
        return neighbor_info_view{ first + 1, n - 1 };    // omit 'self' 
      }
      else {
        return neighbor_info_view{ first, n };
      }
    }

    // returns complete neighborhood
    template <size_t S1, size_t S2>
    neighbor_info_view raw_view_impl(size_t idx) const noexcept
    {
      const auto n = state_[S2].update_times.size();
      const auto first = &state_[S1].RNI[S2][idx * n];
      return neighbor_info_view{ first, n };
    }

  private:
    tick_t tick_ = 0;
    tick_t flock_update_ = 0;
    tick_t flock_interval_ = 0;
    float flock_dd_ = 0.f;


    mutable std::atomic<int> force_ni_update_ = 0;       // forced neighbor info update every tick if > 0
    mutable std::recursive_mutex mutex_;                 // simulation lock
    mutable species_pop species_;
    mutable std::atomic<bool> terminate_ = false;

    struct state_t
    {
      size_t size()const noexcept { return update_times.size(); }
      std::vector<tick_t> update_times;
      std::vector<float> stress;
      std::array<std::vector<neighbor_info>, n_species> SNI;   // sorted neighbor info matrices
      std::array<std::vector<neighbor_info>, n_species> RNI;   // raw neighbor info matrices
      flock_tracker flock_tracker;
    };
    mutable std::array<state_t, n_species> state_;
    friend class flock_tracker;

   public:
     using state_array = decltype(state_);
     std::vector<int> esc_states_; // which states are escapes and copyable

   };

}

#endif
