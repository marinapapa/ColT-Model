#ifndef MODEL_STRESS_BASE_HPP_INCLUDED
#define MODEL_STRESS_BASE_HPP_INCLUDED

#include <algorithm>
#include <model/action_base.hpp>    // reuse actions::package


namespace model {
  namespace stress {

    /*
     *  a stress-source shall be modeled along:
     *
     *    template <typename Agent>
     *    class source
     *    {
     *    public:
     *      static constexpr const char* name();
     *      using agent_type = Agent;
     *      using Tag = typename Agent::Tag;
     *      source() = default;
     *      source(size_t idx, const json& J);
     *      void operator(agent_type* self, size_t idx, tick_t T, const Simulation& sim);
     *    };
     *
     */

    // stress accumulator over several sources
    template <typename Agent, typename ... Sources>
    class accumulator : public ::model::actions::package<Agent, Sources...>
    {
    public:
      using typename ::model::actions::package<Agent, Sources...>::package_tuple;

      static void apply(package_tuple& sources, Agent* self, size_t idx, tick_t T, const Simulation& sim)
      {
        apply_<0>(sources, self, idx, T, sim);
      }

    private:
      template <size_t I>
      static void apply_(package_tuple& sources, Agent* self, size_t idx, tick_t T, const Simulation& sim)
      {
        std::get<I>(sources)(self, idx, T, sim);
        apply_<I + 1>(sources, self, idx, T, sim);
      }

      template <>
      static void apply_<std::tuple_size_v<package_tuple>>(package_tuple&, Agent*, size_t, tick_t T, const Simulation& sim)
      {}

    };


#define make_stress_source_from_this(s) make_action_from_this(s)

  }
}
#endif
