#ifndef STARLING_AGENTS_FWD_HPP_INCLUDED
#define STARLING_AGENTS_FWD_HPP_INCLUDED

#include <type_traits>
#include <tuple>
#include <vector>
#include <glm/glm.hpp>


namespace model {

  using tick_t = size_t;
  using glm::vec3;

  // publish our species type(s) to the model core
  class Starling;
  class Pred;


  using starling_tag = std::integral_constant<size_t, 0>;
  using pred_tag = std::integral_constant<size_t, 1>;
  

  using species_pop = std::tuple<
    std::vector<Starling>,
    std::vector<Pred>
  >;


  template <typename Tag>
  struct known_color_maps {
    static constexpr size_t size = 0;
    static constexpr const char** descr = nullptr;
  };

  template <>
  struct known_color_maps<starling_tag>;

  template <>
  struct known_color_maps<pred_tag>;


  template <typename Tag>
  struct snapshot_entry {};
  
  template <>
  struct snapshot_entry<starling_tag>;

  template <>
  struct snapshot_entry<pred_tag>;

  using species_snapshots = std::tuple<
    std::vector<snapshot_entry<starling_tag>>,
    std::vector<snapshot_entry<pred_tag>>
  >;

}

#endif
