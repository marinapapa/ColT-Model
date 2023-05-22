#ifndef SPACE_HPP_INCLUDED
#define SPACE_HPP_INCLUDED

#include <cassert>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glmutils/perp_dot.hpp>
#include <libs/rndutils.hpp>
#include <libs/math.hpp>

namespace space {

  // a + ofs == b
  template <typename T>
  constexpr T ofs(const T& a, const T& b) noexcept
  {
    return b - a;
  }


  template <typename T> // if individual b is behind from individual a
  constexpr bool is_behind(const T& a_p, const T& a_h, const T& b_p) noexcept
  {
    return glm::dot(a_h, ofs(a_p, b_p)) < T(0);
  }


  template <typename T, typename A> // if individual b is behind from individual a
  constexpr bool is_atside(const T& a_p, const T& a_h, const T& b_p, const A side_angle) noexcept
  {
    const auto cFrontFov = std::cos(glm::radians(180.0f - 0.5f * (360.0f - side_angle))); // in-front field of view
    const auto ddist = std::sqrt(glm::distance2(a_p, b_p));

    return glm::dot(a_h, ofs(a_p, b_p)) > ddist* cFrontFov ? false : true;
  }


  template <typename T>
  constexpr bool is_wrapped(T WH, T x) noexcept
  {
    return (x >= T(0)) && (x <= WH);
  }


  template <typename T>
  constexpr bool is_wrapped(T WH, const glm::tvec3<T>& pos) noexcept
  {
    return is_wrapped(WH, pos.x) && is_wrapped(WH, pos.y) && is_wrapped(WH, pos.z);
  }


}

#endif
