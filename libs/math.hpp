#ifndef MATH_UTILS_HPP_INCLUDED
#define MATH_UTILS_HPP_INCLUDED

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glmutils/perp_dot.hpp>
#include <libs/rndutils.hpp>


namespace math {


  template <typename T>
  inline T constexpr pi = T(3.1415926535897932384626433832795);

  // statistics

  template <typename T>
  constexpr T normalize_min_max(const T& a, const T& min, const T& max) noexcept
  {
    return (a - min) / (max - min);
  }

  template <typename T>
  constexpr std::vector<T> normalize_vector_min_max(const std::vector<T>& a) 
  {
    auto minp = std::min(a);
    auto maxp = std::max(a);
    auto retv = a;

    std::for_each(retv.begin(), retv.end(), [](T& el) { math::normalize_min_max(el, minp, maxp); });
    std::for_each(retv.begin(), retv.end(), [](T& el) { assert(el <= 1); });

    return retv;
  }

  template <typename T>
  constexpr T normalize_angle(const T& angle)
  {
    T newAngle = angle;
    while (newAngle <= -math::pi<T>) newAngle += T(2) * math::pi<T>;
    while (newAngle > math::pi<T>) newAngle -= T(2) * math::pi<T>;
    return newAngle;
  }

  //template <typename T, typename X, size_t N>
  //constexpr void normalize_array(T (&a)[N])
  //{
  //  auto minp = std::min(a);
  //  auto maxp = std::max(a);

  //  std::for_each(a.size(), a.capacity(), [](X& el) { normalize(el, minp, maxp); });
  //  std::for_each(a.size(), a.capacity(), [](X& el) { assert(el <= 1); });
  //}



  template <typename T>
  constexpr T min_difference(const T& a, const T& b) noexcept
  {
    return std::min(a - b, b - a);
  }

  template <typename T>
  constexpr T min_rad_difference_unsigned(const T& b, const T& a) noexcept
  {
    const auto c = abs(b - a); 
    return std::min(T(2) * math::pi<T> - c, c);
  }

  template <typename T>
  constexpr T min_rad_difference(const T& b, const T& a) noexcept
  {
    const auto c = b - a;
    return std::copysign(std::min(T(2) * math::pi<T> - abs(c), abs(c)), c); // Signed unsighed ?
  }

  template <typename T>
  constexpr T wrap_rad(const T& a) noexcept
  {
    const auto b = glmutils::mod(a, T(2) * math::pi<T>);
    return ( b < 0 ) ? b + T(2) * math::pi<T> : b ;
  }



  // Geometry
  
  // returns angle [rad] between a and b clamped to [-rad(maxDeg), +rad(maxDeg)].
  template <typename T, glm::precision P, template <typename, glm::precision> class vecType>
  constexpr T rad_between_xy(const vecType<T, P>& a, const vecType<T, P>& b, T maxDeg = T(180)) noexcept
  {
    auto c = glmutils::perpDot(a, b);
    auto d = glm::dot(a, b);
    auto maxRad = glm::radians(maxDeg);
    return glm::clamp(std::atan2(c, d), -maxRad, +maxRad);
  }


  // returns angle [rad] between a and b clamped to [-rad(maxDeg), +rad(maxDeg)].
  template <typename T, glm::precision P, template <typename, glm::precision> class vecType>
  constexpr T rad_between_xy_max_rad(const vecType<T, P>& a, const vecType<T, P>& b, T maxRad = pi<T>) noexcept
  {
    auto c = glmutils::perpDot(a, b);
    auto d = glm::dot(a, b);
    return glm::clamp(std::atan2(c, d), -maxRad, +maxRad);
  }


  template <typename T, glm::precision P, template <typename, glm::precision> class vecType>
  constexpr decltype(auto) rotate_xy(const vecType<T,P>& a, T rad) noexcept
  {
    const auto c = std::cos(rad);
    const auto s = std::sin(rad);
    return vecType<T,P>(a.x * c - a.y * s, a.x * s + a.y * c, T(0));
  }


  template <typename T, glm::precision P, template <typename, glm::precision> class vecType>
  constexpr decltype(auto) save_normalize(const vecType<T,P>& a, const vecType<T,P>& fallBack) noexcept
  {
    auto len2 = glm::dot(a, a);   // length2
    return (len2 > T(0.0000001)) ? a / std::sqrt(len2) : fallBack;  // covers NaN -> fallBack
  }

  template <typename T, glm::precision P, template <typename, glm::precision> class vecType>
  constexpr decltype(auto) slerp(const vecType<T, P>& a, const vecType<T, P>& b, T mix) noexcept
  {
    const auto theta = math::rad_between_xy(a, b);
    return math::rotate_xy(a, mix * theta);
  }

  template <typename T, glm::precision P, template <typename, glm::precision> class vecType>
  constexpr decltype(auto) slerp_max_rad(const vecType<T, P>& a, const vecType<T, P>& b, T mix, T maxRad = pi<T>) noexcept
  {
    const auto theta = math::rad_between_xy_max_rad(a, b, maxRad);
    return math::rotate_xy(a, mix * theta);
  }


  // Interpolation functions

  template <typename T>
  constexpr T fade(T x) noexcept
  {
    return x * x * x * (x * (x * T(6) - T(15)) + T(10));
  }

  template <typename T>
  constexpr T smootherstep(T x, const T edge0, const T edge1) noexcept
  {
    // Scale, bias and saturate x to 0..1 range
    x = std::clamp((x - edge0) / (edge1 - edge0), T(0), T(1));
    return math::fade(x);
  }

  template <typename T>
  constexpr T smoothstep(T x, const T edge0, const T edge1) noexcept
  {
    // Scale, bias and saturate x to 0..1 range
    x = std::clamp((x - edge0) / (edge1 - edge0), T(0), T(1));

    return x * x * (T(3) - T(2) * x);
  }

}

#endif
