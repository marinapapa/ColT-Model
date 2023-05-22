#ifndef CIRCUMCENTER_HPP_INCLUDED
#define CIRCUMCENTER_HPP_INCLUDED


#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>


namespace circum {


  template <typename T>
  struct circumcenter2d_t
  {
    glm::tvec2<T> k = glm::tvec2<T>(0);     // curvature vector
    T R = 0;                                // radius
    glm::tvec2<T> M = glm::tvec2<T>(0);     // center
  };


  template <typename T>
  struct circumcenter3d_t
  {
    glm::tvec3<T> k = glm::tvec3<T>(0);     // curvature vector
    T R = 0;                                // radius
    glm::tvec3<T> M = glm::tvec3<T>(0);     // center
  };


  // returns circumcenter_t for A
  template <typename T>
  circumcenter3d_t<T> circumcenter(
    const glm::tvec3<T>& A,   // r(t)
    const glm::tvec3<T>& B,   // r(t-1)
    const glm::tvec3<T>& C)   // r(t+1)
  {
    constexpr auto eps = T(10e-16);
    const auto AB = B - A;
    const auto AC = C - A;
    const auto D = glm::cross(AB, AC);
    const auto dd = glm::length2(D);
    if (dd < eps) {
      return {};   // colinear
    }
    const auto bb = glm::length2(AC);
    const auto cc = glm::length2(AB);
    const auto E = glm::cross(D, AB);
    const auto F = glm::cross(D, AC);
    const auto G = T(0.5) * (bb * E - cc * F) / dd;
    const auto RR = glm::length2(G);
    return { G / RR, std::sqrt(RR), A + G };
  }


  // returns circumcenter_t for A
  template <typename T>
  circumcenter2d_t<T> circumcenter(
    const glm::tvec2<T>& A,   // r(t)
    const glm::tvec2<T>& B,   // r(t-1)
    const glm::tvec2<T>& C)   // t(t+1)
  {
    auto c3d = circumcenter(glm::tvec3<T>(A, 0), glm::tvec3<T>(B, 0), glm::tvec3<T>(C, 0));
    return { glm::tvec2<T>(c3d.k), c3d.R, c3d.M };
  }


}

#endif
