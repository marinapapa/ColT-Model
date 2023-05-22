#ifndef MODEL_FLOCK_HPP_INCLUDED
#define MODEL_FLOCK_HPP_INCLUDED

#include <vector>
#include <model/model.hpp>


namespace model {


  struct flock_descr
  {
    size_t size = 0;
    vec3 vel = vec3(0);  // velocity
    float pol = 0.f;     // polarization
    glm::mat3x3 H;         // homogeneous transformation matrix flock -> Euclidean
	  vec3 ext;

    vec3 gc() const { return vec3(H[2]); }
  };

  constexpr unsigned no_flock = static_cast<unsigned>(-1);


  class flock_tracker
  {
  public:
    flock_tracker() {}

    size_t pop_size() const 
    { 
      return flock_id_.size(); 
    }
    
    const std::vector<flock_descr>& flocks() const noexcept
    {
      return descr_;
    }

    flock_descr descr(int id) const noexcept
    {
      return (static_cast<size_t>(id) < descr_.size()) ? descr_[id] : flock_descr{};
    }

    int id_of(size_t idx) const noexcept
    {
      return flock_id_[idx];
    }

    void prepare(size_t n)
    {
      proxy_.assign(n, proxy{});
    }

    template <typename T>
    void feed(const T& ind, size_t idx)
    {
      proxy_[idx] = proxy(ind, idx);
    }

    void cluster(float dd);
    void track();

  private:
    struct proxy 
    { 
      proxy() : idx(static_cast<unsigned>(-1)) {}

      template <typename T>
      proxy(const T& ind, size_t idx) :
        idx(static_cast<unsigned>(idx)),
        pos(ind.pos),
        vel(ind.speed* ind.dir)
      {}

      unsigned idx; vec3 pos, vel;
    };
    std::vector<proxy> proxy_;
    std::vector<flock_descr> descr_;
    std::vector<vec3> vpos_;
    std::vector<vec3> vvel_;
    std::vector<unsigned> flock_id_;
  };

}

#endif
