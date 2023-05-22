#ifndef INIT_CONDIT_HPP_INCLUDED
#define INIT_CONDIT_HPP_INCLUDED

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>
#include <libs/math.hpp>
#include <glmutils/random.hpp>
#include <model/simulation.hpp>


namespace initial_conditions {

  // config key: defined
  class defined_pos_dir
  { 
  public:
	defined_pos_dir(const json& J) :
    speed_(J["speed"]),
      pos0_(J["pos"][0], J["pos"][1], J["pos"][2]),
      dir0_(J["dir"][0], J["dir"][1], J["dir"][2]),
      radius_(J["radius"]),
      raddev_(glm::radians<float>(J["degdev"]))
    {}

    template <typename Entry>
    void operator()(Entry& entry)
    {
      auto uni = std::uniform_real_distribution<float>(0.f, 1.f);
      entry.pos = pos0_ + radius_ * model::vec3(uni(model::reng), uni(model::reng), uni(model::reng));
      const auto a = std::normal_distribution<float>(0, raddev_)(model::reng);
      const auto c = std::cos(a);
      const auto s = std::sin(a);
      const auto Rz = glm::mat3(
        c, s, 0,
        -s, c, 0,
        0, 0, 1
      );
      entry.dir = Rz * dir0_;
      entry.speed = speed_;
    }

  private:
    model::vec3 pos0_;
    model::vec3 dir0_;
    float speed_;
    float radius_;
    float raddev_;
  };

  // config key: random
  class random_pos_dir
  {
  public:
	  random_pos_dir(const json& J) :
      radius_(J["radius"])
    {}

	  template <typename Entry>
	  void operator()(Entry& entry)
	  {
		  auto pdist = std::uniform_real_distribution<float>(0.f, radius_);
		  entry.pos = model::vec3(pdist(model::reng), pdist(model::reng), pdist(model::reng));
		  entry.dir = model::vec3(glmutils::unit_vec2(model::reng), 0.f);
	  }

  private:
    float radius_;
  };

  // config key: random_dead
  class random_dead
  {
  public:
    random_dead(const json& J) :
      radius_(J["radius"])
    {}

    template <typename Entry>
    void operator()(Entry& entry)
    {
      auto pdist = std::uniform_real_distribution<float>(0.f, radius_);
      entry.pos = model::vec3(pdist(model::reng), pdist(model::reng), pdist(model::reng));
      entry.dir = glm::vec3(glmutils::unit_vec2(model::reng), 0.f);
    }

  private:
    float radius_;
  };

  // config key: csv
  class from_csv
  { 
  public:
	 from_csv(const json& J) :
		csv_(std::filesystem::path(std::string(J["file"])))
		{
			csv_.ignore(2048, '\n');    // skip header
		}

    template <typename Entry>
    void operator()(Entry& entry)
    {
	  // function reads only one line
      Entry::stream_from_csv(csv_, entry);
	  // delete line that has been read already
	    csv_.ignore(2048, '\n'); 
    }

  private:
    std::ifstream csv_;
  };


  // config key: flock
  class in_flock
  {
  public:
	  in_flock(const json& J) :
      speed_(J["speed"]),
		  dir0_(J["dir"][0], J["dir"][1], J["dir"][2]),
		  radius_(J["radius"]),
		  raddev_(glm::radians<float>(J["degdev"]))
	  {}

	  template <typename Entry>
	  void operator()(Entry& entry)
	  {
		  auto uni = std::uniform_real_distribution<float>(0.f, 1.f);
		  entry.pos = radius_ * model::vec3(uni(model::reng), uni(model::reng), 0.f);
      const auto a = std::normal_distribution<float>(0, raddev_)(model::reng);
      const auto c = std::cos(a);
      const auto s = std::sin(a);
      const auto Rz = glm::mat3(
        c, s, 0,
        -s, c, 0,
        0, 0, 1
      );
      entry.dir = Rz * dir0_;
		  entry.speed = speed_;
	  }

  private:
	  model::vec3 dir0_;
	  float speed_;
	  float radius_;
	  float raddev_;
  };

    //template <typename Agent>
    //void deviate_speed(Agent* self)
    //{  // individual variation in speeds
    //  auto spdist = std::normal_distribution<float>(0,4);
    //  self->cs_dev_ = spdist(model::reng); // cruise speed deviation
    //}
}
#endif
