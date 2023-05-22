#ifndef MODEL_FLIGHT_ABC_HPP_INCLUDED
#define MODEL_FLIGHT_ABC_HPP_INCLUDED

#include <libs/math.hpp>
#include <model/json.hpp>


namespace model {
  namespace flight {

    // returns cruise speed based on wingload.
      // Alerstam et al PLOS Biol 5, 2007
    template <typename T>
    inline T cruise_speed(T bodyMass, float wingArea)
    {
      const auto wingLoad = bodyMass * T(9.81) / wingArea;
      return T(4.8) * std::pow(wingLoad, T(0.28));
    }


    // returns lift coefficient CL base on wing aspect ratio
    template <typename T>
    inline T CL(T wingAspectRatio, T alpha = T(1))
    {
      const auto piAR = math::pi<T> * wingAspectRatio;
      const auto Cl = T(2) * math::pi<T> * alpha /
        (T(1) + T(2) * wingAspectRatio + T(16) * std::log(piAR) - T(9) / T(8)) /
        (piAR * piAR);
      return Cl;
    }


    template <typename T>
    struct aero_info
    {
      T bodyMass;
      T cruiseSpeed;
      T cruiseSpeedSd;
      T minSpeed;
      T maxSpeed;
      T CL;
      T aspectRatio;
      T wingArea;
      T maxSteerF;
      T w;
    };


    template <typename T>
    inline aero_info<T> create_aero_info(const json& J)
    {
      aero_info<T> ai;

      // body mass deviation
      ai.bodyMass = J["bodyMass"];
      auto sds = J["bodyMassSd"];
      if (sds != 0)
      {
        auto spdist = std::uniform_real_distribution<T>(0, sds);
        ai.bodyMass += spdist(model::reng);
      }

      ai.cruiseSpeedSd = J["cruiseSpeedSd"];
      if (ai.cruiseSpeedSd != 0)
      {
        auto spdist = std::uniform_real_distribution<T>(0, ai.cruiseSpeedSd);
        ai.cruiseSpeedSd = spdist(model::reng);
      }

      ai.aspectRatio = J["wingAspectRatio"];
      ai.wingArea = J["wingArea"];
      auto jit = J.find("cruiseSpeed");
      ai.cruiseSpeed = (jit == J.end()) ? cruise_speed(ai.bodyMass, ai.wingArea) : T(*jit);
      ai.cruiseSpeed += ai.cruiseSpeedSd;

      jit = J.find("CL");
      ai.CL = (jit == J.end()) ? CL(ai.aspectRatio) : T(*jit);
      ai.minSpeed = J["minSpeed"];
      ai.maxSpeed = J["maxSpeed"];
      ai.maxSteerF = J["maxSteerF"];
      auto wit = J.find("w");
      ai.w = (wit == J.end()) ? 1.f : T(*jit);
      return ai;
    }

    template <typename T>
    struct state_aero
    {
      T cruiseSpeed;
      T w; // weight to return to cruise speed
    };

    template <typename T>
    inline state_aero<T> create_state_aero(const json& J)
    {
      state_aero<T> sa;
      sa.cruiseSpeed = J["cruiseSpeed"];
      sa.w = J["w"];
      return sa;
    }
  }
}

#endif


