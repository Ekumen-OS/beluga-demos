// Copyright 2025 Ekumen, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MH_AMCL_PARTICLE_TRAITS_HPP
#define MH_AMCL_PARTICLE_TRAITS_HPP

#include <beluga/type_traits/particle_traits.hpp>

#include "beluga_demo_mh_amcl/particles_distribution.hpp"

namespace beluga {

// It is necessary to speicify how to use Beluga's particle traits with my new
// defined type of Particle, to be able to obtain the state and weight of the particles
template <> struct particle_traits<mh_amcl::Particle> {
  // Define the types for the particle state and weight:
  using state_type = Sophus::SE2d;
  using weight_type = beluga::Weight;

  static inline const state_type &state(const mh_amcl::Particle &p) {
    return p.state;
  }

  static inline const weight_type &weight(const mh_amcl::Particle &p) {
    return p.weight;
  }
};

// Specialize for const Particle as well
template <>
struct particle_traits<const mh_amcl::Particle>
    : particle_traits<mh_amcl::Particle> {};

} // namespace beluga

#endif // MH_AMCL_PARTICLE_TRAITS_HPP
