#include "math/math.hpp"
#include "math/vector.hpp"
#include "physics/spring.hpp"

namespace _462 {

    Spring::Spring() {
        body1_offset = Vector3::Zero;
        body2_offset = Vector3::Zero;
        damping = 0.0;
    }

    void Spring::step(real_t dt)
    {
        Vector3 realb1 = body1->position + body1_offset;
        Vector3 realb2 = body2->position + body2_offset;

        Vector3 direction = (realb1 - realb2) / length(realb1 - realb2);
        Vector3 dxdt = direction * dot(this->body1->velocity, direction);
        Vector3 x = direction * (length(realb1 - realb2) - this->equilibrium);
        Vector3 f = -constant * x  - damping *  dxdt;

        body1->apply_force(f, Vector3::Zero);
     }

}


