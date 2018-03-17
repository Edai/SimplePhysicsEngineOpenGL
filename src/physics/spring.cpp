#include "math/math.hpp"
#include "math/vector.hpp"
#include "physics/spring.hpp"

namespace _462 {

    Spring::Spring() {
        body1_offset = Vector3::Zero;
        body2_offset = Vector3::Zero;
        damping = 0.0;
    }

    void Spring::step(real_t dt) {
        // TODO apply forces to attached bodies
    }

}


