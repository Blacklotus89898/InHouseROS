#include "robotics_algo/common/types.hpp"
#include <cmath>

namespace robotics::math {

    Scalar normalize_angle(Scalar angle) {
        // Use standard formula to wrap angle between -PI and PI
        angle = std::fmod(angle + PI, 2.0 * PI);
        if (angle < 0.0) {
            angle += 2.0 * PI;
        }
        return angle - PI;
    }

}
