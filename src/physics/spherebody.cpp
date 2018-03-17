#include "physics/spherebody.hpp"
#include "math/matrix.hpp"

namespace _462 {

    SphereBody::SphereBody(Sphere *geom)
    {
        sphere = geom;
        position = sphere->position;
        radius = sphere->radius;
        orientation = sphere->orientation;
        mass = 0.0;
        velocity = Vector3::Zero;
        angular_velocity = Vector3::Zero;
        force = Vector3::Zero;
        torque = Vector3::Zero;
    }

    Vector3 SphereBody::step_position(real_t dt, real_t motion_damping)
    {
        velocity += force / mass * dt;
        position += (velocity * dt);
        sphere->position = position;
        return (position);
    }

    Vector3 SphereBody::step_orientation(real_t dt, real_t motion_damping)
    {
        Quaternion q = Quaternion(angular_velocity / length(angular_velocity), dt * length(angular_velocity));
        orientation = normalize(orientation * q);
        sphere->orientation = orientation;
        return (position);
    }

    void SphereBody::apply_force(const Vector3 &f, const Vector3 &offset)
    {
        if (offset == Vector3::Zero || cross(f, offset) == Vector3::Zero)
        {
            force += f;
            return;
        }
        force += (dot(f, offset) / squared_length(offset)) * offset;
        auto i = 2.0f / 5.0f * mass * radius * radius;
        angular_velocity =  force / i;
        return;
    }
}
