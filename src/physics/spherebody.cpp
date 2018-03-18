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

    Vector3 SphereBody::calculateRK4(real_t dt, Vector3 v)
    {
        Vector3 k1, k2, k3, k4;

        k1 = dt * v;
        k2 = dt * ((v + k1)/2.0f);
        k3 = dt * ((v + k2)/2.0f);
        k4 = dt * (v + k3);
        return (k1 + 2.0f * k2 + 2.0f * k3  + k4) / 6.0f * dt;
    }

    Vector3 SphereBody::step_position(real_t dt, real_t motion_damping)
    {
        position += calculateRK4(dt, velocity);
        velocity += force / mass * dt;
        position += (velocity * dt);
        sphere->position = position;
        return (position);
    }

    Vector3 SphereBody::step_orientation(real_t dt, real_t motion_damping)
    {
        velocity += calculateRK4(dt, force / mass);
        Quaternion q = Quaternion(angular_velocity / length(angular_velocity), dt * length(angular_velocity));
        orientation = orientation * q;
        sphere->orientation = orientation;
        return (position);
    }

    void SphereBody::apply_force(const Vector3 &f, const Vector3 &offset)
    {
        torque = cross(f, offset);
        if (offset == Vector3::Zero || torque == Vector3::Zero)
        {
            force += f;
            return;
        }
        force += (dot(f, offset) / squared_length(offset)) * offset;
        return;
    }
}
