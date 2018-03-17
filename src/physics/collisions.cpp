
#include "physics/collisions.hpp"

namespace _462 {

    //Formula from p5.pdf
    bool collides(SphereBody &body1, SphereBody &body2, real_t collision_damping)
    {
        real_t dist_spheres = length(body1.position - body2.position);

        if (fabs(dist_spheres) < (body1.radius + body2.radius))
        {
            Vector3 vp_1 = body1.velocity - body2.velocity;
            Vector3 d = (body2.position - body1.position) / length(body2.position - body1.position);
            Vector3 vpp_2 = 2 * d * (body1.mass / (body1.mass + body2.mass)) * dot(vp_1, d);

            Vector3 tmp = body2.velocity;
            body2.velocity = body2.velocity + vpp_2;
            body1.velocity = (body1.mass * body1.velocity + body2.mass * tmp - body2.mass * body2.velocity) / (body1.mass);
            body1.velocity -= collision_damping * body1.velocity;
            body2.velocity -= collision_damping * body2.velocity;
            return (true);
        }
        return(false);
    }

    //Formula from Ray-triangle intersection for WASHINGTON UNIVERSITY BY Brian Curless
    bool collides(SphereBody &body1, TriangleBody &body2, real_t collision_damping)
    {
        Vector3 n = normalize(cross(body2.vertices[1] - body2.vertices[0], body2.vertices[2] - body2.vertices[0]));
        real_t d =  dot((body1.position - body2.position), n);

        if (fabs(d) < body1.radius)
        {
            Vector3 q = body1.position - d * n;
            Vector3 a = body2.vertices[0];
            Vector3 b = body2.vertices[1];
            Vector3 c = body2.vertices[2];

            if (dot(cross(b - a, q - a), n) < 0 || dot(cross(c - b, q - b), n) < 0 || dot(cross(a - c, q - c), n) < 0)
                return (false);
            body1.velocity = body1.velocity - 2.0f  * dot(body1.velocity, n) * n;
            body1.velocity -= collision_damping * body1.velocity;
            return (true);
        }
        return(false);
    }

    // Formula from p5.pdf
    bool collides(SphereBody &body1, PlaneBody &body2, real_t collision_damping)
    {
        Vector3 a = body1.position - body2.position;
        real_t d = dot(a, body2.normal);

        if (fabs(d) < body1.radius)
        {
            body1.velocity = body1.velocity - 2.0f * dot(body1.velocity, body2.normal) * body2.normal;
            body1.velocity -= collision_damping * body1.velocity;
            return true;
        }
        return false;
    }
}