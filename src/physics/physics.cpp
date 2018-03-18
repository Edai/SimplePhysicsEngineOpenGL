#include <iostream>
#include "physics/physics.hpp"

namespace _462 {

    Physics::Physics() {
        reset();
    }

    Physics::~Physics() {
        reset();
    }

    void Physics::step(real_t dt) {
        for (SphereList::iterator sphere = spheres.begin(); sphere != spheres.end(); sphere++) {
            SphereBody &s = *(*sphere);

            s.force = Vector3::Zero;
            s.apply_force(gravity * s.mass, Vector3::Zero);
            real_t i = 2.0f / 5.0f *  s.mass *  s.radius *  s.radius;
            s.angular_velocity +=  s.torque / i * dt;

            for (SphereList::iterator iter_spheres = spheres.begin(); iter_spheres != spheres.end(); iter_spheres++) {
                if (sphere != iter_spheres)
                    collides(s, *(*iter_spheres), collision_damping);
            }
            for (TriangleList::iterator t = triangles.begin(); t != triangles.end(); t++) {
                collides(s, *(*t), collision_damping);
            }
            for (PlaneList::iterator iter_planes = planes.begin(); iter_planes != planes.end(); iter_planes++) {
                collides(s, *(*iter_planes), collision_damping);
            }
            for (SpringList::iterator iter_springs = springs.begin(); iter_springs != springs.end(); iter_springs++) {
                (*iter_springs)->step(dt);
            }
            s.step_position(dt, collision_damping);
            s.step_orientation(dt, collision_damping);
        }
    }

    void Physics::add_sphere(SphereBody *b) {
        spheres.push_back(b);
    }

    size_t Physics::num_spheres() const {
        return spheres.size();
    }

    void Physics::add_plane(PlaneBody *p) {
        planes.push_back(p);
    }

    size_t Physics::num_planes() const {
        return planes.size();
    }

    void Physics::add_triangle(TriangleBody *t) {
        triangles.push_back(t);
    }

    size_t Physics::num_triangles() const {
        return triangles.size();
    }

    void Physics::add_spring(Spring *s) {
        springs.push_back(s);
    }

    size_t Physics::num_springs() const {
        return springs.size();
    }

    void Physics::reset() {
        for (SphereList::iterator i = spheres.begin(); i != spheres.end(); i++) {
            delete *i;
        }
        for (PlaneList::iterator i = planes.begin(); i != planes.end(); i++) {
            delete *i;
        }
        for (TriangleList::iterator i = triangles.begin(); i != triangles.end(); i++) {
            delete *i;
        }
        for (SpringList::iterator i = springs.begin(); i != springs.end(); i++) {
            delete *i;
        }

        spheres.clear();
        planes.clear();
        triangles.clear();
        springs.clear();

        gravity = Vector3::Zero;
        collision_damping = 0.0;
    }

}
