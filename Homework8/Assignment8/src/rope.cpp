#include <iostream>
#include <vector>

#include "CGL/vector2D.h"
#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        for(int i = 0; i < num_nodes; i++) {
            masses.push_back(new Mass(start + (end - start) * i / (num_nodes - 1), node_mass, false));
        }
        for(int i = 0; i < num_nodes - 1; i++) {
            springs.push_back(new Spring(masses[i], masses[i + 1], k));
        }
//        Comment-in this part when you implement the constructor
       for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
       }
       masses[0]->pinned = true;
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D force = ((s->m2->position - s->m1->position).norm() - s->rest_length) * s->k * (s->m2->position - s->m1->position).unit();
            s->m1->forces += force;
            s->m2->forces -= force;
        }
        float kd = 0.005f;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                // Vector2D accel = m->forces / m->mass;
                // m->position = m->position + m->velocity * delta_t;
                // Explicit Euler: update position first, then velocity.
                // m->velocity = m->velocity + accel * delta_t;

                // TODO (Part 2): Add global damping
                Vector2D accel = m->forces / m->mass - kd * m->velocity/m->mass;
                m->position = m->position + m->velocity * delta_t;
                // Explicit Euler: update position first, then velocity.
                m->velocity = m->velocity + accel * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
    // Simplicit Euler
    void Rope::simulateEuler_SimiImplicit(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D force = ((s->m2->position - s->m1->position).norm() - s->rest_length) * s->k * (s->m2->position - s->m1->position).unit();
            s->m1->forces += force;
            s->m2->forces -= force;
        }

        float kd = 0.005f;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                // Vector2D accel = m->forces / m->mass;
                // // Symplectic (semi-implicit) Euler: update velocity first, then position.
                // m->velocity = m->velocity + accel * delta_t;
                // m->position = m->position + m->velocity * delta_t;

                // TODO (Part 2): Add global damping
                Vector2D accel = m->forces / m->mass - kd * m->velocity/m->mass;
                // Symplectic (semi-implicit) Euler: update velocity first, then position.
                m->velocity = m->velocity + accel * delta_t;
                m->position = m->position + m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D force = ((s->m2->position - s->m1->position).norm() - s->rest_length) * s->k * (s->m2->position - s->m1->position).unit();
            s->m1->forces += force;
            s->m2->forces -= force;
        }
        float kd = 0.00005f;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity * m->mass;
                Vector2D accel = m->forces / m->mass;
                // m->position = temp_position + (temp_position - m->last_position) + accel * delta_t * delta_t;
                // TODO (Part 4): Add global Verlet damping
                m->position = temp_position + (1.f - kd) * (temp_position - m->last_position) + accel * delta_t * delta_t;

                m->last_position = temp_position;
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
