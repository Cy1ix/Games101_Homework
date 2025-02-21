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
        Vector2D length = (end - start) / (num_nodes - 1);
        masses.push_back(new Mass{ start, node_mass, false });
        for (int i = 1; i < num_nodes; ++i) {
            masses.push_back(new Mass{ start + i * length, node_mass, false });
            springs.push_back(new Spring{ masses[i - 1], masses[i], k });
        }

        for (auto& i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D force_dir = s->m2->position - s->m1->position;
            double force_norm = force_dir.norm();
            Vector2D force = s->k * (force_dir / force_norm) * (force_norm - s->rest_length);
            s->m1->forces += force;
            s->m2->forces += -force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                // TODO (Part 2): Add global damping
                double damping = 0.01;
                m->forces += -damping * m->velocity;

                Vector2D acc = m->forces / m->mass;

                // For explicit method
                // m->position += m->velocity * delta_t;
                // m->velocity += a * delta_t;

                // For semi-implicit method
                m->velocity += acc * delta_t;
                m->position += m->velocity * delta_t;
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
            Vector2D force_dir = s->m2->position - s->m1->position;
            double force_norm = force_dir.norm();
            Vector2D f = s->k * (force_dir / force_norm) * (force_norm - s->rest_length);
            s->m1->forces += f;
            s->m2->forces += -f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                m->forces += gravity * m->mass;
                Vector2D acc = m->forces / m->mass;
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                double damping = 0.00005;
                // TODO (Part 4): Add global Verlet damping
                m->position += (1 - damping) * (m->position - m->last_position) + acc * delta_t * delta_t;
                m->last_position = temp_position;
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
