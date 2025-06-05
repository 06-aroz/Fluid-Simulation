#pragma once

#include "definition.hpp"
#include "math.hpp"

void detectBorder(Particle &one) {
    if (one.position.y + one.radius >= Screen_Settings.screen_height / Screen_Settings.zoom) {
        one.position.y = (Screen_Settings.screen_height / Screen_Settings.zoom - one.radius);
        one.velocity.y *= -Particle_Settings.damp_factor;
    }

    if (one.position.y - one.radius <= -Screen_Settings.screen_height / Screen_Settings.zoom) {
        one.position.y = (-Screen_Settings.screen_height / Screen_Settings.zoom + one.radius);
        one.velocity.y *= -Particle_Settings.damp_factor;
    }

    if (one.position.x + one.radius >= Screen_Settings.screen_width / Screen_Settings.zoom) {
        one.position.x = (Screen_Settings.screen_width / Screen_Settings.zoom - one.radius);
        one.velocity.x *= -Particle_Settings.damp_factor;
    }

    if (one.position.x - one.radius <= -Screen_Settings.screen_width / Screen_Settings.zoom) {
        one.position.x = (-Screen_Settings.screen_width / Screen_Settings.zoom + one.radius);
        one.velocity.x *= -Particle_Settings.damp_factor;
    }
}

Vec2 getMeanVelocity(std::vector<Particle> body_list) {
    Vec2 avg_velocity;
    for (auto &body : body_list) {
        avg_velocity += body.velocity;
    }

    if (!body_list.empty()) {
        avg_velocity /= (body_list.size());
    }
    return avg_velocity;
}

// assuming acceleration is only position dependent
Vec2 simpleGravity() {
    Vec2 gravity = {0.0f, -5.0f};
    return gravity;
}

// Checking collisions, only check square for distanc
bool CheckCollision(Particle &one, Particle &two) {
    float error = Particle_Settings.min_radius * 0.5;
    float dx = one.position.x - two.position.x;
    float dy = one.position.y - two.position.y;
    float dist_sq = dx * dx + dy * dy;
    float radiusSum = one.radius + two.radius + error;

    return dist_sq <= radiusSum * radiusSum;
}

// Computing collisions (assuming elastic)
matrix ElasticCheck(Particle &one, Particle &two) {
    matrix velocity_matrix;

    float mass_difference = one.mass - two.mass;
    float mass_sum = one.mass + two.mass;

    Vec2 velocity_difference = one.velocity - two.velocity;
    Vec2 position_difference = one.position - two.position;

    float vector_scale = 2.0f / mass_sum * DotProduct(velocity_difference, position_difference) /
                         (position_difference.x * position_difference.x + position_difference.y * position_difference.y);

    const Vec2 &velocity1 = one.velocity - position_difference * two.mass * vector_scale;
    const Vec2 &velocity2 = two.velocity + position_difference * one.mass * vector_scale;

    velocity_matrix.V1 = velocity1;
    velocity_matrix.V2 = velocity2;

    return velocity_matrix;
}

// Updating each particle's potision after collision
matrix PositionCollide(Particle &one, Particle &two) {
    matrix position_matrix;
    const float &dX = one.position.x - two.position.x;
    const float &dY = one.position.y - two.position.y;
    float min_dist = one.radius + two.radius;
    const float &distsq = dX * dX + dY * dY;

    if (distsq < min_dist * min_dist) {
        float dist = std::sqrt(distsq);
        Vec2 Normal_dr = ScaleVector({dX, dY}, 1.0 / dist); // normalizing the position vector
        float mass_ratio = 1.0 / (1.0 + (two.radius / one.radius) * (two.radius / one.radius));
        float delta = 0.5f * (min_dist - dist);

        // Mass ratio allows objects of larger size to move less
        one.position += Normal_dr * (1 - mass_ratio) * delta;
        two.position -= Normal_dr * mass_ratio * delta;
    }

    position_matrix.V1 = one.position;
    position_matrix.V2 = two.position;

    return position_matrix;
}
