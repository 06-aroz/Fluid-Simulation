#pragma once
#include "definition.hpp"
#include "math.hpp"

#include <GLUT/glut.h>
#include <cmath>
#include <vector>

void createCircle(Vec2 position, float radius, Color obj_color) {
    int resolution = 16;
    float angle = 0.0;

    glColor3f(obj_color.R, obj_color.G, obj_color.B);
    glBegin(GL_TRIANGLE_FAN);
    glVertex2d(position.x, position.y);

    for (int i = 0; i <= resolution; i++) {
        angle = 2.0 * M_PI * (static_cast<float>(i) / resolution);
        float x = radius * std::cos(angle) + position.x;
        float y = radius * std::sin(angle) + position.y;

        glVertex2d(x, y);
    }
    glEnd();
}

std::vector<Particle> genRandPositions(int number_objects) {
    std::vector<Particle> Bodies;

    for (int i = 0; i < number_objects; i++) {
        Vec2 bod_pos = {randomRange(-Screen_Settings.screen_width + 10.0f, Screen_Settings.screen_width - 10.0f),
                        randomRange(-Screen_Settings.screen_height + 10.0f, Screen_Settings.screen_height - 10.0f)};
        Vec2 bod_vel = {randomRange(-Particle_Settings.max_speed, Particle_Settings.max_speed),
                        randomRange(-Particle_Settings.max_speed, Particle_Settings.max_speed)};
        float bod_rad = randomRange(Particle_Settings.min_radius, Particle_Settings.max_radius);
        Bodies.push_back(Particle(bod_pos, bod_vel, bod_rad));
    }
    return Bodies;
}
