#pragma once
#include <GLUT/glut.h>

struct Vec2 {
    float x;
    float y;

    // Constructors
    Vec2() : x(0.0f), y(0.0f) {} // Default state
    Vec2(float x_, float y_) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2 &other) {
        return Vec2(x + other.x, y + other.y);
    }

    Vec2 operator-(const Vec2 &other) {
        return Vec2(x - other.x, y - other.y);
    }

    Vec2 operator+=(const Vec2 &other) {
        x += other.x;
        y += other.y;
        return Vec2(x, y);
    }

    Vec2 operator-=(const Vec2 &other) {
        x -= other.x;
        y -= other.y;
        return Vec2(x, y);
    }

    Vec2 operator*(const float &scale) {
        return Vec2(x * scale, y * scale);
    }

    Vec2 operator/(const float &scale) {
        return Vec2(x / scale, y / scale);
    }

    Vec2 operator*=(const float &scale) {
        x *= scale;
        y *= scale;
        return Vec2(x, y);
    }

    Vec2 operator/=(const float &scale) {
        x /= scale;
        y /= scale;
        return Vec2(x, y);
    }

    Vec2 operator-() const { // Allows us to state Vec2 V = -V2;
        return Vec2(-x, -y);
    }

    Vec2 zero() {
        x = 0.0f;
        y = 0.0f;
        return {x, y};
    }
};

struct matrix {
    Vec2 V1;
    Vec2 V2;
};

struct Color {
    float R;
    float G;
    float B;
};

class ParticleParameters {
  public:
    float damp_factor;
    float max_speed;
    float min_radius;
    float max_radius;
    int number_objects;

    ParticleParameters(int number_objects, float damp_factor, float max_speed, float max_radius, float min_radius) {
        this->number_objects = number_objects;
        this->damp_factor = damp_factor;
        this->max_speed = max_speed;
        this->max_radius = max_radius;
        this->min_radius = min_radius;
    }
};

class SimParameters {
  public:
    float screen_width;
    float screen_height;
    float zoom = 1.0f;

    float dt = 0.0f;
    float scale_time;
    int last_time = 0;

    bool elastic_collisions = false;
    bool Color_map = true;
    bool is_paused = false;

    const Color bg_color = {40.0 / 255.0, 40.0 / 255.0, 50.0 / 255.0};

    SimParameters(float screen_width, float scale_time) {
        this->screen_width = screen_width;
        this->screen_height = screen_width * 0.75;
        this->scale_time = scale_time;
    }
    // Delta time implementation
    void updateTime() {
        int cur_time = glutGet(GLUT_ELAPSED_TIME);
        dt = scale_time * static_cast<float>(cur_time - last_time) / 1000.0f;
        last_time = cur_time;
    }
};

class Particle {
  public:
    Vec2 position;
    Vec2 velocity;
    float radius;
    float mass;

    Particle(Vec2 position, Vec2 velocity, float radius) {
        this->position = position;
        this->velocity = velocity;
        this->radius = radius;
        this->mass = radius;
    }

    void updateVelocity(Vec2 acceleration, float dt) {
        this->velocity += acceleration * dt;
    }

    // Euler's method
    void updatePositionE(float dt) {
        this->position += this->velocity * dt;
    }

    // Verlet integration
    Vec2 init_acc = {0.0f, 0.0f};

    void updatePositionV(Vec2 (*applyAccleration)(), float dt) {
        Vec2 new_pos;
        Vec2 new_acc;
        Vec2 new_vel;

        new_pos = position + velocity * dt + init_acc * dt * dt * 0.5;
        new_acc = applyAccleration();
        new_vel = velocity + (init_acc + new_acc) * (dt * 0.5);

        this->position = new_pos;
        this->velocity = new_vel;
        this->init_acc = new_acc;
    }
};

SimParameters Screen_Settings(800, 15.0f);
ParticleParameters Particle_Settings(2000, 0.5, 100, 10, 2);
