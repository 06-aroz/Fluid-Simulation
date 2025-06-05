#pragma once
#include "definition.hpp"

#include <cmath>
#include <random>

float DotProduct(const Vec2 &V1, const Vec2 &V2) {
    return V1.x * V2.x + V1.y * V2.y;
}

float VectorMagnitude(Vec2 V) {
    return std::sqrt(DotProduct(V, V));
}

float compDist(Vec2 V1, Vec2 V2) {
    float dx = V1.x - V2.x;
    float dy = V1.y - V2.y;
    return std::sqrt(dx * dx + dy * dy);
}

Vec2 ScaleVector(Vec2 V1, float scalar) {
    return {V1.x * scalar, V1.y * scalar};
}

Vec2 Normalize(Vec2 V) {
    return V / VectorMagnitude(V);
}

Vec2 VectorSum(Vec2 V1, Vec2 V2, bool is_difference) {
    Vec2 sum;
    if (is_difference) {
        V2.x *= -1;
        V2.y *= -1;
    }
    sum.x = V1.x + V2.x;
    sum.y = V1.y + V2.y;
    return sum;
}

// Change this
float randomRange(float min, float max) {
    // Create a random number generator and seed it with a random device
    static std::random_device rd;
    static std::mt19937 gen(rd());

    // Create a uniform real distribution in the given range
    std::uniform_real_distribution<float> dist(min, max);

    return dist(gen);
}
