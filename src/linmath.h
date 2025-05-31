#pragma once
#include <cmath>

struct Vec2 {
    float x;
    float y;
};

struct matrix {
    Vec2 V1;
    Vec2 V2;
};

float compDist(Vec2 V1, Vec2 V2) {
    float dx = V1.x - V2.x;
    float dy = V1.y - V2.y;
    return std::sqrt(dx * dx + dy * dy);
}

float VectorMagnitude(Vec2 V) {
    return std::sqrt(V.x * V.x + V.y * V.y);
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

float DotProduct(Vec2 V1, Vec2 V2) {
    return V1.x * V2.x + V1.y * V2.y;
}
