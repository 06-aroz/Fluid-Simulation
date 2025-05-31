#pragma once
#include "linmath.h"
#include <cmath>
#include <vector>

struct color {
    float R;
    float G;
    float B;
};

std::vector<float> weightFunction(float t) {
    const int number_components = 6;
    std::vector<float> weights(7, 0.0f);

    t = std::clamp(t, 0.0f, 1.0f);
    float scaled = t * number_components;
    int component_index = static_cast<int>(scaled);
    float local_t = scaled - component_index;

    if (component_index >= 6)
        component_index = 5, local_t = 1.0f;

    weights[component_index] = 1.0f - local_t;
    weights[component_index + 1] = local_t;

    return weights;
}

color vectorMapper(float t) {
    // Defining our basis colors in a matrix
    float col_mat[7][3] = {{0.0f, 0.4f, 0.9f}, {0.0f, 0.75f, 0.9f},
                           {0.0f, 1.0f, 0.8f}, {0.5f, 1.0f, 0.4f},
                           {0.9f, 1.0f, 0.4f}, {1.0f, 0.9f, 0.4f},
                           {1.0f, 0.4f, 0.4f}};
    std::vector<float> w_vec = weightFunction(t);
    float col_vec[3] = {0.0, 0.0, 0.0};

    // Performing the matrix transformation
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) {
            col_vec[i] += col_mat[j][i] * w_vec[j];
        }
    }

    return {std::clamp(col_vec[0], 0.0f, 1.0f),
            std::clamp(col_vec[1], 0.0f, 1.0f),
            std::clamp(col_vec[2], 0.0f, 1.0f)};
}

color mapColors(Vec2 obj_velocity, float max_speed) {
    float t;
    float obj_speed =
        obj_velocity.x * obj_velocity.x + obj_velocity.y * obj_velocity.y;
    if (obj_speed > 4 * max_speed * max_speed) {
        return vectorMapper(1.0f);
    }

    t = obj_speed / (max_speed * max_speed);

    return vectorMapper(t);
}
