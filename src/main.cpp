#include <GLUT/glut.h>
#include <iostream>
#include <vector>

#include "colors.hpp"
#include "computation.hpp"
#include "definition.hpp"
#include "math.hpp"
#include "render.hpp"

// Generate objects in random positions
std::vector<Particle> body_list = genRandPositions(Particle_Settings.number_objects);

void spawnParticle(Vec2 position, float radius) {
    body_list.push_back(Particle(position, {0, 0}, radius));
}

// Applying the collisions. O(n^2)
void detectCollision() {
    for (size_t i = 0; i < body_list.size(); i++) {
        for (size_t j = i + 1; j < body_list.size(); j++) {
            if (CheckCollision(body_list[i], body_list[j])) {
                const matrix &updatedposition = PositionCollide(body_list[i], body_list[j]);

                body_list[i].position = updatedposition.V1;
                body_list[j].position = updatedposition.V2;

                const matrix &updatedvelocity = ElasticCheck(body_list[i], body_list[j]);
                body_list[i].velocity = updatedvelocity.V1;
                body_list[j].velocity = updatedvelocity.V2;
            }
        }
    }
}

/*
Vec2 ClickForce(Vec2 mousePos, Particle &one) {
    Vec2 Force = {0, 0};
    Vec2 offset = mousePos - body_list[2].position;
    const float strength = 10.0;
    const float &distsq = DotProduct(offset, offset);
    float error = 0.25;

    // If inside input radius, calculate force towards input point
    if (distsq < one.radius * one.radius) {
        Vec2 dirToInput = distsq <= error ? Vec2{0.0, 0.0} : Normalize(offset);
        float centre = 1 - std::sqrt(distsq) / one.radius;
        Force += (dirToInput * strength - body_list[2].velocity) * centre;
    }
    return Force;
}
*/

void mainDisplay(std::vector<Particle> &body_list) {
    Screen_Settings.updateTime();
    if (Screen_Settings.elastic_collisions) {
        detectCollision();
    }

    float elapsed_time = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;

    // Main generation loop
    for (auto &body : body_list) {
        Color body_col;
        if (!Screen_Settings.is_paused) {
            body.updatePositionV(simpleGravity, Screen_Settings.dt);
        }

        if (Screen_Settings.Color_map) {
            body_col = mapColors(body.velocity, Particle_Settings.max_speed);
            // body_col = mapPeriodColors(elapsed_time);
        } else {
            body_col = {1.0, 1.0, 1.0};
        }
        createCircle(body.position, body.radius, body_col);
        detectBorder(body);
    }
}

// This is where all the main physics/objecst are rendered
void displayCallback() {
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    glClearColor(Screen_Settings.bg_color.R, Screen_Settings.bg_color.G, Screen_Settings.bg_color.B, 0.5);
    glScalef(Screen_Settings.zoom, Screen_Settings.zoom, 1.0);
    mainDisplay(body_list);
    glutSwapBuffers();
}

// Mouse click logic
void onClick(int button, int state, int x, int y) {
    switch (button) {
    case GLUT_LEFT_BUTTON:
        if (state == GLUT_DOWN) {
            float mouse_x = static_cast<float>(x);
            float mouse_y = static_cast<float>(y);
            float true_x = (2.0f * mouse_x - Screen_Settings.screen_width) / Screen_Settings.zoom;
            float true_y = (-2.0f * mouse_y + Screen_Settings.screen_height) / Screen_Settings.zoom;
            spawnParticle({true_x, true_y}, 50);
        }
        break;
    case GLUT_RIGHT_BUTTON:
        if (state == GLUT_DOWN) {
            // despawnBody();
        }
        break;
    }
}

void handleKeypress(unsigned char key, int x, int y) {
    switch (key) {
    case 27:
        std::cout << "Exiting Program" << std::endl;
        std::exit(0);
        break;
    case 'R':
        Screen_Settings.zoom = 1.0;
        body_list.clear();
        body_list = genRandPositions(Particle_Settings.number_objects);
        break;
    case 'r':
        body_list.clear();
        break;
    case 'p':
        Screen_Settings.elastic_collisions = !Screen_Settings.elastic_collisions;
        break;
    case 'c':
        Screen_Settings.Color_map = !Screen_Settings.Color_map;
        break;
    case ' ':
        Screen_Settings.is_paused = !Screen_Settings.is_paused;
        Screen_Settings.elastic_collisions = !Screen_Settings.elastic_collisions;
        break;
    case '+':
        Screen_Settings.zoom *= 1.5f;
        break;
    case '_':
        Screen_Settings.zoom /= 1.5f;
        break;
    }
}

// For resizing the window
void checkResize(int new_width, int new_height) {
    Screen_Settings.screen_width = new_width;
    Screen_Settings.screen_height = new_height;
    glViewport(0, 0, new_width, new_height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-new_width, new_width, -new_height, new_height, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    displayCallback();
}

// Main function for rendering
void GlutStart(int argc, char **argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(Screen_Settings.screen_width, Screen_Settings.screen_height);
    glutCreateWindow("Particle Simulation");
    glutInitWindowPosition(100, 100);

    glMatrixMode(GL_PROJECTION);

    glOrtho(-Screen_Settings.screen_width, Screen_Settings.screen_width, -Screen_Settings.screen_height, Screen_Settings.screen_height, -1.0, 1.0);

    glutDisplayFunc(displayCallback);
    glutIdleFunc(displayCallback);
    glutReshapeFunc(checkResize);

    glutKeyboardFunc(handleKeypress);
    glutMouseFunc(onClick);
}

void showMessage() {
    std::cout << "=============================================" << std::endl;
    std::cout << "              Simulation Started" << std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << "                  Controls" << std::endl;
    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "  ESC           : Exit" << std::endl;
    std::cout << "  R             : Reset simulation" << std::endl;
    std::cout << "  r             : Clear all bodies" << std::endl;
    std::cout << "  SPACE         : Pause/unpause" << std::endl;
    std::cout << "  p             : Toggle collisions" << std::endl;
    std::cout << "  b             : Toggle border collisions" << std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << "              Visual Controls" << std::endl;
    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "  +/-           : Screen_Settings.zoom In/Out" << std::endl;
    std::cout << "  c             : Toggle Color mapping" << std::endl;
    std::cout << "=============================================" << std::endl;
}

int main(int argc, char **argv) {
    showMessage();
    GlutStart(argc, argv);
    glutMainLoop();

    return 0;
}
