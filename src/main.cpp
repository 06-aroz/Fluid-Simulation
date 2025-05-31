#include <GLUT/glut.h>
#include <iostream>
#include <random>
#include <vector>

#include "colrender.h"
#include "glutInit.h"
#include "linmath.h"

int number_objects = 50e3;

float screen_width = 800;
float screen_height = screen_width * 3.0 / 4.0;
float zoom = 1.0f;
const float damp_factor = 0.95f;
const color bg_color = {40.0 / 255.0, 40.0 / 255.0, 45.0 / 255.0};

const float &max_speed = 100.0f;
const float &min_radius = 1.0f;
const float &max_radius = 8.0f;

bool elastic_collisions = false;
bool color_map = true;
bool is_paused = false;

float dt = 0.0f;
float scale_time = 60.0f / 4;

int path_length = 500;
int last_time = 0;

// Delta time implementation
void updateTime() {
    int cur_time = glutGet(GLUT_ELAPSED_TIME);
    dt = scale_time * static_cast<float>(cur_time - last_time) / 1000.0f;
    last_time = cur_time;
}

class ObjectBody {
  public:
    Vec2 position;
    Vec2 velocity;
    float radius;
    float mass;

    ObjectBody(Vec2 position, Vec2 velocity, float radius) {
        this->position = position;
        this->velocity = velocity;
        this->radius = radius;
        this->mass = radius;
    }

    void updateVelocity(Vec2 acceleration) {
        this->velocity.x += acceleration.x * dt;
        this->velocity.y += acceleration.y * dt;
    }

    // Euler's method
    void updatePositionE() {
        this->position.x += this->velocity.x * dt;
        this->position.y += this->velocity.y * dt;
    }

    // Verlet integration
    Vec2 init_acc = {0.0f, 0.0f};

    void updatePositionV(Vec2 (*applyAccleration)()) {
        Vec2 new_pos;
        Vec2 new_acc;
        Vec2 new_vel;

        new_pos.x = position.x + velocity.x * dt + init_acc.x * dt * dt * 0.5;
        new_pos.y = position.y + velocity.y * dt + init_acc.y * dt * dt * 0.5;
        new_acc.x = applyAccleration().x;
        new_acc.y = applyAccleration().y;
        new_vel.x = velocity.x + (init_acc.x + new_acc.x) * (dt * 0.5);
        new_vel.y = velocity.y + (init_acc.y + new_acc.y) * (dt * 0.5);

        this->position.x = new_pos.x;
        this->position.y = new_pos.y;

        this->velocity.x = new_vel.x;
        this->velocity.y = new_vel.y;

        this->init_acc.x = new_acc.x;
        this->init_acc.y = new_acc.y;
    }
};

// Creating circle
void createCircle(Vec2 position, float radius, color obj_color) {
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

float randomRange(float min, float max) {
    // Create a random number generator and seed it with a random device
    static std::random_device rd;
    static std::mt19937 gen(rd());

    // Create a uniform real distribution in the given range
    std::uniform_real_distribution<float> dist(min, max);

    return dist(gen);
}

void detectBorder(ObjectBody &one) {
    if (one.position.y + one.radius >= screen_height / zoom) {
        one.position.y = (screen_height - one.radius) / zoom;
        one.velocity.y *= -damp_factor;
    }

    if (one.position.y - one.radius <= -screen_height / zoom) {
        one.position.y = (-screen_height + one.radius) / zoom;
        one.velocity.y *= -damp_factor;
    }

    if (one.position.x + one.radius >= screen_width / zoom) {
        one.position.x = (screen_width - one.radius) / zoom;
        one.velocity.x *= -damp_factor;
    }

    if (one.position.x - one.radius <= -screen_width / zoom) {
        one.position.x = (-screen_width + one.radius) / zoom;
        one.velocity.x *= -damp_factor;
    }
}

// Generate objects in random positions

std::vector<ObjectBody> genRandPositions(int number_objects) {
    std::vector<ObjectBody> Bodies;

    for (int i = 0; i < number_objects; i++) {
        Vec2 bod_pos = {
            randomRange(-screen_width + 10.0f, screen_width - 10.0f),
            randomRange(-screen_height + 10.0f, screen_height - 10.0f)};
        Vec2 bod_vel = {randomRange(-max_speed, max_speed),
                        randomRange(-max_speed, max_speed)};
        float bod_rad = randomRange(min_radius, max_radius);
        Bodies.push_back(ObjectBody(bod_pos, bod_vel, bod_rad));
    }
    return Bodies;
}

// assuming acceleration is only position dependent
Vec2 simpleGravity() {
    Vec2 gravity = {0.0f, -5.0f};
    return gravity;
}

Vec2 getMeanVelocity(std::vector<ObjectBody> body_list) {
    Vec2 avg_velocity = {0.0f, 0.0f};
    for (auto &body : body_list) {
        avg_velocity.x += body.velocity.x;
        avg_velocity.y += body.velocity.y;
    }

    if (!body_list.empty()) {
        avg_velocity.x /= body_list.size();
        avg_velocity.y /= body_list.size();
    }
    return avg_velocity;
}

std::vector<ObjectBody> body_list = genRandPositions(number_objects);

// Checking collisions, only check square for distanc
bool CheckCollision(ObjectBody &one, ObjectBody &two) {
    float error = min_radius;
    float dx = one.position.x - two.position.x;
    float dy = one.position.y - two.position.y;
    float dist_sq = dx * dx + dy * dy;
    float radiusSum = one.radius + two.radius + error;

    return dist_sq <= radiusSum * radiusSum;
}

// Computing collisions (assuming elastic)

matrix ElasticCheck(ObjectBody &one, ObjectBody &two) {
    Vec2 velocity1;
    Vec2 velocity2;
    matrix velocity_matrix;

    float mass_difference = one.mass - two.mass;
    float mass_sum = one.mass + two.mass;

    Vec2 velocity_difference1 = VectorSum(one.velocity, two.velocity, true);
    Vec2 velocity_difference2 = VectorSum(two.velocity, one.velocity, true);

    Vec2 position_difference1 = VectorSum(one.position, two.position, true);
    Vec2 position_difference2 = VectorSum(two.position, one.position, true);

    float vector_scale1 =
        (2.0f * two.mass) / mass_sum *
        DotProduct(velocity_difference1, position_difference1) /
        (position_difference1.x * position_difference1.x +
         position_difference1.y * position_difference1.y);
    float vector_scale2 =
        (2.0f * one.mass) / mass_sum *
        DotProduct(velocity_difference2, position_difference2) /
        (position_difference2.x * position_difference2.x +
         position_difference2.y * position_difference2.y);

    velocity1.x = one.velocity.x - position_difference1.x * vector_scale1;
    velocity1.y = one.velocity.y - position_difference1.y * vector_scale1;
    velocity2.x = two.velocity.x - position_difference2.x * vector_scale2;
    velocity2.y = two.velocity.y - position_difference2.y * vector_scale2;

    velocity_matrix.V1 = velocity1;
    velocity_matrix.V2 = velocity2;

    return velocity_matrix;
}

// Applying the collisions
void detectCollision() {
    for (size_t i = 0; i < body_list.size(); i++) {
        for (size_t j = i + 1; j < body_list.size(); j++) {
            if (CheckCollision(body_list[i], body_list[j])) {
                matrix updatedvelocity =
                    ElasticCheck(body_list[i], body_list[j]);
                body_list[i].velocity = updatedvelocity.V1;
                body_list[j].velocity = updatedvelocity.V2;
            }
        }
    }
}

void mainDisplay(std::vector<ObjectBody> &body_list) {
    updateTime();
    if (elastic_collisions) {
        detectCollision();
    }

    // Main generation loop
    for (auto &body : body_list) {
        color body_col;
        if (color_map) {
            body_col = mapColors(body.velocity, max_speed);
        } else {
            body_col = {1.0, 1.0, 1.0};
        }
        createCircle(body.position, body.radius, body_col);
        detectBorder(body);

        if (!is_paused) {
            body.updatePositionV(simpleGravity);
        }
    }
}

// This is where all the main physics/objecst are rendered
void displayCallback() {
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    glClearColor(bg_color.R, bg_color.G, bg_color.B, 0.5);
    glScalef(zoom, zoom, 1.0);
    mainDisplay(body_list);
    glutSwapBuffers();
}

void handleKeypress(unsigned char key, int x, int y) {
    switch (key) {
    case 27:
        std::cout << "Exiting Program" << std::endl;
        std::exit(0);
        break;
    case 'R':
        zoom = 1.0;
        body_list.clear();
        body_list = genRandPositions(number_objects);
        break;
    case 'r':
        body_list.clear();
        break;
    case 'p':
        elastic_collisions = !elastic_collisions;
        break;
    case 'c':
        color_map = !color_map;
        break;
    case ' ':
        is_paused = !is_paused;
        break;
    case '+':
        zoom *= 1.5f;
        break;
    case '_':
        zoom /= 1.5f;
        break;
    }
}

void handleKeyRelease(unsigned char key, int x, int y) {}

// For resizing the window
void checkResize(int new_width, int new_height) {
    // screen_width and screen_height have to be global
    screen_width = new_width;
    screen_height = new_height;
    glViewport(0, 0, new_width, new_height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-new_width, new_width, -new_height, new_height, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    displayCallback(); // refresh window.
}

// Main function for rendering
void GlutStart(int argc, char **argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(screen_width, screen_height);
    glutCreateWindow("Particle Simulation");

    glMatrixMode(GL_PROJECTION);

    glOrtho(-screen_width, screen_width, -screen_height, screen_height, -1.0,
            1.0);

    glutDisplayFunc(displayCallback);
    glutReshapeFunc(checkResize);
    glutIdleFunc(displayCallback);
    glutKeyboardFunc(handleKeypress);
    glutKeyboardUpFunc(handleKeyRelease);
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
    std::cout << "  +/-           : Zoom In/Out" << std::endl;
    std::cout << "  c             : Toggle color mapping" << std::endl;
    std::cout << "=============================================" << std::endl;
}

int main(int argc, char **argv) {
    showMessage();
    GlutStart(argc, argv);
    glutMainLoop();

    return 0;
}
