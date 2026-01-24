#include <SDL2/SDL.h>
#include <iostream>
#include <cmath>
#include "robotics_algo/kinematics/n_link_arm.hpp"

using namespace robotics;
using namespace robotics::kinematics;

// --- CAMERA GLOBALS (High Angle View) ---
double cam_yaw = 0.0;     // Start aligned with Z-axis
double cam_pitch = 1.3;   // High angle (~75 degrees looking down)
double cam_dist = 750.0;  // Zoom out to see the floor plan
const double SCREEN_W = 800;
const double SCREEN_H = 600;

struct Point2D { int x, y; double scale; bool visible; };

// Full 3D Orbit Projection
Point2D project(Vector3 p) {
    // 1. Apply YAW (Rotate around Y axis)
    double x1 = p.x() * std::cos(cam_yaw) - p.z() * std::sin(cam_yaw);
    double z1 = p.x() * std::sin(cam_yaw) + p.z() * std::cos(cam_yaw);
    double y1 = p.y();

    // 2. Apply PITCH (Rotate around X axis)
    double y2 = y1 * std::cos(cam_pitch) - z1 * std::sin(cam_pitch);
    double z2 = y1 * std::sin(cam_pitch) + z1 * std::cos(cam_pitch);
    double x2 = x1;

    // 3. Apply Camera Distance
    double depth = z2 + cam_dist;
    if(depth < 1.0) return {0, 0, 0.0, false};

    // 4. Perspective Divide
    double fov = 600.0;
    double scale = fov / depth;

    return {
        (int)(SCREEN_W/2 + x2 * scale),
        (int)(SCREEN_H/2 - y2 * scale), 
        scale,
        true
    };
}

void drawLine3D(SDL_Renderer* ren, Vector3 p1, Vector3 p2, int r, int g, int b, int base_thick=2) {
    Point2D s1 = project(p1); Point2D s2 = project(p2);
    if(!s1.visible || !s2.visible) return;

    SDL_SetRenderDrawColor(ren, r, g, b, 255);
    int thick = (s1.scale + s2.scale) * 0.5 * base_thick; 
    if(thick < 1) thick = 1;
    for(int i=-thick/2; i<=thick/2; ++i) {
         SDL_RenderDrawLine(ren, s1.x+i, s1.y, s2.x+i, s2.y);
         SDL_RenderDrawLine(ren, s1.x, s1.y+i, s2.x, s2.y+i);
    }
}

void drawPoint3D(SDL_Renderer* ren, Vector3 p, int r, int g, int b, int size_base) {
    Point2D s = project(p);
    if(!s.visible) return;
    SDL_SetRenderDrawColor(ren, r, g, b, 255);
    int size = size_base * s.scale;
    SDL_Rect rect = {s.x - size/2, s.y - size/2, size, size};
    SDL_RenderFillRect(ren, &rect);
}

int main() {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* win = SDL_CreateWindow("Drone View Snake (Drag to Rotate)", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, (int)SCREEN_W, (int)SCREEN_H, SDL_WINDOW_SHOWN);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    // 10 Links, 30 units each. Base at (0, 0, 0)
    NLinkArm3D snake(10, 30.0, Vector3(0, 0, 0));
    
    Vector3 target(100, 100, 0); 
    bool running = true;
    bool dragging = false;

    while(running) {
        SDL_Event e;
        while(SDL_PollEvent(&e)) {
            if(e.type == SDL_QUIT) running = false;
            
            // Camera Orbit
            if(e.type == SDL_MOUSEBUTTONDOWN) dragging = true;
            if(e.type == SDL_MOUSEBUTTONUP) dragging = false;
            if(e.type == SDL_MOUSEMOTION && dragging) {
                cam_yaw -= e.motion.xrel * 0.01;
                cam_pitch += e.motion.yrel * 0.01;
                // Clamp Pitch (Allow looking straight down, but not flipping over)
                if(cam_pitch > 1.55) cam_pitch = 1.55; // 89 degrees
                if(cam_pitch < 0.1) cam_pitch = 0.1;   // Don't go below floor level
            }
            if(e.type == SDL_MOUSEWHEEL) {
                cam_dist -= e.wheel.y * 30.0;
                if(cam_dist < 200) cam_dist = 200;
            }
        }

        // Controls (Relative to Camera)
        const Uint8* keys = SDL_GetKeyboardState(NULL);
        double cy = std::cos(cam_yaw); 
        double sy = std::sin(cam_yaw);
        
        Vector3 cam_fwd(sy, 0, cy);
        Vector3 cam_right(cy, 0, -sy);

        double speed = 4.0;
        if(keys[SDL_SCANCODE_UP])    target.y() += speed;
        if(keys[SDL_SCANCODE_DOWN])  target.y() -= speed;
        if(keys[SDL_SCANCODE_W])     target = target + cam_fwd * speed;
        if(keys[SDL_SCANCODE_S])     target = target - cam_fwd * speed;
        if(keys[SDL_SCANCODE_D])     target = target + cam_right * speed;
        if(keys[SDL_SCANCODE_A])     target = target - cam_right * speed;

        // Solve IK
        snake.solveIK(target);

        // Render
        SDL_SetRenderDrawColor(ren, 15, 15, 20, 255);
        SDL_RenderClear(ren);

        // Grid (Floor)
        for(int i=-400; i<=400; i+=100) {
            drawLine3D(ren, Vector3(i, 0, -400), Vector3(i, 0, 400), 40, 40, 50, 1);
            drawLine3D(ren, Vector3(-400, 0, i), Vector3(400, 0, i), 40, 40, 50, 1);
        }

        // Snake
        const auto& joints = snake.getJoints();
        for(size_t i=0; i<joints.size()-1; ++i) {
            int r = 0; int g = (i * 255) / joints.size(); int b = 255 - g;
            drawLine3D(ren, joints[i], joints[i+1], r, g, b, 4);
            drawPoint3D(ren, joints[i], 200, 200, 200, 6);
        }
        
        // Target & Shadow
        drawPoint3D(ren, target, 255, 50, 50, 10);
        drawLine3D(ren, target, Vector3(target.x(), 0, target.z()), 255, 50, 50, 1);

        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
