#include <SDL2/SDL.h>
#include <iostream>
#include <cmath>
#include <vector>
#include "robotics_algo/kinematics/six_link_arm.hpp"

using namespace robotics;
using namespace robotics::kinematics;

// --- CAMERA GLOBALS ---
double cam_yaw = 0.5;
double cam_pitch = 0.5;
double cam_dist = 600.0;
const double SCREEN_W = 800;
const double SCREEN_H = 600;

struct Point2D { int x, y; double scale; bool visible; };

Point2D project(Vector3 p) {
    // 1. Rotate Y (Yaw)
    double x1 = p.x()*cos(cam_yaw) - p.z()*sin(cam_yaw);
    double z1 = p.x()*sin(cam_yaw) + p.z()*cos(cam_yaw);
    double y1 = p.y();
    // 2. Rotate X (Pitch)
    double y2 = y1*cos(cam_pitch) - z1*sin(cam_pitch);
    double z2 = y1*sin(cam_pitch) + z1*cos(cam_pitch);
    double x2 = x1;
    // 3. Perspective
    double depth = z2 + cam_dist;
    if(depth < 1.0) return {0,0,0,false};
    double scale = 600.0 / depth;
    return {(int)(SCREEN_W/2 + x2*scale), (int)(SCREEN_H/2 - y2*scale), scale, true};
}

void drawLine3D(SDL_Renderer* ren, Vector3 p1, Vector3 p2, int r, int g, int b, int w=2) {
    Point2D s1 = project(p1); Point2D s2 = project(p2);
    if(!s1.visible || !s2.visible) return;
    SDL_SetRenderDrawColor(ren, r, g, b, 255);
    for(int i=-w/2; i<=w/2; ++i) {
         SDL_RenderDrawLine(ren, s1.x+i, s1.y, s2.x+i, s2.y);
         SDL_RenderDrawLine(ren, s1.x, s1.y+i, s2.x, s2.y+i);
    }
}

void drawJoint(SDL_Renderer* ren, Vector3 p, int r, int g, int b) {
    Point2D s = project(p);
    if(!s.visible) return;
    SDL_SetRenderDrawColor(ren, r, g, b, 255);
    int rad = 4 * s.scale;
    SDL_Rect rect = {s.x - rad, s.y - rad, rad*2, rad*2};
    SDL_RenderFillRect(ren, &rect);
}

// Full Frame for Robot Tip
void drawFrame(SDL_Renderer* ren, Vector3 pos, Matrix3 rot, double scale) {
    Vector3 x = rot.col(0) * scale;
    Vector3 y = rot.col(1) * scale;
    Vector3 z = rot.col(2) * scale;
    drawLine3D(ren, pos, pos+x, 255, 50, 50, 3); // X Red
    drawLine3D(ren, pos, pos+y, 50, 255, 50, 3); // Y Green
    drawLine3D(ren, pos, pos+z, 50, 50, 255, 3); // Z Blue
}

struct Color { int r, g, b; };
std::vector<Color> link_colors = {
    {100, 100, 100}, // Base
    {255, 140, 0},   // Shoulder
    {255, 215, 0},   // Elbow
    {50, 205, 50},   // Wrist 1
    {0, 191, 255},   // Wrist 2
    {255, 0, 255},   // Wrist 3
    {200, 200, 200}  // Tip
};

int main() {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* win = SDL_CreateWindow("6-DOF Robot (Magenta Line = Target)", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 600, SDL_WINDOW_SHOWN);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    SixLinkArm arm;
    std::vector<double> q = {0, 0, 0, 0, 0, 0};

    Vector3 target_pos(100, 200, 100);
    Matrix3 target_rot = Matrix3::Identity();
    double rx=0, ry=0, rz=0;

    bool running = true;
    bool dragging = false;

    while(running) {
        SDL_Event e;
        while(SDL_PollEvent(&e)) {
            if(e.type == SDL_QUIT) running = false;
            
            if(e.type == SDL_MOUSEBUTTONDOWN) dragging = true;
            if(e.type == SDL_MOUSEBUTTONUP) dragging = false;
            if(e.type == SDL_MOUSEMOTION && dragging) {
                cam_yaw -= e.motion.xrel * 0.01;
                cam_pitch += e.motion.yrel * 0.01;
                if(cam_pitch > 1.5) cam_pitch = 1.5;
                if(cam_pitch < -1.5) cam_pitch = -1.5;
            }
            if(e.type == SDL_MOUSEWHEEL) {
                cam_dist -= e.wheel.y * 50.0;
                if(cam_dist < 100) cam_dist = 100;
            }
        }

        const Uint8* keys = SDL_GetKeyboardState(NULL);

        // Move Position
        if(keys[SDL_SCANCODE_W]) target_pos.z() += 2.0;
        if(keys[SDL_SCANCODE_S]) target_pos.z() -= 2.0;
        if(keys[SDL_SCANCODE_A]) target_pos.x() -= 2.0;
        if(keys[SDL_SCANCODE_D]) target_pos.x() += 2.0;
        if(keys[SDL_SCANCODE_Q]) target_pos.y() += 2.0;
        if(keys[SDL_SCANCODE_E]) target_pos.y() -= 2.0;

        // Rotate Orientation
        if(keys[SDL_SCANCODE_UP])    rx += 0.04;
        if(keys[SDL_SCANCODE_DOWN])  rx -= 0.04;
        if(keys[SDL_SCANCODE_LEFT])  ry += 0.04;
        if(keys[SDL_SCANCODE_RIGHT]) ry -= 0.04;
        if(keys[SDL_SCANCODE_U])     rz += 0.04;
        if(keys[SDL_SCANCODE_I])     rz -= 0.04;

        target_rot = Eigen::AngleAxisd(ry, Vector3::UnitY())
                   * Eigen::AngleAxisd(rx, Vector3::UnitX())
                   * Eigen::AngleAxisd(rz, Vector3::UnitZ());

        // Solve IK
        for(int i=0; i<30; ++i) {
            q = arm.solveIK(q, target_pos, target_rot);
        }

        SDL_SetRenderDrawColor(ren, 15, 15, 20, 255);
        SDL_RenderClear(ren);

        // Floor
        for(int i=-400; i<=400; i+=100) {
            drawLine3D(ren, Vector3(i, 0, -400), Vector3(i, 0, 400), 40, 40, 50, 1);
            drawLine3D(ren, Vector3(-400, 0, i), Vector3(400, 0, i), 40, 40, 50, 1);
        }

        // Robot
        auto skeleton = arm.getSkeleton(q);
        for(size_t i=0; i<skeleton.size()-1; ++i) {
            Color c = link_colors[i % link_colors.size()];
            drawLine3D(ren, skeleton[i], skeleton[i+1], c.r, c.g, c.b, 8 - i);
            drawJoint(ren, skeleton[i], 200, 200, 200);
        }

        // Draw Tip Frame (Blue Z-Axis is the pointer)
        Matrix4 tip_T = arm.forward(q);
        drawFrame(ren, 
            Vector3(tip_T(0,3), tip_T(1,3), tip_T(2,3)), 
            tip_T.block<3,3>(0,0), 
            30.0
        );

        // --- NEW TARGET VISUAL (Single Vector) ---
        // 1. Draw the "Base" of the target vector
        drawJoint(ren, target_pos, 255, 0, 255); 

        // 2. Draw the "Direction" vector (Magenta Line)
        // We use the Z-column of the rotation matrix as the "Pointer" direction
        Vector3 target_dir = target_rot.col(2) * 60.0; // 60px length
        Vector3 tip_point = target_pos + target_dir;
        
        drawLine3D(ren, target_pos, tip_point, 255, 0, 255, 5); // Thick Magenta Line

        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
