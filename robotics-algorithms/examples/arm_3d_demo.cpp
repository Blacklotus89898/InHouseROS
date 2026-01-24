#include <SDL2/SDL.h>
#include <iostream>
#include <cmath>
#include "robotics_algo/kinematics/arm_3d.hpp"

using namespace robotics;
using namespace robotics::kinematics;

// --- RENDERING CONFIG ---
struct Point2D { int x, y; double scale; };
const double CAM_DIST = 800.0;
const double CENTER_X = 400.0;
const double CENTER_Y = 450.0; // Moved camera up slightly
const double VIEW_ANGLE_X = 0.5; // Pitch (Looking down)
const double VIEW_ANGLE_Y = 0.0; // Yaw (Rotation around vertical)

// 1. Project 3D Point -> 2D Screen
Point2D project(Vector3 p) {
    // A simple fixed camera transform (Iso-ish view)
    // Rotate world around X axis to look down
    double rx = p.x();
    double ry = p.y() * std::cos(VIEW_ANGLE_X) - p.z() * std::sin(VIEW_ANGLE_X);
    double rz = p.z() * std::cos(VIEW_ANGLE_X) + p.y() * std::sin(VIEW_ANGLE_X);

    // Add perspective
    // Move world "into" screen so we don't clip at z=0
    double z_offset = 400.0; 
    double depth = rz + z_offset;
    
    if (depth < 1.0) depth = 1.0; // Safety

    double factor = CAM_DIST / depth;

    return {
        (int)(CENTER_X + rx * factor),
        (int)(CENTER_Y - ry * factor), // SDL Y is down, Math Y is up
        factor
    };
}

// 2. Ray-Plane Intersection (The Fix for Mouse Clicking)
// Casts a ray from camera through screen pixel (sx, sy)
// Returns the point where it hits the plane Y = target_height
Vector3 getFloorIntersection(int sx, int sy, double target_height) {
    // This is an approximation of the inverse projection 
    // tailored specifically for our specific camera setup.
    
    // Reverse the screen offset
    double rx_screen = sx - CENTER_X;
    double ry_screen = CENTER_Y - sy;

    // We solve for Z assuming we know Y = target_height.
    // Logic derived from the project() function above.
    
    // We iterate to find Z because perspective makes the algebra messy.
    // (A simple binary search is fast and robust enough for a demo)
    double min_z = -1000.0, max_z = 1000.0;
    Vector3 best_p(0, target_height, 0);
    
    for(int i=0; i<20; ++i) {
        double z_guess = (min_z + max_z) / 2.0;
        
        // Project a test point on our plane at this Z
        // We need X too...
        // Let's use the property that X scales linearly with Z in our projection
        // factor = CAM_DIST / (z_guess_rotated + offset)
        
        double z_offset = 400.0;
        double rz = z_guess * std::cos(VIEW_ANGLE_X) + target_height * std::sin(VIEW_ANGLE_X);
        double depth = rz + z_offset;
        double factor = CAM_DIST / depth;
        
        // Calculate where this Z maps to on screen Y
        double ry_world = target_height * std::cos(VIEW_ANGLE_X) - z_guess * std::sin(VIEW_ANGLE_X);
        double screen_y_guess = ry_world * factor;

        // Binary Search Z to match Mouse Y
        if (screen_y_guess > ry_screen) {
             min_z = z_guess; // Point is "too high" visually, implies Z is too far back (in this rotated frame)
        } else {
             max_z = z_guess;
        }
        
        // Once Z is known, X is easy
        double x_world = rx_screen / factor;
        best_p = Vector3(x_world, target_height, z_guess);
    }
    
    return best_p;
}

void drawLine3D(SDL_Renderer* ren, Vector3 p1, Vector3 p2, int r, int g, int b, int base_thick=2) {
    Point2D s1 = project(p1); Point2D s2 = project(p2);
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
    SDL_SetRenderDrawColor(ren, r, g, b, 255);
    int size = size_base * s.scale;
    SDL_Rect rect = {s.x - size/2, s.y - size/2, size, size};
    SDL_RenderFillRect(ren, &rect);
}

int main() {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* win = SDL_CreateWindow("RoboArm 3D (Mouse = Move on Floor)", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 600, SDL_WINDOW_SHOWN);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    // Arm setup: Base Height=50, Humerus=150, Forearm=150
    Arm3D arm(50.0, 150.0, 150.0);
    Arm3D::Joints q = {0.0, 1.0, -1.0};
    
    // Start target on the floor (Y=0) in front of the robot (Z=150)
    Vector3 current_target(0, 0, 150); 
    Vector3 final_goal(0, 0, 150);     
    Vector3 start_pos(0, 0, 150);      
    double t = 1.0;                      
    bool ik_mode = true;
    bool running = true;

    while(running) {
        SDL_Event e;
        while(SDL_PollEvent(&e)) {
            if(e.type == SDL_QUIT) running = false;
            if(e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_SPACE) {
                ik_mode = !ik_mode;
                if(ik_mode) {
                     current_target = arm.forward(q.q0, q.q1, q.q2);
                     final_goal = current_target;
                     start_pos = current_target;
                     t = 1.0;
                }
            }
            if(ik_mode && e.type == SDL_MOUSEBUTTONDOWN) {
                start_pos = current_target;
                // FIX: Click maps to Plane Y=0 (Floor)
                // If you want "Desk Height", change 0.0 to 50.0
                final_goal = getFloorIntersection(e.button.x, e.button.y, 0.0);
                t = 0.0;
            }
        }

        const Uint8* keys = SDL_GetKeyboardState(NULL);

        if(ik_mode) {
            // Nudge height with W/S if needed
            if(keys[SDL_SCANCODE_W]) { final_goal.y() += 2.0; if(t>=1.0) start_pos.y() = final_goal.y(); }
            if(keys[SDL_SCANCODE_S]) { final_goal.y() -= 2.0; if(t>=1.0) start_pos.y() = final_goal.y(); }
            
            // Interpolate
            if(t < 1.0) {
                t += 0.05; if(t>1.0) t=1.0;
                double smooth = t * (2-t);
                current_target = start_pos + (final_goal - start_pos) * smooth;
            } else {
                current_target = final_goal;
            }
            q = arm.solveIK(q, current_target);
        } else {
            if(keys[SDL_SCANCODE_LEFT])  q.q0 += 0.04;
            if(keys[SDL_SCANCODE_RIGHT]) q.q0 -= 0.04;
            if(keys[SDL_SCANCODE_UP])    q.q1 += 0.04;
            if(keys[SDL_SCANCODE_DOWN])  q.q1 -= 0.04;
            if(keys[SDL_SCANCODE_W])     q.q2 += 0.04;
            if(keys[SDL_SCANCODE_S])     q.q2 -= 0.04;
        }

        SDL_SetRenderDrawColor(ren, 15, 15, 20, 255);
        SDL_RenderClear(ren);

        // 1. Draw Floor Grid (Y=0)
        for(int i=-400; i<=400; i+=50) {
            drawLine3D(ren, Vector3(i, 0, -400), Vector3(i, 0, 400), 50, 50, 60, 1);
            drawLine3D(ren, Vector3(-400, 0, i), Vector3(400, 0, i), 50, 50, 60, 1);
        }

        auto skel = arm.getSkeleton(q.q0, q.q1, q.q2);

        // 2. Draw Bones
        drawLine3D(ren, skel.p_base, skel.p_shoulder, 100, 255, 100, 4);
        drawLine3D(ren, skel.p_shoulder, skel.p_elbow, 100, 100, 255, 3);
        drawLine3D(ren, skel.p_elbow, skel.p_wrist, 255, 100, 100, 2);

        // 3. Draw Joints
        drawPoint3D(ren, skel.p_base, 200, 200, 200, 12);
        drawPoint3D(ren, skel.p_shoulder, 200, 200, 200, 10);
        drawPoint3D(ren, skel.p_elbow, 200, 200, 200, 8);
        drawPoint3D(ren, skel.p_wrist, 255, 255, 0, 6); 

        // 4. Draw Targets
        if(ik_mode) {
            drawPoint3D(ren, final_goal, 0, 255, 255, 8); // Goal
            drawLine3D(ren, final_goal, Vector3(final_goal.x(), 0, final_goal.z()), 0, 255, 255, 1); // Shadow Line
        }

        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
