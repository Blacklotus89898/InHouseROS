#include <SDL2/SDL.h>
#include <iostream>
#include <vector>
#include <cmath>
#include "robotics_algo/perception/voxel_grid.hpp"

using namespace robotics;
using namespace robotics::perception;

// --- 3D MATH HELPERS ---
Vector2 project(Vector3 p, double yaw, int screen_w, int screen_h) {
    double cz = std::cos(yaw);
    double sz = std::sin(yaw);
    
    // Rotate World Point relative to Camera
    double rx = p.x() * cz - p.z() * sz;
    double rz = p.x() * sz + p.z() * cz;
    double ry = p.y(); 

    // Translate FWD
    rz += 1.0; 

    // Clip
    if (rz <= 1.0) return Vector2(-9999, -9999); 
    
    // Perspective Projection
    double fov = 500.0;
    double sx = (rx / rz) * fov + (screen_w / 2);
    double sy = (screen_h / 2) - (ry / rz) * fov;

    return Vector2(sx, sy);
}

int main() {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* win = SDL_CreateWindow("Voxel Explorer (Fixed)", 
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 600, SDL_WINDOW_SHOWN);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    // 1. Setup Grid
    double res = 5.0; 
    int MAX_X = 100;
    int MAX_Y = 50;
    int MAX_Z = 100;
    VoxelGrid grid(MAX_X, MAX_Y, MAX_Z, res);

    // 2. Setup Robot
    Pose3D robot;
    robot.position = Vector3(50.0, 20.0, 50.0);
    double camera_yaw = 0.0;
    double move_speed = 3.0;

    // 3. Generate World
    std::vector<Vector3> world_points;
    auto addBlock = [&](int x, int y, int z) {
        if(x>=0 && x<MAX_X*res && y>=0 && y<MAX_Y*res && z>=0 && z<MAX_Z*res)
            world_points.push_back(Vector3(x, y, z));
    };

    // A. Checkerboard Floor
    for(int x = 0; x < MAX_X*res; x += 10) {
        for(int z = 0; z < MAX_Z*res; z += 10) {
            if ((x/10 + z/10) % 2 == 0) addBlock(x, 0, z);
        }
    }

    // B. The Great Pyramid
    int center_x = MAX_X*res / 2;
    int center_z = MAX_Z*res / 2;
    for(int y=0; y<150; y+=10) {
        int width = 150 - y;
        for(int x = center_x - width; x <= center_x + width; x+=10) {
            for(int z = center_z - width; z <= center_z + width; z+=10) {
                if(x == center_x - width || x == center_x + width || 
                   z == center_z - width || z == center_z + width)
                    addBlock(x, y, z);
            }
        }
    }

    // C. The Tunnel
    for(int z=0; z<MAX_Z*res; z+=10) {
        addBlock(50, 0, z);    
        addBlock(50, 40, z);   
        addBlock(150, 0, z);   
        addBlock(150, 40, z);  
        addBlock(100, 60, z);  
    }

    bool running = true;
    while(running) {
        SDL_Event e;
        while(SDL_PollEvent(&e)) {
            if(e.type == SDL_QUIT) running = false;
        }

        const Uint8* keys = SDL_GetKeyboardState(NULL);
        
        // Rotation
        if(keys[SDL_SCANCODE_LEFT]) camera_yaw -= 0.05;
        if(keys[SDL_SCANCODE_RIGHT]) camera_yaw += 0.05;

        // Vectors
        double fwd_x = std::sin(camera_yaw); 
        double fwd_z = std::cos(camera_yaw);
        double right_x = std::cos(camera_yaw);
        double right_z = -std::sin(camera_yaw);

        // Movement
        if(keys[SDL_SCANCODE_W]) { robot.position.x() += fwd_x * move_speed; robot.position.z() += fwd_z * move_speed; }
        if(keys[SDL_SCANCODE_S]) { robot.position.x() -= fwd_x * move_speed; robot.position.z() -= fwd_z * move_speed; }
        if(keys[SDL_SCANCODE_D]) { robot.position.x() += right_x * move_speed; robot.position.z() += right_z * move_speed; }
        if(keys[SDL_SCANCODE_A]) { robot.position.x() -= right_x * move_speed; robot.position.z() -= right_z * move_speed; }
        if(keys[SDL_SCANCODE_Q]) robot.position.y() += move_speed;
        if(keys[SDL_SCANCODE_E]) robot.position.y() -= move_speed;

        // Sense
        std::vector<Vector3> hits;
        for(const auto& p : world_points) {
            double dist_sq = (p - robot.position).squaredNorm();
            if(dist_sq < 250.0 * 250.0) hits.push_back(p);
        }
        grid.update(robot, hits);

        // Render
        SDL_SetRenderDrawColor(ren, 10, 10, 20, 255);
        SDL_RenderClear(ren);

        for(int z=0; z<MAX_Z; ++z) {
            for(int y=0; y<MAX_Y; ++y) {
                for(int x=0; x<MAX_X; ++x) {
                    if(grid.isOccupied(x, y, z)) {
                        Vector3 v_pos(x*res, y*res, z*res);
                        Vector3 relative = v_pos - robot.position;
                        
                        if (relative.squaredNorm() > 400.0*400.0) continue;

                        Vector2 screen_pos = project(relative, camera_yaw, 800, 600);

                        if (screen_pos.x() > -20 && screen_pos.x() < 820 && 
                            screen_pos.y() > -20 && screen_pos.y() < 620) {
                            
                            int brightness = 50 + (y * 5); 
                            if(brightness > 255) brightness = 255;
                            
                            // --- COLOR LOGIC ---
                            int r = 50;
                            int g = (y < 10) ? 150 : 50; 
                            int b = (y > 20) ? 150 + brightness/2 : 50;

                            SDL_SetRenderDrawColor(ren, r, g, b, 255); 
                            
                            // Size
                            double depth = relative.x()*std::sin(camera_yaw) + relative.z()*std::cos(camera_yaw);
                            int size = (int)(3500.0 / (depth + 1.0));
                            if (size < 2) size = 2; 
                            if (size > 20) size = 20;

                            // --- FIX IS HERE: Renamed 'r' to 'rect' ---
                            SDL_Rect rect = {(int)screen_pos.x() - size/2, (int)screen_pos.y() - size/2, size, size};
                            SDL_RenderFillRect(ren, &rect);
                        }
                    }
                }
            }
        }
        
        // HUD
        SDL_SetRenderDrawColor(ren, 255, 255, 255, 150);
        SDL_RenderDrawLine(ren, 390, 300, 410, 300);
        SDL_RenderDrawLine(ren, 400, 290, 400, 310);

        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
