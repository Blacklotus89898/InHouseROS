#include <SDL2/SDL.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include "robotics_algo/perception/occupancy_grid.hpp"

using namespace robotics;
using namespace robotics::perception;

// --- MATH HELPERS ---

// Standard Line-Segment Intersection
// Ray: Start + Dir * t (t > 0)
// Wall: P1 + (P2 - P1) * u (0 <= u <= 1)
double getRayIntersection(Vector2 ray_start, Vector2 ray_dir, Vector2 p1, Vector2 p2) {
    Vector2 v1 = ray_start - p1;
    Vector2 v2 = p2 - p1;
    Vector2 v3 = Vector2(-ray_dir.y(), ray_dir.x()); // Perpendicular to ray

    double dot = v2.dot(v3);
    
    // Parallel lines check
    if (std::abs(dot) < 1e-6) return -1.0;

    double t1 = (v2.x() * v1.y() - v2.y() * v1.x()) / dot; // Cross product logic
    double t2 = (v1.x() * ray_dir.y() - v1.y() * ray_dir.x()) / dot;

    // t1 is 'u' (along the wall segment), t2 is 't' (distance along ray)
    // We need 0 <= u <= 1 (hit is within wall endpoints)
    // We need t > 0 (hit is in front of robot)
    if (t1 >= 0.0 && t1 <= 1.0 && t2 > 0.0) {
        return t2; // Return distance
    }

    return -1.0; // No hit
}

// Cast a single ray against all walls and return the closest hit point
Vector2 castRay(Vector2 start, double angle, const std::vector<std::pair<Vector2, Vector2>>& walls) {
    double max_range = 300.0;
    Vector2 dir(std::cos(angle), std::sin(angle));
    
    double closest_dist = max_range;
    Vector2 best_hit = start + dir * max_range; // Default to max range

    for (const auto& wall : walls) {
        double dist = getRayIntersection(start, dir, wall.first, wall.second);
        if (dist > 0.0 && dist < closest_dist) {
            closest_dist = dist;
            best_hit = start + dir * dist;
        }
    }
    return best_hit;
}

int main() {
    // 1. Init SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) return 1;
    SDL_Window* win = SDL_CreateWindow("Occupancy Grid (Arrows=Drive, Space=Reset)", 
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 600, SDL_WINDOW_SHOWN);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    // 2. Setup World (A Maze Layout)
    std::vector<std::pair<Vector2, Vector2>> walls;
    auto addWall = [&](double x1, double y1, double x2, double y2) {
        walls.push_back({{x1, y1}, {x2, y2}});
    };

    // Border
    addWall(50, 50, 750, 50);
    addWall(750, 50, 750, 550);
    addWall(750, 550, 50, 550);
    addWall(50, 550, 50, 50);

    // Obstacles
    addWall(200, 50, 200, 400); // Long vertical wall
    addWall(400, 200, 400, 550); // Another vertical
    addWall(550, 100, 650, 100); // Box top
    addWall(650, 100, 650, 200); // Box right
    addWall(650, 200, 550, 200); // Box bot
    addWall(550, 200, 550, 100); // Box left

    // 3. Setup Robot
    Pose2D robot(100, 300, 0); // Start in open area
    double resolution = 5.0; // 5cm/pixel
    OccupancyGrid grid(800/5, 600/5, resolution);

    bool running = true;
    while(running) {
        SDL_Event e;
        while(SDL_PollEvent(&e)) {
            if(e.type == SDL_QUIT) running = false;
            // Reset Map
            if(e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_SPACE) {
                grid = OccupancyGrid(800/5, 600/5, resolution);
            }
        }

        // --- CONTROLS (Tank Drive) ---
        const Uint8* keys = SDL_GetKeyboardState(NULL);
        if(keys[SDL_SCANCODE_LEFT]) robot.theta -= 0.05;
        if(keys[SDL_SCANCODE_RIGHT]) robot.theta += 0.05;
        
        double speed = 0.0;
        if(keys[SDL_SCANCODE_UP]) speed = 3.0;
        if(keys[SDL_SCANCODE_DOWN]) speed = -3.0;

        if (speed != 0.0) {
            robot.position.x() += std::cos(robot.theta) * speed;
            robot.position.y() += std::sin(robot.theta) * speed;
        }

        // --- SENSE (Lidar) ---
        // 120 Rays (High resolution scan)
        std::vector<Vector2> scan;
        int num_rays = 120;
        double fov = 360.0 * (M_PI / 180.0); // Full 360 view
        
        for(int i=0; i<num_rays; ++i) {
            double angle_offset = fov * (double(i)/num_rays) - (fov/2.0);
            double ray_angle = robot.theta + angle_offset;
            scan.push_back(castRay(robot.position, ray_angle, walls));
        }

        // --- MAPPING (Update Grid) ---
        grid.update(robot, scan);

        // --- RENDER ---
        SDL_SetRenderDrawColor(ren, 40, 40, 40, 255); // Dark Background
        SDL_RenderClear(ren);

        // 1. Draw Grid (Visualization of Probability)
        const auto& data = grid.getData();
        int w = grid.getWidth();
        int h = grid.getHeight();
        
        for(int y=0; y<h; ++y) {
            for(int x=0; x<w; ++x) {
                double log_odds = data[y * w + x];
                
                // Skip rendering if it's basically unknown (save FPS)
                if(std::abs(log_odds) < 0.1) continue; 

                // Convert LogOdds -> Probability -> Color
                // Probability p = 1 / (1 + exp(-log_odds))
                // High LogOdds (>0) -> Occupied -> Black (0)
                // Low LogOdds (<0) -> Free -> White (255)
                
                double p = 1.0 - (1.0 / (1.0 + std::exp(log_odds)));
                
                // Color mapping: 0.0 (Free) -> 255 (White), 1.0 (Occ) -> 0 (Black)
                int color = static_cast<int>((1.0 - p) * 255);
                if (color < 0) color = 0;
                if (color > 255) color = 255;

                SDL_SetRenderDrawColor(ren, color, color, color, 255);
                SDL_Rect r = {(int)(x*resolution), (int)(y*resolution), (int)resolution, (int)resolution};
                SDL_RenderFillRect(ren, &r);
            }
        }

        // 2. Draw Robot
        SDL_SetRenderDrawColor(ren, 255, 50, 50, 255); // Red
        SDL_Rect r = {(int)robot.position.x()-6, (int)robot.position.y()-6, 12, 12};
        SDL_RenderFillRect(ren, &r);
        
        // Robot Heading Indicator
        int nose_x = robot.position.x() + std::cos(robot.theta) * 15;
        int nose_y = robot.position.y() + std::sin(robot.theta) * 15;
        SDL_RenderDrawLine(ren, (int)robot.position.x(), (int)robot.position.y(), nose_x, nose_y);

        // 3. Draw Laser Rays (Faint Green)
        SDL_SetRenderDrawColor(ren, 0, 255, 0, 40); // Very transparent
        for(auto& p : scan) {
            SDL_RenderDrawLine(ren, (int)robot.position.x(), (int)robot.position.y(), (int)p.x(), (int)p.y());
        }

        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
