#include <SDL2/SDL.h>
#include <iostream>
#include <cmath>
#include "robotics_algo/decision/fsm/state_machine.hpp"

using namespace robotics::decision;

// --- SHARED CONTEXT ---
// In a real app, pass this pointer to states
struct RobotContext {
    double x = 100, y = 300;
    double target_x = 0, target_y = 0;
    int click_count = 0;
};
RobotContext ctx;

// --- STATES ---

class PatrolState : public State {
    int waypoints[2] = {100, 700};
    int current_idx = 0;
public:
    void onEnter() override {
        // Pick initial target
        ctx.target_x = waypoints[current_idx];
        ctx.target_y = 300;
    }

    std::string update(double dt) override {
        // 1. Move
        double dir = (ctx.target_x > ctx.x) ? 1.0 : -1.0;
        ctx.x += dir * 200 * dt; // Speed 200 px/s

        // 2. Check Waypoint Reached
        if (std::abs(ctx.x - ctx.target_x) < 5.0) {
            current_idx = (current_idx + 1) % 2; // Flip 0 <-> 1
            ctx.target_x = waypoints[current_idx];
        }

        // 3. Transition Logic
        if (ctx.click_count > 0) return "Alert";
        
        return ""; // Stay
    }
};

class AlertState : public State {
    double timer = 0.0;
public:
    void onEnter() override {
        timer = 0.0;
    }

    std::string update(double dt) override {
        timer += dt;
        
        // Wait for 2 seconds
        if (timer > 2.0) {
            ctx.click_count = 0; // Reset
            return "Patrol";     // Bored, go back
        }

        // If user clicks AGAIN while alerted, get angry
        if (ctx.click_count >= 3) return "Chase";

        return "";
    }
};

class ChaseState : public State {
public:
    std::string update(double dt) override {
        // Move towards mouse (target updated by main loop)
        double dx = ctx.target_x - ctx.x;
        double dy = ctx.target_y - ctx.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        
        if (dist > 5.0) {
            ctx.x += (dx/dist) * 400 * dt; // Fast speed!
            ctx.y += (dy/dist) * 400 * dt;
        }

        // Calm down if mouse is far away or after time (Simplified: click resets)
        if (ctx.click_count == 0) return "Patrol";

        return "";
    }
};

// --- MAIN LOOP ---

int main() {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* win = SDL_CreateWindow("FSM: Patrol(G) -> Alert(Y) -> Chase(R)", 100, 100, 800, 600, SDL_WINDOW_SHOWN);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    // Setup FSM
    StateMachine fsm;
    fsm.addState("Patrol", std::make_shared<PatrolState>());
    fsm.addState("Alert", std::make_shared<AlertState>());
    fsm.addState("Chase", std::make_shared<ChaseState>());
    
    fsm.changeState("Patrol");

    bool running = true;
    Uint32 last_time = SDL_GetTicks();

    while(running) {
        Uint32 now = SDL_GetTicks();
        double dt = (now - last_time) / 1000.0;
        last_time = now;

        SDL_Event e;
        while(SDL_PollEvent(&e)) {
            if(e.type == SDL_QUIT) running = false;
            if(e.type == SDL_MOUSEBUTTONDOWN) {
                ctx.click_count++;
                
                // Update target for Chase mode
                int mx, my;
                SDL_GetMouseState(&mx, &my);
                if (fsm.getCurrentStateName() == "Chase") {
                    ctx.target_x = mx;
                    ctx.target_y = my;
                }
            }
            if(e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_SPACE) {
                ctx.click_count = 0; // Force calm down
            }
        }

        // Run FSM Logic
        fsm.update(dt);

        // Render
        SDL_SetRenderDrawColor(ren, 20, 20, 20, 255);
        SDL_RenderClear(ren);

        // Color based on State
        int r=0, g=0, b=0;
        std::string s = fsm.getCurrentStateName();
        if(s == "Patrol") { g = 255; }
        else if(s == "Alert") { r = 255; g = 255; } // Yellow
        else if(s == "Chase") { r = 255; } // Red

        // Draw Robot
        SDL_SetRenderDrawColor(ren, r, g, b, 255);
        SDL_Rect rect = {(int)ctx.x - 20, (int)ctx.y - 20, 40, 40};
        SDL_RenderFillRect(ren, &rect);

        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
