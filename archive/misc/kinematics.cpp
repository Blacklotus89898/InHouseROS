#include <SDL2/SDL.h>
#include <iostream>
#include <cmath>

const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;
const double SCALE = 100.0;
const double R = 0.5, L1 = 1.0, L2 = 1.0;

struct Point { double x, y; };

SDL_Point toScreen(Point p) {
    return { static_cast<int>(SCREEN_WIDTH / 2 + p.x * SCALE), 
             static_cast<int>(SCREEN_HEIGHT / 2 - p.y * SCALE) };
}

Point toMath(int x, int y) {
    return { static_cast<double>(x - SCREEN_WIDTH / 2) / SCALE, 
             static_cast<double>(SCREEN_HEIGHT / 2 - y) / SCALE };
}

// Helper to draw a circle (for the ball and end-effector)
void drawCircle(SDL_Renderer* ren, SDL_Point center, int radius) {
    for (int w = 0; w < radius * 2; w++) {
        for (int h = 0; h < radius * 2; h++) {
            int dx = radius - w;
            int dy = radius - h;
            if ((dx*dx + dy*dy) <= (radius * radius)) {
                SDL_RenderDrawPoint(ren, center.x + dx, center.y + dy);
            }
        }
    }
}

Point fkine_ee(double tb, double t1, double t2) {
    double xc = -R * tb;
    double alpha = tb + M_PI/2.0;
    double x1 = xc + R * std::cos(alpha), y1 = R * std::sin(alpha);
    double phi1 = alpha + t1;
    double x2 = x1 + L1 * std::cos(phi1), y2 = y1 + L1 * std::sin(phi1);
    double phi2 = phi1 + t2;
    return { x2 + L2 * std::cos(phi2), y2 + L2 * std::sin(phi2) };
}

// IK Solvers (Jacobian and Fixed)
void solve_jacobian_ik(Point target, double& tb, double& t1, double& t2) {
    double alpha_step = 0.01;
    for (int i = 0; i < 50; ++i) {
        Point curr = fkine_ee(tb, t1, t2);
        double ex = target.x - curr.x, ey = target.y - curr.y;
        double dt = 1e-6;
        Point f_tb = fkine_ee(tb + dt, t1, t2), f_t1 = fkine_ee(tb, t1 + dt, t2), f_t2 = fkine_ee(tb, t1, t2 + dt);
        tb += alpha_step * (((f_tb.x - curr.x)/dt)*ex + ((f_tb.y - curr.y)/dt)*ey);
        t1 += alpha_step * (((f_t1.x - curr.x)/dt)*ex + ((f_t1.y - curr.y)/dt)*ey);
        t2 += alpha_step * (((f_t2.x - curr.x)/dt)*ex + ((f_t2.y - curr.y)/dt)*ey);
    }
}

int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* win = SDL_CreateWindow("1:Fixed IK | 2:Jacobian IK | 3:Manual FK", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH, SCREEN_HEIGHT, 0);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    bool running = true;
    int mode = 1; // 1: Fixed IK, 2: Jacobian IK, 3: Manual FK
    double tb = 0, t1 = 0, t2 = 0;
    Point target = {1.0, 1.0};

    while (running) {
        SDL_Event e;
        const Uint8* state = SDL_GetKeyboardState(NULL);
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
            if (e.type == SDL_KEYDOWN) {
                if (e.key.keysym.sym == SDLK_1) mode = 1;
                if (e.key.keysym.sym == SDLK_2) mode = 2;
                if (e.key.keysym.sym == SDLK_3) mode = 3;
            }
            if (e.type == SDL_MOUSEMOTION && mode != 3) target = toMath(e.motion.x, e.motion.y);
        }

        // Mode Logic
        if (mode == 1) { tb = 0; solve_jacobian_ik(target, tb, t1, t2); } 
        else if (mode == 2) { solve_jacobian_ik(target, tb, t1, t2); }
        else if (mode == 3) {
            if (state[SDL_SCANCODE_Q]) tb += 0.05; if (state[SDL_SCANCODE_W]) tb -= 0.05;
            if (state[SDL_SCANCODE_A]) t1 += 0.05; if (state[SDL_SCANCODE_S]) t1 -= 0.05;
            if (state[SDL_SCANCODE_Z]) t2 += 0.05; if (state[SDL_SCANCODE_X]) t2 -= 0.05;
        }

        SDL_SetRenderDrawColor(ren, 20, 20, 20, 255);
        SDL_RenderClear(ren);

        // Draw Ground
        SDL_SetRenderDrawColor(ren, 80, 80, 80, 255);
        SDL_RenderDrawLine(ren, 0, SCREEN_HEIGHT/2 + R*SCALE, SCREEN_WIDTH, SCREEN_HEIGHT/2 + R*SCALE);

        // Calculate Frame Positions
        double xc = -R * tb;
        double alpha = tb + M_PI/2.0;
        Point p_ball = { xc, 0 };
        Point p_base = { xc + R * std::cos(alpha), R * std::sin(alpha) };
        Point p_j2 = { p_base.x + L1 * std::cos(alpha + t1), p_base.y + L1 * std::sin(alpha + t1) };
        Point p_ee = fkine_ee(tb, t1, t2);

        // Draw Ball (Purple)
        SDL_SetRenderDrawColor(ren, 150, 0, 255, 255);
        drawCircle(ren, toScreen(p_ball), R * SCALE);

        // Draw Links
        SDL_Point sp0 = toScreen(p_base), sp1 = toScreen(p_j2), sp2 = toScreen(p_ee);
        SDL_SetRenderDrawColor(ren, 0, 255, 100, 255);
        SDL_RenderDrawLine(ren, sp0.x, sp0.y, sp1.x, sp1.y);
        SDL_SetRenderDrawColor(ren, 0, 150, 255, 255);
        SDL_RenderDrawLine(ren, sp1.x, sp1.y, sp2.x, sp2.y);
        
        // Draw End-Effector (Orange)
        SDL_SetRenderDrawColor(ren, 255, 165, 0, 255);
        drawCircle(ren, sp2, 8);

        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}

// TODO: g++ kinematics.cpp -o kinematics -lSDL2

