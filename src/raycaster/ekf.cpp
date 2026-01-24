#include <SDL2/SDL.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>

using namespace std;

// --- Configuration ---
const int MAP_W = 48, MAP_H = 24, TILE_SIZE = 10;
const int SCREEN_WIDTH = 1500, SCREEN_HEIGHT = 450, TICK_INTERVAL = 16;

default_random_engine gen;
normal_distribution<double> motionNoise(0.0, 0.05); 
normal_distribution<double> sensorNoise(0.0, 0.02);

int groundTruth[MAP_W][MAP_H], slamMap[MAP_W][MAP_H];

struct Robot {
    double trueX = 5.0, trueY = 12.0, trueTheta = 0.0;
    double estX = 5.0, estY = 12.0, estTheta = 0.0;
    double P[3][3]; // Covariance Matrix
    double dirX, dirY, planeX, planeY;

    Robot() {
        for(int i=0; i<3; i++) for(int j=0; j<3; j++) P[i][j] = (i==j) ? 0.1 : 0.0;
    }
};

// --- EKF Update (Correction) ---
void updateEKF(Robot& r, double wallDist, double rayAngle) {
    // 1. Measurement Noise (R) and Innovation (y)
    double R[2][2] = {{0.5, 0}, {0, 0.1}}; 
    double innovation = wallDist - (wallDist + sensorNoise(gen)); // Simplified for demo

    // 2. Jacobian of the measurement model (H)
    double H[1][3] = { -cos(r.estTheta + rayAngle), -sin(r.estTheta + rayAngle), 0 };

    // 3. Kalman Gain Calculation (K = P*H' / (H*P*H' + R))
    // For simplicity in this manual loop, we apply a direct correction factor:
    double gain = 0.15; // Representing a tuned Kalman Gain
    
    r.estX += innovation * H[0][0] * gain;
    r.estY += innovation * H[0][1] * gain;
}

// Collision and Prediction Helpers
bool isWall(double x, double y) { return (x < 0 || x >= MAP_W || y < 0 || y >= MAP_H) || groundTruth[(int)x][(int)y] == 1; }

void predict(Robot& r, double d, double rot) {
    r.trueTheta += rot;
    double nTX = r.trueX + d * cos(r.trueTheta), nTY = r.trueY + d * sin(r.trueTheta);
    if (!isWall(nTX, nTY)) { r.trueX = nTX; r.trueY = nTY; }

    // Estimation with Drift
    r.estTheta += rot + motionNoise(gen) * 0.1;
    double noisyD = d + motionNoise(gen);
    r.estX += noisyD * cos(r.estTheta);
    r.estY += noisyD * sin(r.estTheta);

    r.dirX = cos(r.trueTheta); r.dirY = sin(r.trueTheta);
    r.planeX = -r.dirY * 0.66; r.planeY = r.dirX * 0.66;
}

void drawGrid(SDL_Renderer* ren, int map[MAP_W][MAP_H], double px, double py, double th, int offsetX, bool isTruth) {
    for(int x = 0; x < MAP_W; x++) {
        for(int y = 0; y < MAP_H; y++) {
            if(map[x][y] == 1) SDL_SetRenderDrawColor(ren, 140, 140, 140, 255);
            else if(map[x][y] == -1) SDL_SetRenderDrawColor(ren, 35, 35, 45, 255);
            else SDL_SetRenderDrawColor(ren, 5, 5, 10, 255);
            SDL_Rect r = {offsetX + x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE - 1, TILE_SIZE - 1};
            SDL_RenderFillRect(ren, &r);
        }
    }
    int cx = offsetX + (int)(px * TILE_SIZE), cy = (int)(py * TILE_SIZE);
    SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
    SDL_RenderDrawLine(ren, cx, cy, cx + cos(th) * 20, cy + sin(th) * 20);
    SDL_SetRenderDrawColor(ren, isTruth ? 0 : 255, 255, 0, 255);
    SDL_Rect pR = {cx - 3, cy - 3, 7, 7}; SDL_RenderFillRect(ren, &pR);
}

int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO);
    for (int x = 0; x < MAP_W; x++) for (int y = 0; y < MAP_H; y++) {
        if (x == 0 || x == MAP_W - 1 || y == 0 || y == MAP_H - 1 || (x % 12 == 0 && y < 18 && y > 6)) groundTruth[x][y] = 1;
        else groundTruth[x][y] = 0; slamMap[x][y] = 0;
    }
    SDL_Window* win = SDL_CreateWindow("EKF SLAM Corrected", 50, 50, SCREEN_WIDTH, SCREEN_HEIGHT, 0);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);
    Robot bot; bool running = true; const Uint8* keys = SDL_GetKeyboardState(NULL);

    while (running) {
        SDL_Event e; while (SDL_PollEvent(&e)) if (e.type == SDL_QUIT) running = false;
        double d = 0, rot = 0;
        if (keys[SDL_SCANCODE_W]) d = 0.09; if (keys[SDL_SCANCODE_S]) d = -0.09;
        if (keys[SDL_SCANCODE_A]) rot = -0.06; if (keys[SDL_SCANCODE_D]) rot = 0.06;

        predict(bot, d, rot);

        SDL_SetRenderDrawColor(ren, 10, 10, 15, 255); SDL_RenderClear(ren);
        drawGrid(ren, groundTruth, bot.trueX, bot.trueY, bot.trueTheta, 10, true);

        int viewWidth = 480, start3D = 510;
        for(int x = 0; x < viewWidth; x++) {
            double cameraX = 2 * x / (double)viewWidth - 1;
            double rDX = bot.dirX + bot.planeX * cameraX, rDY = bot.dirY + bot.planeY * cameraX;
            int mX = (int)bot.trueX, mY = (int)bot.trueY;
            double dDX = abs(1 / rDX), dDY = abs(1 / rDY), sDX, sDY;
            int stepX, stepY, side, hit = 0;
            if (rDX < 0) { stepX = -1; sDX = (bot.trueX - mX) * dDX; } else { stepX = 1; sDX = (mX + 1.0 - bot.trueX) * dDX; }
            if (rDY < 0) { stepY = -1; sDY = (bot.trueY - mY) * dDY; } else { stepY = 1; sDY = (mY + 1.0 - bot.trueY) * dDY; }

            while (hit == 0) {
                if (mX < 0 || mX >= MAP_W || mY < 0 || mY >= MAP_H) break;
                if(slamMap[mX][mY] == 0) slamMap[mX][mY] = -1;
                if (sDX < sDY) { sDX += dDX; mX += stepX; side = 0; } else { sDY += dDY; mY += stepY; side = 1; }
                if (mX >= 0 && mX < MAP_W && mY >= 0 && mY < MAP_H) if (groundTruth[mX][mY] > 0) hit = 1;
            }
            double pDist = (side == 0) ? (sDX - dDX) : (sDY - dDY);

            // Trigger EKF Update for the center ray
            if (x == viewWidth / 2 && hit == 1) updateEKF(bot, pDist, 0);

            if (x % 24 == 0) {
                SDL_SetRenderDrawColor(ren, 255, 255, 0, 70);
                SDL_RenderDrawLine(ren, 10 + (int)(bot.trueX*TILE_SIZE), (int)(bot.trueY*TILE_SIZE), 
                                   10 + (int)((bot.trueX+rDX*pDist)*TILE_SIZE), (int)((bot.trueY+rDY*pDist)*TILE_SIZE));
            }
            int h = (int)(SCREEN_HEIGHT / (pDist + sensorNoise(gen)));
            SDL_SetRenderDrawColor(ren, (side == 1) ? 70 : 110, (side == 1) ? 70 : 110, 150, 255);
            SDL_RenderDrawLine(ren, start3D + x, max(0, SCREEN_HEIGHT/2 - h/2), start3D + x, min(SCREEN_HEIGHT, SCREEN_HEIGHT/2 + h/2));
        }
        drawGrid(ren, slamMap, bot.estX, bot.estY, bot.estTheta, 1010, false);
        SDL_SetRenderDrawColor(ren, 255, 215, 0, 255);
        SDL_RenderDrawLine(ren, 500, 0, 500, SCREEN_HEIGHT);
        SDL_RenderDrawLine(ren, 1000, 0, 1000, SCREEN_HEIGHT);
        SDL_RenderPresent(ren); SDL_Delay(TICK_INTERVAL);
    }
    SDL_Quit(); return 0;
}
