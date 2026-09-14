#include <SDL2/SDL.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>

using namespace std;

const int SCREEN_WIDTH = 1200; 
const int SCREEN_HEIGHT = 400;
const int MAP_SIZE = 24;
const int TILE_SIZE = 15;
const int TICK_INTERVAL = 16;

// --- Ground Truth ---
int groundTruth[MAP_SIZE][MAP_SIZE] = {
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,1,1,0,0,0,0,1,1,1,1,1,0,0,0,0,2,2,2,0,0,1},
    {1,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,2,0,2,0,0,1},
    {1,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,2,0,2,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,2,2,0,0,1},
    {1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

int slamMap[MAP_SIZE][MAP_SIZE];

struct Player {
    double posX = 2.0, posY = 2.0; 
    double dirX = 1.0, dirY = 0.0;
    double planeX = 0.0, planeY = 0.66;
    double distL = 0, distC = 0, distR = 0;
};

// --- Helper Functions ---
void rotate(Player& p, double rotSpeed) {
    double oldDirX = p.dirX, oldPlaneX = p.planeX;
    p.dirX = p.dirX * cos(rotSpeed) - p.dirY * sin(rotSpeed);
    p.dirY = oldDirX * sin(rotSpeed) + p.dirY * cos(rotSpeed);
    p.planeX = p.planeX * cos(rotSpeed) - p.planeY * sin(rotSpeed);
    p.planeY = oldPlaneX * sin(rotSpeed) + p.planeY * cos(rotSpeed);
}

void move(Player& p, double moveSpeed) {
    if(!groundTruth[int(p.posX + p.dirX * moveSpeed)][int(p.posY)]) p.posX += p.dirX * moveSpeed;
    if(!groundTruth[int(p.posX)][int(p.posY + p.dirY * moveSpeed)]) p.posY += p.dirY * moveSpeed;
}

// BFS to find the nearest reachable unknown tile
double findReachableFrontierAngle(Player& p) {
    queue<pair<int, int>> q;
    vector<vector<bool>> visited(MAP_SIZE, vector<bool>(MAP_SIZE, false));
    vector<vector<pair<int, int>>> parent(MAP_SIZE, vector<pair<int, int>>(MAP_SIZE, {-1, -1}));

    int startX = (int)p.posX;
    int startY = (int)p.posY;
    q.push({startX, startY});
    visited[startX][startY] = true;

    int dx[] = {0, 0, 1, -1};
    int dy[] = {1, -1, 0, 0};

    while (!q.empty()) {
        pair<int, int> curr = q.front();
        q.pop();

        for (int i = 0; i < 4; i++) {
            int nx = curr.first + dx[i];
            int ny = curr.second + dy[i];

            if (nx >= 0 && nx < MAP_SIZE && ny >= 0 && ny < MAP_SIZE && !visited[nx][ny]) {
                if (slamMap[nx][ny] == 0) { // Found a frontier!
                    // Calculate angle to the first step of the path
                    double angleToTarget = atan2(ny + 0.5 - p.posY, nx + 0.5 - p.posX);
                    double botAngle = atan2(p.dirY, p.dirX);
                    double diff = angleToTarget - botAngle;
                    while (diff > M_PI) diff -= 2 * M_PI;
                    while (diff < -M_PI) diff += 2 * M_PI;
                    return diff;
                }
                if (slamMap[nx][ny] == -1) { // Only travel through explored empty space
                    visited[nx][ny] = true;
                    q.push({nx, ny});
                }
            }
        }
    }
    return 0;
}

void drawGrid(SDL_Renderer* ren, int map[MAP_SIZE][MAP_SIZE], double px, double py, int offsetX) {
    for(int x = 0; x < MAP_SIZE; x++) {
        for(int y = 0; y < MAP_SIZE; y++) {
            if(map[x][y] == 1) SDL_SetRenderDrawColor(ren, 180, 180, 180, 255);
            else if(map[x][y] == 2) SDL_SetRenderDrawColor(ren, 0, 180, 0, 255);
            else if(map[x][y] == -1) SDL_SetRenderDrawColor(ren, 40, 40, 40, 255);
            else SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
            SDL_Rect r = {offsetX + x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE-1, TILE_SIZE-1};
            SDL_RenderFillRect(ren, &r);
        }
    }
    SDL_SetRenderDrawColor(ren, 255, 255, 0, 255);
    SDL_Rect pRect = {offsetX + (int)(px * TILE_SIZE)-2, (int)(py * TILE_SIZE)-2, 4, 4};
    SDL_RenderFillRect(ren, &pRect);
}

int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* win = SDL_CreateWindow("SLAM: BFS Frontier Logic (Press M)", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH, SCREEN_HEIGHT, 0);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    for(int i = 0; i < MAP_SIZE; i++) for(int j = 0; j < MAP_SIZE; j++) slamMap[i][j] = 0;

    Player p;
    bool running = true, botMode = false;
    const Uint8* keys = SDL_GetKeyboardState(NULL);

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) { 
            if (e.type == SDL_QUIT) running = false; 
            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_m) botMode = !botMode;
        }

        double moveSpeed = 0.08, rotSpeed = 0.07;

        if (botMode) {
            double angleToGoal = findReachableFrontierAngle(p);

            // 1. Reactive Obstacle Avoidance (Priority)
            if (p.distC < 1.0) { 
                if (p.distL > p.distR) rotate(p, -rotSpeed);
                else rotate(p, rotSpeed);
            } 
            // 2. BFS Path Following
            else {
                if (abs(angleToGoal) > 0.1) rotate(p, (angleToGoal > 0 ? 1 : -1) * rotSpeed);
                move(p, moveSpeed);
            }
        } else {
            if (keys[SDL_SCANCODE_W]) move(p, moveSpeed);
            if (keys[SDL_SCANCODE_S]) move(p, -moveSpeed);
            if (keys[SDL_SCANCODE_A]) rotate(p, -rotSpeed);
            if (keys[SDL_SCANCODE_D]) rotate(p, rotSpeed);
        }

        SDL_SetRenderDrawColor(ren, 15, 15, 15, 255);
        SDL_RenderClear(ren);
        drawGrid(ren, groundTruth, p.posX, p.posY, 0); 
        drawGrid(ren, slamMap, p.posX, p.posY, 840);

        int viewWidth = 400, start3D = 400;

        for(int x = 0; x < viewWidth; x++) {
            double cameraX = 2 * x / (double)viewWidth - 1;
            double rDX = p.dirX + p.planeX * cameraX;
            double rDY = p.dirY + p.planeY * cameraX;

            int mX = (int)p.posX, mY = (int)p.posY;
            double dDX = (rDX == 0) ? 1e30 : abs(1 / rDX);
            double dDY = (rDY == 0) ? 1e30 : abs(1 / rDY);
            double sDX, sDY;
            int stepX, stepY, side, hit = 0;

            if (rDX < 0) { stepX = -1; sDX = (p.posX - mX) * dDX; }
            else { stepX = 1; sDX = (mX + 1.0 - p.posX) * dDX; }
            if (rDY < 0) { stepY = -1; sDY = (p.posY - mY) * dDY; }
            else { stepY = 1; sDY = (mY + 1.0 - p.posY) * dDY; }

            while (hit == 0) {
                if(slamMap[mX][mY] == 0) slamMap[mX][mY] = -1; 
                if (sDX < sDY) { sDX += dDX; mX += stepX; side = 0; }
                else { sDY += dDY; mY += stepY; side = 1; }
                if (groundTruth[mX][mY] > 0) { hit = 1; slamMap[mX][mY] = groundTruth[mX][mY]; }
            }

            double pDist = (side == 0) ? (sDX - dDX) : (sDY - dDY);
            if (x == 0) p.distL = pDist;
            if (x == viewWidth / 2) p.distC = pDist;
            if (x == viewWidth - 1) p.distR = pDist;

            int h = (int)(SCREEN_HEIGHT / pDist);
            int start = max(0, -h / 2 + SCREEN_HEIGHT / 2);
            int end = min(SCREEN_HEIGHT - 1, h / 2 + SCREEN_HEIGHT / 2);

            if (x % 15 == 0) {
                SDL_SetRenderDrawColor(ren, botMode ? 0 : 255, 255, 0, 100);
                SDL_RenderDrawLine(ren, (int)(p.posX * TILE_SIZE), (int)(p.posY * TILE_SIZE), (int)((p.posX + rDX * pDist) * TILE_SIZE), (int)((p.posY + rDY * pDist) * TILE_SIZE));
            }
            SDL_SetRenderDrawColor(ren, (side == 1) ? 90 : 160, (side == 1) ? 90 : 160, (side == 1) ? 90 : 160, 255);
            SDL_RenderDrawLine(ren, start3D + x, start, start3D + x, end);
        }

        SDL_SetRenderDrawColor(ren, 255, 255, 0, 255);
        SDL_RenderDrawLine(ren, 400, 0, 400, SCREEN_HEIGHT);
        SDL_RenderDrawLine(ren, 800, 0, 800, SCREEN_HEIGHT);
        SDL_RenderPresent(ren);
        SDL_Delay(TICK_INTERVAL);
    }
    SDL_Quit();
    return 0;
}

