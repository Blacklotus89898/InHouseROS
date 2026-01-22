#include <SDL2/SDL.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>

// --- Constants ---
const int SCREEN_WIDTH = 1024;
const int SCREEN_HEIGHT = 512;
const int TICK_INTERVAL = 16; 

const int MAP_WIDTH = 24;
const int MAP_HEIGHT = 24;
const int TILE_SIZE = 20; 

int worldMap[MAP_WIDTH][MAP_HEIGHT] = {
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,2,2,2,2,2,0,0,0,0,3,0,3,0,3,0,0,0,1},
    {1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,3,0,0,0,3,0,0,0,1},
    {1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,2,2,0,2,2,0,0,0,0,3,0,3,0,3,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,4,4,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,4,0,4,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,4,0,0,0,0,5,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,4,0,4,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,4,0,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,4,4,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

struct Player {
    double posX, posY;      
    double dirX, dirY;      
    double planeX, planeY; 
    
    double posZ;   
    double velZ;   
    int pitch;     
};

void drawMap2D(SDL_Renderer* renderer, const Player& p) {
    for(int x = 0; x < MAP_WIDTH; x++) {
        for(int y = 0; y < MAP_HEIGHT; y++) {
            if(worldMap[x][y] > 0) {
                switch(worldMap[x][y]) {
                    case 1: SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255); break; 
                    case 2: SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255); break;     
                    case 3: SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255); break;     
                    case 4: SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); break;     
                    default: SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255); break;
                }
            } else {
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); 
            }
            SDL_Rect tileRect = {x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE - 1, TILE_SIZE - 1};
            SDL_RenderFillRect(renderer, &tileRect);
        }
    }
    // Draw Player
    SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
    SDL_Rect playerRect = {(int)(p.posX * TILE_SIZE) - 3, (int)(p.posY * TILE_SIZE) - 3, 6, 6};
    SDL_RenderFillRect(renderer, &playerRect);
    
    SDL_RenderDrawLine(renderer, 
        (int)(p.posX * TILE_SIZE), (int)(p.posY * TILE_SIZE), 
        (int)((p.posX + p.dirX) * TILE_SIZE), (int)((p.posY + p.dirY) * TILE_SIZE));
}

int main(int argc, char* argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) return 1;

    SDL_Window* window = SDL_CreateWindow("Fixed Physics: Jump with B", 
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    Player p;
    p.posX = 12; p.posY = 12;
    p.dirX = -1; p.dirY = 0; 
    p.planeX = 0; p.planeY = -0.66; 
    
    p.posZ = 0.5; 
    p.velZ = 0;
    p.pitch = 0;

    bool running = true;
    const Uint8* keys = SDL_GetKeyboardState(NULL);

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
            
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_b) {
                    if (std::abs(p.posZ - 0.5) < 0.01) {
                        p.velZ = 0.15; // Set upward velocity
                    }
                }
            }
        }

        // --- CORRECTED PHYSICS ---
        // 1. Apply velocity immediately
        p.posZ += p.velZ;

        // 2. Apply gravity if above ground
        if (p.posZ > 0.5) {
            p.velZ -= 0.005; // Gravity
        }

        // 3. Collision / Reset
        if (p.posZ < 0.5) {
            p.posZ = 0.5;
            p.velZ = 0;
        }

        double moveSpeed = 0.05;
        double rotSpeed = 0.03;
        int pitchSpeed = 15; 

        if (keys[SDL_SCANCODE_W]) {
            if(worldMap[int(p.posX + p.dirX * moveSpeed)][int(p.posY)] == 0) p.posX += p.dirX * moveSpeed;
            if(worldMap[int(p.posX)][int(p.posY + p.dirY * moveSpeed)] == 0) p.posY += p.dirY * moveSpeed;
        }
        if (keys[SDL_SCANCODE_S]) {
            if(worldMap[int(p.posX - p.dirX * moveSpeed)][int(p.posY)] == 0) p.posX -= p.dirX * moveSpeed;
            if(worldMap[int(p.posX)][int(p.posY - p.dirY * moveSpeed)] == 0) p.posY -= p.dirY * moveSpeed;
        }
        
        if (keys[SDL_SCANCODE_D]) { 
            double oldDirX = p.dirX;
            double rot = rotSpeed; 
            p.dirX = p.dirX * cos(rot) - p.dirY * sin(rot);
            p.dirY = oldDirX * sin(rot) + p.dirY * cos(rot);
            double oldPlaneX = p.planeX;
            p.planeX = p.planeX * cos(rot) - p.planeY * sin(rot);
            p.planeY = oldPlaneX * sin(rot) + p.planeY * cos(rot);
        }
        if (keys[SDL_SCANCODE_A]) { 
            double oldDirX = p.dirX;
            double rot = -rotSpeed; 
            p.dirX = p.dirX * cos(rot) - p.dirY * sin(rot);
            p.dirY = oldDirX * sin(rot) + p.dirY * cos(rot);
            double oldPlaneX = p.planeX;
            p.planeX = p.planeX * cos(rot) - p.planeY * sin(rot);
            p.planeY = oldPlaneX * sin(rot) + p.planeY * cos(rot);
        }

        if (keys[SDL_SCANCODE_UP]) p.pitch += pitchSpeed;
        if (keys[SDL_SCANCODE_DOWN]) p.pitch -= pitchSpeed;
        if (p.pitch > 200) p.pitch = 200;
        if (p.pitch < -200) p.pitch = -200;

        SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255); 
        SDL_RenderClear(renderer);

        drawMap2D(renderer, p);

        int viewWidth = SCREEN_WIDTH / 2;
        int offset3D = SCREEN_WIDTH / 2;

        for(int x = 0; x < viewWidth; x++) {
            double cameraX = 2 * x / (double)viewWidth - 1; 
            double rayDirX = p.dirX + p.planeX * cameraX;
            double rayDirY = p.dirY + p.planeY * cameraX;

            int mapX = int(p.posX);
            int mapY = int(p.posY);

            double sideDistX, sideDistY;
            double deltaDistX = (rayDirX == 0) ? 1e30 : std::abs(1 / rayDirX);
            double deltaDistY = (rayDirY == 0) ? 1e30 : std::abs(1 / rayDirY);
            double perpWallDist;

            int stepX, stepY;
            int hit = 0; 
            int side; 

            if (rayDirX < 0) { stepX = -1; sideDistX = (p.posX - mapX) * deltaDistX; }
            else { stepX = 1; sideDistX = (mapX + 1.0 - p.posX) * deltaDistX; }
            if (rayDirY < 0) { stepY = -1; sideDistY = (p.posY - mapY) * deltaDistY; }
            else { stepY = 1; sideDistY = (mapY + 1.0 - p.posY) * deltaDistY; }

            while (hit == 0) {
                if (sideDistX < sideDistY) { sideDistX += deltaDistX; mapX += stepX; side = 0; }
                else { sideDistY += deltaDistY; mapY += stepY; side = 1; }
                if (worldMap[mapX][mapY] > 0) hit = 1;
            }

            if (side == 0) perpWallDist = (sideDistX - deltaDistX);
            else           perpWallDist = (sideDistY - deltaDistY);

            int lineHeight = (int)(SCREEN_HEIGHT / perpWallDist);
            int horizon = SCREEN_HEIGHT / 2 + p.pitch;
            int drawStart = horizon - (int)(lineHeight * (1.0 - p.posZ)); 
            int drawEnd   = horizon + (int)(lineHeight * p.posZ);       
            int renderStart = std::max(0, drawStart);
            int renderEnd   = std::min(SCREEN_HEIGHT - 1, drawEnd);

            int colorR = 255, colorG = 255, colorB = 255;
            switch(worldMap[mapX][mapY]) {
                case 1: colorR = 255; colorG = 255; colorB = 255; break; 
                case 2: colorR = 0;   colorG = 255; colorB = 0;   break; 
                case 3: colorR = 0;   colorG = 0;   colorB = 255; break; 
                case 4: colorR = 255; colorG = 0;   colorB = 0;   break; 
            }
            if (side == 1) { colorR /= 2; colorG /= 2; colorB /= 2; }

            SDL_SetRenderDrawColor(renderer, colorR, colorG, colorB, 255);
            SDL_RenderDrawLine(renderer, x + offset3D, renderStart, x + offset3D, renderEnd);
            SDL_SetRenderDrawColor(renderer, 135, 206, 235, 255); 
            SDL_RenderDrawLine(renderer, x + offset3D, 0, x + offset3D, renderStart);
            SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255); 
            SDL_RenderDrawLine(renderer, x + offset3D, renderEnd, x + offset3D, SCREEN_HEIGHT);
        }

        SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
        SDL_RenderDrawLine(renderer, SCREEN_WIDTH / 2, 0, SCREEN_WIDTH / 2, SCREEN_HEIGHT);

        SDL_RenderPresent(renderer);
        SDL_Delay(TICK_INTERVAL);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
