#include <SDL2/SDL.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>

// --- Constants ---
const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;
const int TICK_INTERVAL = 16; 

const int MAP_SIZE = 64; // Larger map for exploration

// WORLD DATA: Integers represent HEIGHT
// 0 = Water/Abyss, 1 = Sand, 2 = Grass, 3 = Stone, 4 = Snow
int worldMap[MAP_SIZE][MAP_SIZE]; 

// Smoother terrain generation
void generateWorld() {
    for (int x = 0; x < MAP_SIZE; x++) {
        for (int y = 0; y < MAP_SIZE; y++) {
            // Simple distance-based island generation + random noise
            float cx = x - MAP_SIZE/2.0f;
            float cy = y - MAP_SIZE/2.0f;
            float dist = sqrt(cx*cx + cy*cy);
            
            int baseHeight = 4 - (int)(dist / 5.0f); 
            if (baseHeight < 0) baseHeight = 0;

            // Add noise
            if (baseHeight > 0) {
                if (rand() % 10 > 7) baseHeight += 1;
                if (rand() % 10 > 8) baseHeight -= 1;
            }
            
            // Border
            if (x==0 || x==MAP_SIZE-1 || y==0 || y==MAP_SIZE-1) baseHeight = 10;
            
            worldMap[x][y] = std::max(0, baseHeight);
        }
    }
    // Flatten start area
    worldMap[32][32] = 2;
}

struct Player {
    double x, y;       
    double dirX, dirY; 
    double planeX, planeY; 
    double z;      // Eyes Height
    double velZ;   
    double pitch;  // Look Up/Down horizon offset
    bool onGround;
};

// --- RENDERERS ---

void drawMiniMap(SDL_Renderer* renderer, const Player& p) {
    int scale = 4;
    for(int x = 0; x < MAP_SIZE; x++) {
        for(int y = 0; y < MAP_SIZE; y++) {
            int h = worldMap[x][y];
            if(h > 0) {
                SDL_SetRenderDrawColor(renderer, 50 * h, 100 + (20*h), 50, 255);
                SDL_Rect r = {x * scale, y * scale, scale, scale};
                SDL_RenderFillRect(renderer, &r);
            }
        }
    }
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_Rect pr = {(int)(p.x * scale)-1, (int)(p.y * scale)-1, 3, 3};
    SDL_RenderFillRect(renderer, &pr);
}

void renderVoxelSpace(SDL_Renderer* renderer, const Player& p) {
    // 1. Draw Sky
    int horizon = SCREEN_HEIGHT / 2 + (int)p.pitch;
    SDL_SetRenderDrawColor(renderer, 135, 206, 235, 255);
    SDL_Rect sky = {0, 0, SCREEN_WIDTH, SCREEN_HEIGHT};
    SDL_RenderFillRect(renderer, &sky);

    // 2. Voxel Loop (Front-to-Back)
    // We cast a ray for every vertical column on screen
    for(int x = 0; x < SCREEN_WIDTH; x += 2) { 
        double cameraX = 2 * x / (double)SCREEN_WIDTH - 1; 
        double rayDirX = p.dirX + p.planeX * cameraX;
        double rayDirY = p.dirY + p.planeY * cameraX;

        double mapX = p.x;
        double mapY = p.y;
        
        // Ray increments (Low quality step for speed/retro feel)
        double stepSize = 0.1; 
        double distance = 0;
        
        // yBuffer tracks the lowest pixel we have drawn so far (starts at bottom)
        int yBuffer = SCREEN_HEIGHT;
        
        // Track previous height to detect Walls vs Tops
        int lastHeight = 0; // Assume starting at 0 (or player's current block height)

        // Ray Marching (Max distance 40 blocks)
        for (int i = 0; i < 400; i++) {
            mapX += rayDirX * stepSize;
            mapY += rayDirY * stepSize;
            distance += stepSize;

            int iMapX = (int)mapX;
            int iMapY = (int)mapY;

            // Bounds check
            if (iMapX < 0 || iMapX >= MAP_SIZE || iMapY < 0 || iMapY >= MAP_SIZE) break;

            int height = worldMap[iMapX][iMapY];
            
            // PROJECT HEIGHT TO SCREEN
            // Standard perspective projection
            // screenY = (HeightDiff / Distance) * Scale + Horizon
            // Note: We subtract because Screen Y is inverted (0 is top)
            int heightOnScreen = (int)((p.z - height) / distance * 200.0 + horizon);

            // Clamp top (don't draw above screen, though strictly not needed if we stop at 0)
            if (heightOnScreen < 0) heightOnScreen = 0;

            // RENDER STRIP
            // Only draw if this block sticks up higher than what we've already drawn (yBuffer)
            if (heightOnScreen < yBuffer) {
                
                // Color Selection
                int r, g, b;
                if (height == 0) { r=60; g=100; b=200; }      // Water
                else if (height == 1) { r=210; g=180; b=140;} // Sand
                else if (height == 2) { r=34; g=139; b=34; }  // Grass
                else if (height == 3) { r=105; g=105; b=105;} // Stone
                else { r=255; g=250; b=250; }                 // Snow/Border

                // "Edge/Wall" Detection
                // If the height jumped up significantly from the last block we processed,
                // it's a "Front Face" (Wall). We make it darker.
                if (height > lastHeight) {
                     r/=2; g/=2; b/=2; 
                } 
                // If it's the same height (or lower), it's a "Top Face" (Roof).
                // Leave color bright.

                SDL_SetRenderDrawColor(renderer, r, g, b, 255);
                
                // Draw vertical strip from previous lowest point up to new height
                SDL_Rect strip = {x, heightOnScreen, 2, yBuffer - heightOnScreen};
                SDL_RenderFillRect(renderer, &strip);

                // Update the buffer to the new height
                yBuffer = heightOnScreen;
            }
            
            lastHeight = height;

            // Optimization: If we hit the top of the screen, stop casting for this column
            if (yBuffer <= 0) break;
        }
    }
}

int main(int argc, char* argv[]) {
    srand(time(0));
    generateWorld();

    if (SDL_Init(SDL_INIT_VIDEO) < 0) return 1;

    SDL_Window* window = SDL_CreateWindow("Voxel Space (Comanche Style)", 
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    Player p;
    p.x = 32.5; p.y = 32.5; // Start middle
    p.dirX = -1; p.dirY = 0; 
    p.planeX = 0; p.planeY = -0.66; 
    p.z = 3.0; // Start high up to see the view
    p.velZ = 0;
    p.pitch = 0;
    p.onGround = false;

    bool running = true;
    const Uint8* keys = SDL_GetKeyboardState(NULL);

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_SPACE) {
                 // Fly/Jump
                 p.velZ = 0.5;
                 p.onGround = false;
            }
        }

        // --- PHYSICS ---
        int mapX = (int)p.x;
        int mapY = (int)p.y;
        
        // Boundary safety
        if (mapX < 0) mapX=0; if (mapX >= MAP_SIZE) mapX=MAP_SIZE-1;
        if (mapY < 0) mapY=0; if (mapY >= MAP_SIZE) mapY=MAP_SIZE-1;

        double groundHeight = worldMap[mapX][mapY]; 

        p.z += p.velZ;
        if (!p.onGround) p.velZ -= 0.02; // Gravity

        // Collision with floor
        // We add 1.5 height for the player's eyes
        if (p.z < groundHeight + 1.5) {
            p.z = groundHeight + 1.5;
            p.velZ = 0;
            p.onGround = true;
        } else {
            p.onGround = false;
        }

        // --- MOVEMENT ---
        double moveSpeed = 0.2;
        double rotSpeed = 0.04;

        double nextX = p.x;
        double nextY = p.y;

        if (keys[SDL_SCANCODE_W]) { nextX += p.dirX * moveSpeed; nextY += p.dirY * moveSpeed; }
        if (keys[SDL_SCANCODE_S]) { nextX -= p.dirX * moveSpeed; nextY -= p.dirY * moveSpeed; }

        // Simple collision: Can't walk into blocks higher than current foot level
        // Foot level is approx (EyeHeight - 1.0)
        double footZ = p.z - 0.5;
        int nextMapX = (int)nextX; 
        int nextMapY = (int)nextY;
        
        if (nextMapX >=0 && nextMapX < MAP_SIZE && nextMapY >=0 && nextMapY < MAP_SIZE) {
            if (worldMap[nextMapX][(int)p.y] <= footZ) p.x = nextX;
            if (worldMap[(int)p.x][nextMapY] <= footZ) p.y = nextY;
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
        
        if (keys[SDL_SCANCODE_UP]) p.pitch += 10;
        if (keys[SDL_SCANCODE_DOWN]) p.pitch -= 10;
        if (p.pitch > 300) p.pitch = 300;
        if (p.pitch < -300) p.pitch = -300;

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        renderVoxelSpace(renderer, p);
        drawMiniMap(renderer, p);

        SDL_RenderPresent(renderer);
        SDL_Delay(TICK_INTERVAL);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
