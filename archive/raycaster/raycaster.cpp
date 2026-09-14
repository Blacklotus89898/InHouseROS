#include <SDL2/SDL.h>
#include <cmath>
#include <vector>
#include <iostream>

// --- Constants ---
const int SCREEN_WIDTH = 1024;
const int SCREEN_HEIGHT = 512;
const int TICK_INTERVAL = 16; // ~60 FPS

// Map dimensions
const int MAP_WIDTH = 24;
const int MAP_HEIGHT = 24;
const int TILE_SIZE = 20; // Size of tile in 2D view (Left side)

// The Map (1 = Wall, 0 = Empty)
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
    double posX, posY;      // Position vector
    double dirX, dirY;      // Direction vector
    double planeX, planeY;  // Camera plane vector (determines FOV)
};

void drawMap2D(SDL_Renderer* renderer, const Player& p) {
    // 1. Draw Grid Tiles
    for(int x = 0; x < MAP_WIDTH; x++) {
        for(int y = 0; y < MAP_HEIGHT; y++) {
            if(worldMap[x][y] > 0) {
                // Different colors for different block types
                switch(worldMap[x][y]) {
                    case 1: SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255); break; // White
                    case 2: SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255); break;     // Green
                    case 3: SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255); break;     // Blue
                    case 4: SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); break;     // Red
                    default: SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255); break;
                }
            } else {
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // Black floor
            }
            SDL_Rect tileRect = {x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE - 1, TILE_SIZE - 1};
            SDL_RenderFillRect(renderer, &tileRect);
        }
    }

    // 2. Draw Player (Yellow Dot)
    SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
    SDL_Rect playerRect = {(int)(p.posX * TILE_SIZE) - 3, (int)(p.posY * TILE_SIZE) - 3, 6, 6};
    SDL_RenderFillRect(renderer, &playerRect);
    
    // Draw small Direction Line
    SDL_RenderDrawLine(renderer, 
        (int)(p.posX * TILE_SIZE), (int)(p.posY * TILE_SIZE), 
        (int)((p.posX + p.dirX) * TILE_SIZE), (int)((p.posY + p.dirY) * TILE_SIZE));
}

int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = SDL_CreateWindow("Dual View: Raycasting (Wolf3D)", 
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    // Initialize Player
    Player p;
    p.posX = 12; p.posY = 12; // Start in middle
    p.dirX = -1; p.dirY = 0; // Initial direction vector
    p.planeX = 0; p.planeY = -0.66; // <--- Negative sign fixes the 3D inversion

    bool running = true;
    
    // Input state
    const Uint8* keys = SDL_GetKeyboardState(NULL);

    while (running) {
        // 1. Input Handling
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
        }

        // Movement Speed
        double moveSpeed = 0.05;
        double rotSpeed = 0.03;

        // Move Forward
        if (keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP]) {
            if(worldMap[int(p.posX + p.dirX * moveSpeed)][int(p.posY)] == 0) p.posX += p.dirX * moveSpeed;
            if(worldMap[int(p.posX)][int(p.posY + p.dirY * moveSpeed)] == 0) p.posY += p.dirY * moveSpeed;
        }
        // Move Backward
        if (keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN]) {
            if(worldMap[int(p.posX - p.dirX * moveSpeed)][int(p.posY)] == 0) p.posX -= p.dirX * moveSpeed;
            if(worldMap[int(p.posX)][int(p.posY - p.dirY * moveSpeed)] == 0) p.posY -= p.dirY * moveSpeed;
        }
        // --- CORRECTED ROTATION LOGIC ---
        
        // Rotate Right (Now uses positive rotation for SDL Y-Down space)
        if (keys[SDL_SCANCODE_D] || keys[SDL_SCANCODE_RIGHT]) {
            double oldDirX = p.dirX;
            double rot = rotSpeed; // Positive for Right
            p.dirX = p.dirX * cos(rot) - p.dirY * sin(rot);
            p.dirY = oldDirX * sin(rot) + p.dirY * cos(rot);
            double oldPlaneX = p.planeX;
            p.planeX = p.planeX * cos(rot) - p.planeY * sin(rot);
            p.planeY = oldPlaneX * sin(rot) + p.planeY * cos(rot);
        }
        // Rotate Left (Now uses negative rotation)
        if (keys[SDL_SCANCODE_A] || keys[SDL_SCANCODE_LEFT]) {
            double oldDirX = p.dirX;
            double rot = -rotSpeed; // Negative for Left
            p.dirX = p.dirX * cos(rot) - p.dirY * sin(rot);
            p.dirY = oldDirX * sin(rot) + p.dirY * cos(rot);
            double oldPlaneX = p.planeX;
            p.planeX = p.planeX * cos(rot) - p.planeY * sin(rot);
            p.planeY = oldPlaneX * sin(rot) + p.planeY * cos(rot);
        }
        // 2. Clear Screen
        SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255); // Dark Gray BG
        SDL_RenderClear(renderer);

        // 3. Draw Top-Down Map (Left Side)
        drawMap2D(renderer, p);

        // 4. RAYCASTING LOOP (Draws 3D on Right Side AND Rays on Left Side)
        // We render the 3D view on the right half (x = 512 to 1024)
        int viewWidth = SCREEN_WIDTH / 2;
        int offset3D = SCREEN_WIDTH / 2;

        for(int x = 0; x < viewWidth; x++) {
            // -- Calculate Ray Position and Direction --
            // cameraX is -1 for left side of screen, 0 center, 1 right side
            double cameraX = 2 * x / (double)viewWidth - 1; 
            double rayDirX = p.dirX + p.planeX * cameraX;
            double rayDirY = p.dirY + p.planeY * cameraX;

            // -- Map Coords --
            int mapX = int(p.posX);
            int mapY = int(p.posY);

            // -- DDA Variables --
            double sideDistX, sideDistY;
            double deltaDistX = (rayDirX == 0) ? 1e30 : std::abs(1 / rayDirX);
            double deltaDistY = (rayDirY == 0) ? 1e30 : std::abs(1 / rayDirY);
            double perpWallDist;

            int stepX, stepY;
            int hit = 0; 
            int side; // 0 for NS, 1 for EW

            // -- Step Calculation --
            if (rayDirX < 0) {
                stepX = -1;
                sideDistX = (p.posX - mapX) * deltaDistX;
            } else {
                stepX = 1;
                sideDistX = (mapX + 1.0 - p.posX) * deltaDistX;
            }
            if (rayDirY < 0) {
                stepY = -1;
                sideDistY = (p.posY - mapY) * deltaDistY;
            } else {
                stepY = 1;
                sideDistY = (mapY + 1.0 - p.posY) * deltaDistY;
            }

            // -- DDA Hit Loop --
            while (hit == 0) {
                // Jump to next map square, OR in x-direction, OR in y-direction
                if (sideDistX < sideDistY) {
                    sideDistX += deltaDistX;
                    mapX += stepX;
                    side = 0;
                } else {
                    sideDistY += deltaDistY;
                    mapY += stepY;
                    side = 1;
                }
                // Check if ray has hit a wall
                if (worldMap[mapX][mapY] > 0) hit = 1;
            }

            // -- Calculate Distance (Corrected for Fisheye) --
            if (side == 0) perpWallDist = (sideDistX - deltaDistX);
            else           perpWallDist = (sideDistY - deltaDistY);

            // -- DRAW 2D RAY (On Left View) --
            // We only draw every 10th ray to keep the map readable
            if (x % 10 == 0) { 
                SDL_SetRenderDrawColor(renderer, 255, 255, 0, 50); // Faint Yellow
                // End point of the ray
                double endX = p.posX + rayDirX * perpWallDist;
                double endY = p.posY + rayDirY * perpWallDist;
                SDL_RenderDrawLine(renderer, 
                    (int)(p.posX * TILE_SIZE), (int)(p.posY * TILE_SIZE), 
                    (int)(endX * TILE_SIZE), (int)(endY * TILE_SIZE));
            }

            // -- DRAW 3D SLICE (On Right View) --
            
            // Calculate height of line to draw on screen
            int lineHeight = (int)(SCREEN_HEIGHT / perpWallDist);

            // Calculate lowest and highest pixel to fill in current stripe
            int drawStart = -lineHeight / 2 + SCREEN_HEIGHT / 2;
            if (drawStart < 0) drawStart = 0;
            int drawEnd = lineHeight / 2 + SCREEN_HEIGHT / 2;
            if (drawEnd >= SCREEN_HEIGHT) drawEnd = SCREEN_HEIGHT - 1;

            // Choose Wall Color
            int colorR = 255, colorG = 255, colorB = 255;
            switch(worldMap[mapX][mapY]) {
                case 1: colorR = 255; colorG = 255; colorB = 255; break; // White
                case 2: colorR = 0;   colorG = 255; colorB = 0;   break; // Green
                case 3: colorR = 0;   colorG = 0;   colorB = 255; break; // Blue
                case 4: colorR = 255; colorG = 0;   colorB = 0;   break; // Red
            }

            // Give x and y sides different brightness for depth effect
            if (side == 1) { 
                colorR /= 2; colorG /= 2; colorB /= 2; 
            }

            // Draw the vertical line
            SDL_SetRenderDrawColor(renderer, colorR, colorG, colorB, 255);
            SDL_RenderDrawLine(renderer, x + offset3D, drawStart, x + offset3D, drawEnd);
            
            // Draw Floor/Ceiling (Simple solid colors)
            // Ceiling
            SDL_SetRenderDrawColor(renderer, 135, 206, 235, 255); // Sky Blue
            SDL_RenderDrawLine(renderer, x + offset3D, 0, x + offset3D, drawStart);
            // Floor
            SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255); // Gray
            SDL_RenderDrawLine(renderer, x + offset3D, drawEnd, x + offset3D, SCREEN_HEIGHT);
        }

        // Draw Divider Line
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
