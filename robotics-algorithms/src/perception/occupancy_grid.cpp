#include "robotics_algo/perception/occupancy_grid.hpp"
#include <algorithm>
#include <cmath>

namespace robotics::perception {

    OccupancyGrid::OccupancyGrid(int w, int h, double res) 
        : width_(w), height_(h), resolution_(res) {
        log_odds_.resize(w * h, 0.0);
    }

    // Helper: World -> Grid Index
    struct GridIdx { int x, y; };
    GridIdx worldToGrid(double wx, double wy, double res) {
        return { static_cast<int>(wx / res), static_cast<int>(wy / res) };
    }

    bool isValid(GridIdx p, int w, int h) {
        return p.x >= 0 && p.x < w && p.y >= 0 && p.y < h;
    }

    void OccupancyGrid::update(const Pose2D& pose, const std::vector<Vector2>& hits) {
        const double LO_OCCUPIED = 0.85; 
        const double LO_FREE = -0.4;
        const double MAX_VAL = 10.0;
        const double MIN_VAL = -10.0;

        Vector2 start = pose.position;

        for (const auto& end : hits) {
            GridIdx p0 = worldToGrid(start.x(), start.y(), resolution_);
            GridIdx p1 = worldToGrid(end.x(), end.y(), resolution_);

            int x0 = p0.x; int y0 = p0.y;
            int x1 = p1.x; int y1 = p1.y;

            int dx = std::abs(x1 - x0);
            int dy = std::abs(y1 - y0);
            int sx = (x0 < x1) ? 1 : -1;
            int sy = (y0 < y1) ? 1 : -1;
            int err = dx - dy;

            while (true) {
                // CASE A: We are at the Wall (Hit Point)
                if (x0 == x1 && y0 == y1) {
                    if (isValid({x0, y0}, width_, height_)) {
                        int idx = y0 * width_ + x0;
                        log_odds_[idx] += LO_OCCUPIED;
                        if(log_odds_[idx] > MAX_VAL) log_odds_[idx] = MAX_VAL;
                    }
                    break; // STOP! Do not continue loop
                } 
                
                // CASE B: We are in Free Space
                if (isValid({x0, y0}, width_, height_)) {
                    int idx = y0 * width_ + x0;
                    log_odds_[idx] += LO_FREE;
                    if(log_odds_[idx] < MIN_VAL) log_odds_[idx] = MIN_VAL;
                }

                int e2 = 2 * err;
                if (e2 > -dy) { err -= dy; x0 += sx; }
                if (e2 < dx) { err += dx; y0 += sy; }
            }
        }
    }

    // Stub to satisfy linker
    std::vector<Vector2> OccupancyGrid::traceRay(Vector2 s, Vector2 e) { (void)s; (void)e; return {}; }
    
    bool OccupancyGrid::isOccupied(double x, double y) const {
        GridIdx idx = worldToGrid(x, y, resolution_);
        if (!isValid(idx, width_, height_)) return true;
        return log_odds_[idx.y * width_ + idx.x] > 0.0;
    }
}
