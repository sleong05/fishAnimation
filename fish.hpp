#include <vector>
#include <SDL.h>
#include "BodyPart.hpp"

class Fish
{
public:
    Fish();
    ~Fish();

    void swim(int canvasWidth, int canvasHeight);

    void draw(SDL_Renderer *renderer);
    void drawEyes(SDL_Renderer *&renderer);
    void drawOutline(SDL_Renderer *&renderer, std::vector<SDL_Point> &leftOutline, std::vector<SDL_Point> &frontPoints, std::vector<SDL_Point> &rightOutline, std::vector<SDL_Point> &backPoints);
    void fillBody(std::vector<SDL_Point> &frontPoints, std::vector<SDL_Point> &leftOutline, std::vector<SDL_Point> &backPoints, std::vector<SDL_Point> &rightOutline, SDL_Renderer *&renderer);
    void drawCircles(SDL_Renderer *renderer);

    void setHeadCircleOffset(const SDL_Event &e);
    void moveHeadCircle(const SDL_Event &e);
    void toggleDragging();

private:
    int launchFrames = 0;
    float computeOverallCurvatureDeg() const;
    void drawDorsalFinCurves(SDL_Renderer *renderer);

    float turnBias = 0.0f; // persistent per-fish; + = CCW, − = CW
    // Add to the Fish class (public or private)
    int turnDirection = 1;     // +1 = CCW, -1 = CW
    int arcDurationFrames = 0; // countdown for how long to keep current arc
    // fish.hpp  (inside class Fish)
    float headingVel = 0.0f; // current turn speed  (°/frame, signed)
    int arcFrames = 0;       // frames remaining in this arc
    int turnSign = 1;        // +1 = CCW, -1 = CW
    // Fish.hpp  (inside class Fish)
    float turnRate = 0.0f; // current signed turning speed (deg/frame)
    int arcTimer = 0;      // frames remaining in this arc

    int changeDirection = 0;
    void updateHead(float targetX, float targetY);
    void updateBody();

    std::vector<BodyPart> bodyParts;
    Head *head;

    int CIRCLE_RADIUS = 20;
};