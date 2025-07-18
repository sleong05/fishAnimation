// draw.cpp
#include "drawCircle.hpp"
#include <cmath>
#include <numbers>
#include <utility>

void draw_circle(SDL_Renderer *renderer, BodyPart &bodyPart)
{
    int cx = bodyPart.x;
    int cy = bodyPart.y;
    int r = bodyPart.radius;
    drawDots(renderer, cx, cy, r);
}

void draw_circle(SDL_Renderer *renderer, float cx, float cy, float r)
{
    drawDots(renderer, cx, cy, r);
}

void drawDots(SDL_Renderer *renderer, float cx, float cy, float r)
{
    const int points = 200;
    for (int i = 0; i < points; ++i)
    {
        float angle = 2 * M_PI * i / points;
        int x = static_cast<int>(cx + r * cosf(angle));
        int y = static_cast<int>(cy + r * sinf(angle));
        SDL_RenderDrawPoint(renderer, x, y);
    }
}

// gets the angle between two bodyParts
float getAngle(BodyPart &a, BodyPart &b)
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return std::atan2(dy, dx);
}

OutlinePair getOutlinePoints(BodyPart &part, BodyPart &nextPart)
{
    float theta = getAngle(part, nextPart);
    float radius = part.radius;

    int x = static_cast<int>(part.x + radius * cosf(theta - pi / 2.0));
    int y = static_cast<int>(part.y + radius * sinf(theta - pi / 2.0));

    SDL_Point left = {x, y};

    x = static_cast<int>(part.x + radius * cosf(theta + pi / 2.0));
    y = static_cast<int>(part.y + radius * sinf(theta + pi / 2.0));

    SDL_Point right = {x, y};

    return {left, right};
}

std::vector<SDL_Point> getOutlineEndPoint(BodyPart &endPart, BodyPart &closestPart)
{
    float x1 = endPart.x;
    float y1 = endPart.y;

    float x2 = closestPart.x;
    float y2 = closestPart.y;

    float dx = x2 - x1;
    float dy = y2 - y1;
    float length = std::sqrt(dx * dx + dy * dy);

    float unitX = -dx / length;
    float unitY = -dy / length;
    float angle45 = M_PI / 4;
    // Rotate -45
    float rightX = unitX * std::cos(-angle45) - unitY * std::sin(-angle45);
    float rightY = unitX * std::sin(-angle45) + unitY * std::cos(-angle45);

    // Rotate +45
    float leftX = unitX * std::cos(angle45) - unitY * std::sin(angle45);
    float leftY = unitX * std::sin(angle45) + unitY * std::cos(angle45);

    int distance = 15;
    SDL_Point tailLeft = {
        static_cast<int>(x1 + leftX * distance),
        static_cast<int>(y1 + leftY * distance)};

    SDL_Point tailRight = {
        static_cast<int>(x1 + rightX * distance),
        static_cast<int>(y1 + rightY * distance)};

    int endX = static_cast<int>(x1 + unitX * 15);
    int endY = static_cast<int>(y1 + unitY * 15);

    SDL_Point tailCenter = {
        static_cast<int>(x1 + unitX * 15),
        static_cast<int>(y1 + unitY * 15)};

    return {tailRight, tailCenter, tailLeft};
}

std::pair<float, float> getNewPosition(float x1, float y1, float x2, float y2, float desiredLength)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    float length = std::sqrt(dx * dx + dy * dy);

    if (length < 1e-5f) // avoid division by zero
        return {x1, y1};

    float ux = dx / length;
    float uy = dy / length;

    float endX = x1 + ux * desiredLength;
    float endY = y1 + uy * desiredLength;

    return {endX, endY};
}

eyeLocationsPair getEyeLocations(BodyPart &head, BodyPart &firstPart)
{
    float cx = head.x;
    float cy = head.y;
    float angle = getAngle(head, firstPart);

    float forwardOffset = 15.0f; // distance in front of head center
    float sideOffset = 22.0f;    // eye spacing

    float baseX = cx + std::cos(angle) * forwardOffset;
    float baseY = cy + std::sin(angle) * forwardOffset;

    float dx = -std::sin(angle);
    float dy = std::cos(angle);

    // Eye positions
    float eye1X = baseX + dx * sideOffset;
    float eye1Y = baseY + dy * sideOffset;

    float eye2X = baseX - dx * sideOffset;
    float eye2Y = baseY - dy * sideOffset;
    std::pair eye1 = {eye1X, eye1Y};
    std::pair eye2 = {eye2X, eye2Y};
    return {eye1, eye2};
}
