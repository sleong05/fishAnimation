#include "fish.hpp"
#include "drawCircle.hpp"
#include <cmath>
#include <cstdlib>
#include <SDL.h>
#include <vector>
#include "SDL2_gfxPrimitives.h"

#define DEG_TO_RAD (M_PI / 180.0f)
#define TURN_ANGLE 15.0f
#define TURN_CHANCE 0.25f
#define EDGE_BUFFER 250
#define STEP_SIZE 8.0f
#define SPINE_DISTANCE 35

constexpr int BODY_OUTLINE_THICKNESS = 2;
constexpr float OUTER_CURVE_BOOST = 1.3f;

static inline float radNorm(float a)
{
    while (a <= -M_PI)
        a += 2.0f * M_PI;
    while (a > M_PI)
        a -= 2.0f * M_PI;
    return a;
}

static inline float shortestDiffRad(float from, float to)
{
    return radNorm(to - from);
}

static inline void quadPoint(float t,
                             float ax, float ay,
                             float cx, float cy,
                             float bx, float by,
                             float &ox, float &oy)
{
    float u = 1.0f - t;
    float uu = u * u;
    float tt = t * t;
    ox = uu * ax + 2.0f * u * t * cx + tt * bx;
    oy = uu * ay + 2.0f * u * t * cy + tt * by;
}

static inline void drawQuadPolyline(SDL_Renderer *r,
                                    float ax, float ay,
                                    float cx, float cy,
                                    float bx, float by,
                                    int samples, int thickness,
                                    Uint8 R, Uint8 G, Uint8 B, Uint8 A)
{
    float px_prev = ax, py_prev = ay;
    for (int i = 1; i <= samples; ++i)
    {
        float t = (float)i / (float)samples;
        float px, py;
        // uses the quadPoint you already added
        quadPoint(t, ax, ay, cx, cy, bx, by, px, py);
        thickLineRGBA(r,
                      (Sint16)std::lround(px_prev), (Sint16)std::lround(py_prev),
                      (Sint16)std::lround(px), (Sint16)std::lround(py),
                      thickness, R, G, B, A);
        px_prev = px;
        py_prev = py;
    }
}

static inline float lerp(float a, float b, float t) { return a + (b - a) * t; }

constexpr float MAX_SPINE_BEND_DEG = 31.0f; // max bend between adjacent spine links
constexpr float MAX_NECK_BEND_DEG = 17.0f;  // max bend between head direction and first link

Fish::Fish(int startingX, int startingY)
{
    head = new Head(startingX, startingY, 30);

    int partSizes[] = {45, 50, 55, 50, 45, 35, 25, 23, 20, 15};

    float xLocation = 800.0f;
    for (int &size : partSizes)
    {

        bodyParts.emplace_back(xLocation, 400.0f, size);
        xLocation -= 35;
    }
}

Fish::~Fish()
{
    delete head;
}

void Fish::drawCircles(SDL_Renderer *renderer)
{
    draw_circle(renderer, *head); // draw head

    for (auto &bodyPart : bodyParts)
    { // draw body parts
        draw_circle(renderer, bodyPart);
    }
}

static inline void cubicPoint(float t,
                              float ax, float ay,
                              float cx0, float cy0,
                              float cx1, float cy1,
                              float bx, float by,
                              float &ox, float &oy)
{
    float u = 1.0f - t;
    float uu = u * u;
    float tt = t * t;
    float uuu = uu * u;
    float ttt = tt * t;
    ox = uuu * ax + 3.0f * uu * t * cx0 + 3.0f * u * tt * cx1 + ttt * bx;
    oy = uuu * ay + 3.0f * uu * t * cy0 + 3.0f * u * tt * cy1 + ttt * by;
}

void Fish::drawDorsalFinCurves(SDL_Renderer *renderer)
{
    if (bodyParts.size() < 9)
        return; // need indices 3..7

    // --- choose anchors (moved back by 1) ---
    int iA = 3;
    int iB = 6;
    iA = std::min(std::max(iA, 0), (int)bodyParts.size() - 2);
    iB = std::min(std::max(iB, iA + 1), (int)bodyParts.size() - 1);

    // --- endpoints on spine ---
    float Ax = bodyParts[iA].x, Ay = bodyParts[iA].y;
    float Bx = bodyParts[iB].x, By = bodyParts[iB].y;

    // --- global back-normal (for coherent fin “up”) ---
    float spineDir = std::atan2(By - Ay, Bx - Ax);
    float gNx = -std::sin(spineDir), gNy = std::cos(spineDir);

    // --- midpoint ---
    float midx = 0.5f * (Ax + Bx);
    float midy = 0.5f * (Ay + By);

    // --- (kept) size knobs for reference (not used directly now) ---
    float subtleBase = 5.0f, subtleGain = 16.0f;
    float sharpBase = 10.0f, sharpGain = 40.0f;

    // ===== smooth near-flat behavior: spring–damper on signed height =====
    float curvDeg = computeOverallCurvatureDeg();

    // map curvature -> target height (odd & smooth around 0°)
    constexpr float CURV_SOFTEN_DEG = 26.0f; // higher = flatter near zero
    constexpr float H_MAX_SHARP = 30.0f;     // max |height| (px) for outer curve
    constexpr float DEG_AT_MAX = 90.0f;
    float raw = -H_MAX_SHARP * (curvDeg / DEG_AT_MAX);
    float hTarget = std::clamp(raw, -H_MAX_SHARP, H_MAX_SHARP);

    // critically-damped spring for buttery zero-crossing (no jump)
    static bool s_init = false;
    static float s_h = 0.0f; // current height (px)
    static float s_v = 0.0f; // velocity (px/s)

    const float dt = 1.0f / 60.0f; // assume ~60 FPS
    const float omega = 7.0f;      // responsiveness (rad/s)
    const float zeta = 1.0f;       // 1.0 = critical damping

    if (!s_init)
    {
        s_h = hTarget;
        s_v = 0.0f;
        s_init = true;
    }
    {
        float k = omega * omega;
        float c = 2.0f * zeta * omega;
        float a = k * (hTarget - s_h) - c * s_v; // acceleration
        s_v += a * dt;
        s_h += s_v * dt;
    }

    // control points for the SHARP (outer) and SUBTLE (inner) curves
    float cSharpX = midx + gNx * (s_h * OUTER_CURVE_BOOST);
    float cSharpY = midy + gNy * (s_h * OUTER_CURVE_BOOST);

    // inner sits closer to the spine (ratio of the smoothed height)
    constexpr float SUB_RATIO = 0.66f;
    float hSubtle = s_h * SUB_RATIO;
    float cSubtleX = midx + gNx * hSubtle;
    float cSubtleY = midy + gNy * hSubtle;

    // Colors
    const Uint8 finR = 129, finG = 196, finB = 211, finA = 235; // fin fill/inner line color
    const Uint8 outR = 245, outG = 245, outB = 235, outA = 255; // crisp outline color

    // Sampling
    const int SAMPLES = 28; // higher = smoother curves

    // --- sample both curves for fill and stroke ---
    std::vector<Sint16> sharpX;
    sharpX.reserve(SAMPLES + 1);
    std::vector<Sint16> sharpY;
    sharpY.reserve(SAMPLES + 1);
    std::vector<Sint16> subtleX;
    subtleX.reserve(SAMPLES + 1);
    std::vector<Sint16> subtleY;
    subtleY.reserve(SAMPLES + 1);

    for (int i = 0; i <= SAMPLES; ++i)
    {
        float t = (float)i / (float)SAMPLES;
        float px, py;

        quadPoint(t, Ax, Ay, cSharpX, cSharpY, Bx, By, px, py);
        sharpX.push_back((Sint16)std::lround(px));
        sharpY.push_back((Sint16)std::lround(py));

        quadPoint(t, Ax, Ay, cSubtleX, cSubtleY, Bx, By, px, py);
        subtleX.push_back((Sint16)std::lround(px));
        subtleY.push_back((Sint16)std::lround(py));
    }

    // --- fill between curves: sharp A->B then subtle B->A ---
    std::vector<Sint16> vx;
    vx.reserve(2 * (SAMPLES + 1));
    std::vector<Sint16> vy;
    vy.reserve(2 * (SAMPLES + 1));
    for (int i = 0; i <= SAMPLES; ++i)
    {
        vx.push_back(sharpX[i]);
        vy.push_back(sharpY[i]);
    }
    for (int i = SAMPLES; i >= 0; --i)
    {
        vx.push_back(subtleX[i]);
        vy.push_back(subtleY[i]);
    }
    filledPolygonRGBA(renderer, vx.data(), vy.data(), (int)vx.size(), finR, finG, finB, finA);

    // --- crisp strokes with thickness that matches your body outline ---
    for (int i = 1; i <= SAMPLES; ++i)
    {
        thickLineRGBA(renderer,
                      subtleX[i - 1], subtleY[i - 1],
                      subtleX[i], subtleY[i],
                      BODY_OUTLINE_THICKNESS, outR, outG, outB, outA);
    }
    for (int i = 1; i <= SAMPLES; ++i)
    {
        thickLineRGBA(renderer,
                      sharpX[i - 1], sharpY[i - 1],
                      sharpX[i], sharpY[i],
                      BODY_OUTLINE_THICKNESS, outR, outG, outB, outA);
    }
}

float Fish::computeOverallCurvatureDeg() const
{
    if (bodyParts.size() < 3)
        return 0.0f;

    // start from neck
    float prev = std::atan2(head->y - bodyParts[0].y,
                            head->x - bodyParts[0].x);

    float sumRad = 0.0f;
    for (int i = 1; i < (int)bodyParts.size(); ++i)
    {
        const auto &P = bodyParts[i - 1];
        const auto &C = bodyParts[i];
        float h = std::atan2(P.y - C.y, P.x - C.x);
        double scaler = (i < 3) ? 1.6 : 1;
        sumRad += shortestDiffRad(prev, h) * scaler;
        prev = h;
    }
    return sumRad * (180.0f / M_PI);
}

void Fish::draw(SDL_Renderer *renderer)
{
    std::vector<SDL_Point> leftOutline;
    std::vector<SDL_Point> rightOutline;
    // for drawing the front and back parts
    auto frontPoints = getOutlineEndPoints(*head, bodyParts[0]);
    auto backPoints = getOutlineEndPoints(bodyParts[bodyParts.size() - 1], bodyParts[bodyParts.size() - 2]);

    // push outline coordinates
    auto headOutline = getOutlinePoints(*head, bodyParts[0]);
    leftOutline.push_back(headOutline.first);
    rightOutline.push_back(headOutline.second);

    for (int i = 1; i < bodyParts.size(); i++)
    {
        auto bodyOutline = getOutlinePoints(bodyParts[i - 1], bodyParts[i]);
        leftOutline.push_back(bodyOutline.first);
        rightOutline.push_back(bodyOutline.second);
    }
    // big fins
    int bigFinPart = 2;
    float midAngle = getAngle(*head, bodyParts[bigFinPart]);

    // dorsal fin, slightly above spine
    auto [dx, dy] = getPerpendicularOffset(midAngle, 50); // push upwards
    float dorsalX = bodyParts[bigFinPart].x - dx;
    float dorsalY = bodyParts[bigFinPart].y - dy;
    drawRotatedEllipse(renderer, dorsalX, dorsalY, 52, 20, midAngle + 40, {129, 196, 211, 255});

    dorsalX = bodyParts[bigFinPart].x + dx;
    dorsalY = bodyParts[bigFinPart].y + dy;
    drawRotatedEllipse(renderer, dorsalX, dorsalY, 52, 20, midAngle - 40, {129, 196, 211, 255});

    // small fins
    int smallFinPart = 8;
    float tailAngle = getAngle(bodyParts[smallFinPart - 1], bodyParts[smallFinPart]);

    // dorsal fin, slightly above spine
    auto [dx1, dy1] = getPerpendicularOffset(tailAngle, 30); // push upwards
    dorsalX = bodyParts[smallFinPart].x - dx1;
    dorsalY = bodyParts[smallFinPart].y - dy1;
    drawRotatedEllipse(renderer, dorsalX, dorsalY, 30, 11, tailAngle + 40, {129, 196, 211, 255});

    dorsalX = bodyParts[smallFinPart].x + dx1;
    dorsalY = bodyParts[smallFinPart].y + dy1;
    drawRotatedEllipse(renderer, dorsalX, dorsalY, 30, 11, tailAngle - 40, {129, 196, 211, 255});

    // fill in body
    fillBody(frontPoints, leftOutline, backPoints, rightOutline, renderer);

    drawDorsalFinCurves(renderer);
    // draw outline
    drawOutline(renderer, leftOutline, frontPoints, rightOutline, backPoints);

    // draw eyes
    drawEyes(renderer);
}

void Fish::drawEyes(SDL_Renderer *&renderer)
{
    auto [eye1, eye2] = getEyeLocations(*head, bodyParts[0]);
    filledCircleRGBA(renderer, static_cast<Sint16>(eye1.first), static_cast<Sint16>(eye1.second), 7, 245, 245, 235, 255);
    filledCircleRGBA(renderer, static_cast<Sint16>(eye2.first), static_cast<Sint16>(eye2.second), 7, 245, 245, 235, 255);
}

void Fish::drawOutline(SDL_Renderer *&renderer, std::vector<SDL_Point> &leftOutline, std::vector<SDL_Point> &frontPoints, std::vector<SDL_Point> &rightOutline, std::vector<SDL_Point> &backPoints)
{
    int thickness = 3;
    // draw head
    thickLineRGBA(renderer, leftOutline[0].x, leftOutline[0].y, frontPoints[2].x, frontPoints[2].y, thickness, 245, 245, 235, 255);   // left body to outer left head
    thickLineRGBA(renderer, frontPoints[2].x, frontPoints[2].y, frontPoints[1].x, frontPoints[1].y, thickness, 245, 245, 235, 255);   // left tail to center head
    thickLineRGBA(renderer, frontPoints[1].x, frontPoints[1].y, frontPoints[0].x, frontPoints[0].y, thickness, 245, 245, 235, 255);   // center head to right head
    thickLineRGBA(renderer, rightOutline[0].x, rightOutline[0].y, frontPoints[0].x, frontPoints[0].y, thickness, 245, 245, 235, 255); // right head to right body
    // draw body
    for (int i = 0; i < leftOutline.size() - 1; ++i)
    {
        thickLineRGBA(renderer, leftOutline[i].x, leftOutline[i].y, leftOutline[i + 1].x, leftOutline[i + 1].y, thickness, 245, 245, 235, 255);
        thickLineRGBA(renderer, rightOutline[i].x, rightOutline[i].y, rightOutline[i + 1].x, rightOutline[i + 1].y, thickness, 245, 245, 235, 255);
    }

    // tail
    auto [leftX, leftY] = leftOutline.back();
    auto [rightX, rightY] = rightOutline.back();
    thickLineRGBA(renderer, rightX, rightY, backPoints[2].x, backPoints[2].y, thickness, 245, 245, 235, 255);                   // left body to outer left tail
    thickLineRGBA(renderer, backPoints[2].x, backPoints[2].y, backPoints[1].x, backPoints[1].y, thickness, 245, 245, 235, 255); // left tail to center tail
    thickLineRGBA(renderer, backPoints[1].x, backPoints[1].y, backPoints[0].x, backPoints[0].y, thickness, 245, 245, 235, 255); // center tail to right tail
    thickLineRGBA(renderer, leftX, leftY, backPoints[0].x, backPoints[0].y, thickness, 245, 245, 235, 255);                     // right tail to right body
}

void Fish::fillBody(std::vector<SDL_Point> &frontPoints, std::vector<SDL_Point> &leftOutline, std::vector<SDL_Point> &backPoints, std::vector<SDL_Point> &rightOutline, SDL_Renderer *&renderer)
{
    std::vector<Sint16> vx;
    std::vector<Sint16> vy;

    // add head points go right to left
    for (auto &point : frontPoints)
    {
        vx.push_back(static_cast<Sint16>(point.x));
        vy.push_back(static_cast<Sint16>(point.y));
    }
    // Add left outline points
    for (const auto &p : leftOutline)
    {
        vx.push_back(p.x);
        vy.push_back(p.y);
    }

    // add tail points still go right to left so reverse the order
    for (auto &point : backPoints)
    {
        vx.push_back(static_cast<Sint16>(point.x));
        vy.push_back(static_cast<Sint16>(point.y));
    }

    // Add right outline points in reverse (tail → head)
    for (int i = rightOutline.size() - 1; i >= 0; --i)
    {
        vx.push_back(rightOutline[i].x);
        vy.push_back(rightOutline[i].y);
    }
    Uint8 r = 54, g = 124, b = 162, a = 255;
    filledPolygonRGBA(renderer, vx.data(), vy.data(), vx.size(), r, g, b, a);
}

// ----------------- helpers -----------------
static float normDeg(float a)
{
    while (a < 0)
        a += 360;
    while (a >= 360)
        a -= 360;
    return a;
}
float shortestDiffDeg(float from, float to)
{
    float diff = fmodf(to - from + 540.0f, 360.0f) - 180.0f;
    return diff;
}

// -------------------------------------------
// Call once per frame
void Fish::swim(int canvasW, int canvasH)
{
    constexpr float ARC_TURN_SPEED = 2.5f; // deg per frame
    constexpr float STEP = 8.0f;           // swim speed
    constexpr int ARC_MIN_DURATION = 80;   // min frames per arc
    constexpr int ARC_MAX_DURATION = 180;
    constexpr float WALL_ARC_BOOST = 1.8f;
    constexpr float EDGE_GRADIENT = 120.0f;
    constexpr float MAX_TOTAL_TURN = .9f;

    // 1. Occasionally flip turning direction (every few seconds)
    if (arcDurationFrames <= 0)
    {
        turnDirection = (rand() % 2 == 0) ? 1 : -1;
        arcDurationFrames = ARC_MIN_DURATION + rand() % (ARC_MAX_DURATION - ARC_MIN_DURATION);
    }
    arcDurationFrames--;

    // 2. Base turn: gentle continuous arc
    float desiredTurn = turnDirection * ARC_TURN_SPEED;

    // 3. Steer away softly from walls
    auto steerAway = [&](float dist, float steerTo)
    {
        if (dist < EDGE_GRADIENT)
        {
            float t = 1.0f - dist / EDGE_GRADIENT;
            float wallInfluence = shortestDiffDeg(head->direction, steerTo);
            desiredTurn += wallInfluence * t * 0.2f * WALL_ARC_BOOST;
        }
    };

    steerAway(head->x, 0.0f);             // left
    steerAway(canvasW - head->x, 180.0f); // right
    steerAway(head->y, 90.0f);            // top
    steerAway(canvasH - head->y, 270.0f); // bottom

    float clampedTurn = std::clamp(desiredTurn, -MAX_TOTAL_TURN, MAX_TOTAL_TURN);
    head->direction = normDeg(head->direction + clampedTurn);

    // 5. Move forward
    float rad = head->direction * (M_PI / 180.f);
    float nextX = head->x + std::cos(rad) * STEP;
    float nextY = head->y + std::sin(rad) * STEP;

    updateHead(nextX, nextY);
}

void Fish::updateBody()
{
    // Convert caps to radians once
    const float maxSpineBend = MAX_SPINE_BEND_DEG * (M_PI / 180.0f);
    const float maxNeckBend = MAX_NECK_BEND_DEG * (M_PI / 180.0f);

    // ----- First segment follows the head with a neck bend cap -----
    BodyPart &first = bodyParts[0];

    // Head direction in radians
    float headDirRad = head->direction * (M_PI / 180.0f);

    // Desired direction from first segment toward head
    float desired0 = std::atan2(head->y - first.y, head->x - first.x);

    // Clamp neck bend relative to head facing
    float delta0 = shortestDiffRad(headDirRad, desired0);
    delta0 = std::clamp(delta0, -maxNeckBend, maxNeckBend);
    float dir0 = headDirRad + delta0;

    // Place first segment exactly SPINE_DISTANCE behind the head along clamped dir
    first.x = head->x - std::cos(dir0) * SPINE_DISTANCE;
    first.y = head->y - std::sin(dir0) * SPINE_DISTANCE;

    // Keep track of previous segment direction for curvature continuity
    float prevDir = dir0;

    // ----- Remaining segments: clamp bend vs. previous segment -----
    for (int i = 1; i < static_cast<int>(bodyParts.size()); ++i)
    {
        BodyPart &prev = bodyParts[i - 1];
        BodyPart &cur = bodyParts[i];

        // Heading from current toward prev (where we want to go)
        float desired = std::atan2(prev.y - cur.y, prev.x - cur.x);

        // Limit how much this segment can turn vs. previous segment's heading
        float delta = shortestDiffRad(prevDir, desired);
        delta = std::clamp(delta, -maxSpineBend, maxSpineBend);
        float dir = prevDir + delta;

        // Place this segment SPINE_DISTANCE behind the previous one along clamped dir
        cur.x = prev.x - std::cos(dir) * SPINE_DISTANCE;
        cur.y = prev.y - std::sin(dir) * SPINE_DISTANCE;

        prevDir = dir; // propagate heading down the spine
    }
}

void Fish::setHeadCircleOffset(const SDL_Event &e)
{
    float dx = e.button.x - head->x;
    float dy = e.button.y - head->y;
    if (dx * dx + dy * dy <= CIRCLE_RADIUS * CIRCLE_RADIUS)
    {
        head->dragging = true;
        head->offsetX = dx;
        head->offsetY = dy;
    }
}

void Fish::moveHeadCircle(const SDL_Event &e)
{
    if (head->dragging)
    {
        float targetX = e.motion.x - head->offsetX;
        float targetY = e.motion.y - head->offsetY;
        // smooth
        updateHead(targetX, targetY);
    }
}

void Fish::updateHead(float targetX, float targetY)
{
    head->x += (targetX - head->x) * 0.2f;
    head->y += (targetY - head->y) * 0.2f;
    updateBody();
}

void Fish::toggleDragging()
{
    head->dragging = false;
}