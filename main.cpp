#include <SDL.h>
#include <emscripten.h>
#include <cmath>
#include "drawCircle.hpp"
#include <vector>
#include <utility>
#include "fish.hpp"

SDL_Window *window;
SDL_Renderer *renderer;

// std::vector<Circle> body = {
//     {170, 170}
// };

Fish fish = Fish();

void loop()
{
    SDL_Event e;
    while (SDL_PollEvent(&e))
    {
        switch (e.type)
        {
        case SDL_MOUSEBUTTONDOWN:
            if (e.button.button == SDL_BUTTON_LEFT)
            {
                fish.setHeadCircleOffset(e);
            }
            break;
        case SDL_MOUSEBUTTONUP:
            fish.toggleDragging();
            break;
        case SDL_MOUSEMOTION:
            fish.moveHeadCircle(e);
            break;
        }
    }
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    fish.swim(w, h);

    SDL_SetRenderDrawColor(renderer, 20, 30, 60, 255);
    SDL_RenderClear(renderer);
    SDL_SetRenderDrawColor(renderer, 245, 245, 235, 255);
    fish.draw(renderer);
    // fish.drawCircles(renderer);
    SDL_RenderPresent(renderer);
}

int main()
{
    EM_ASM({
        var canvas = document.getElementById('canvas');
        Module['canvas'] = canvas;
        canvas.focus();

        window.addEventListener('resize', function() {
        var canvas = Module['canvas'];
        canvas.width = window.innerWidth;
        canvas.height = window.innerHeight; });
    });

    SDL_Init(SDL_INIT_VIDEO);

    int canvasWidth = EM_ASM_INT({ return window.innerWidth; });
    int canvasHeight = EM_ASM_INT({ return window.innerHeight; });

    window = SDL_CreateWindow("",
                              SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                              canvasWidth, canvasHeight,
                              SDL_WINDOW_SHOWN);

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    emscripten_set_main_loop(loop, 0, 1);
    return 0;
}
