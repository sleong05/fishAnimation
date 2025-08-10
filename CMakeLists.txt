cmake_minimum_required(VERSION 3.0)
project(SDLCircle)

set(CMAKE_CXX_STANDARD 17)

# Check if using Emscripten
if("${CMAKE_CXX_COMPILER}" MATCHES "em\\+\\+")
    set(CMAKE_EXECUTABLE_SUFFIX ".html")

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
        -sUSE_SDL=2 \
        -sUSE_SDL_GFX=2 \
        -sWASM=1 \
        -sALLOW_MEMORY_GROWTH=1 \
        -sEXIT_RUNTIME=1 \
        -sFULL_ES2=1 \
        -sOFFSCREEN_FRAMEBUFFER=1 \
        -sENVIRONMENT=web \
    ")
endif()

# Add your source files
add_executable(main
    main.cpp
    drawCircle.cpp
    fish.cpp
)
