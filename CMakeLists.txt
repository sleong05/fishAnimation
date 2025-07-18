cmake_minimum_required(VERSION 3.0)
project(SDLCircle)

set(CMAKE_CXX_STANDARD 17)

# If compiling with Emscripten, adjust output file format
if("${CMAKE_CXX_COMPILER}" MATCHES "em\\+\\+")
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
endif()

# Common Emscripten flags
set(EM_FLAGS
    "-sUSE_SDL=2"
    "-sWASM=1"
    "-sALLOW_MEMORY_GROWTH=1"
    "-sEXIT_RUNTIME=1"
    "-sFULL_ES2=1"
    "-sOFFSCREEN_FRAMEBUFFER=1"
    "-sENVIRONMENT=web"
)

# Append flags
add_compile_options(${EM_FLAGS})
link_libraries(${EM_FLAGS})

# Add your source files
add_executable(main
    main.cpp
    drawCircle.cpp
    fish.cpp
    SDL2_gfxPrimitives.o # Precompiled SDL2_gfx from emcc
)
