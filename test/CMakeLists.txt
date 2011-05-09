INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/..)
LINK_DIRECTORIES(..)

if(CMAKE_HOST_UNIX)
set(libs m)
endif(CMAKE_HOST_UNIX)

OPTION(USE_SDL "Use SDL for visualization" ON)
IF(USE_SDL)
    ADD_DEFINITIONS(-DUSE_SDL)
    IF(MSVC)
        FIND_LIBRARY(SDLLIB NAMES SDL PATHS "../third_parties/SDL/lib")
        FIND_LIBRARY(SDLmainLIB NAMES SDLmain PATHS "../third_parties/SDL/lib")
        FIND_LIBRARY(SDLGFXLIB NAMES SDL_gfx PATHS "../third_parties/SDL_gfx/lib")
    ENDIF(MSVC)
    IF(CMAKE_HOST_UNIX)
        FIND_PACKAGE(SDL REQUIRED)
        FIND_LIBRARY(SDLGFXLIB NAMES SDL_gfx PATHS "/usr/local/lib")
    ENDIF(CMAKE_HOST_UNIX)
ELSE(USE_SDL)
    SET(SDLGFXLIB "")
    SET(SDLLIB "")
    SET(SDLmainLIB "")
ENDIF(USE_SDL)

add_executable(test_SDL test_loop_closing.c)
target_link_libraries(test_SDL CoreSLAM ${SDL_LIBRARY} ${SDLLIB} ${SDLmainLIB}
    ${SDLGFXLIB} ${libs})

add_executable(test_loop_closing test_loop_closing.c)
target_link_libraries(test_loop_closing CoreSLAM ${libs})


