INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/..)
LINK_DIRECTORIES(..)

if(CMAKE_HOST_UNIX)
set(libs m)
set(SDLlibs SDL SDL_gfx)
endif(CMAKE_HOST_UNIX)

if(MSVC)
set(SDLlibs SDL SDLmain libgfx)
endif(MSVC)

add_executable(test_SDL test_loop_closing.c)
set_target_properties(test_SDL PROPERTIES COMPILE_FLAGS -DUSE_SDL)
target_link_libraries(test_SDL CoreSLAM ${SDLlibs} ${libs})

add_executable(test_loop_closing test_loop_closing.c)
target_link_libraries(test_loop_closing CoreSLAM ${libs})

add_executable(calcul_frequence calcul_frequence.c)
target_link_libraries(calcul_frequence CoreSLAM ${libs})
