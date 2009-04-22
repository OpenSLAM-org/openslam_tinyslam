check_PROGRAMS = test_SDL test_loop_closing

test_SDL_SOURCES = test_loop_closing.c
test_SDL_LDADD = ${top_builddir}/libCoreSLAM.la -lSDL -lSDL_gfx -lm
test_SDL_LDFLAGS = -no-install 
test_SDL_CFLAGS  = -DUSE_SDL -I${top_srcdir}/src

test_loop_closing_SOURCES = test_loop_closing.c
test_loop_closing_LDADD = ${top_builddir}/libCoreSLAM.la -lm
test_loop_closing_LDFLAGS = -no-install
test_loop_closing_CFLAGS  = -I${top_srcdir}/src

EXTRA_DIST= CMakeLists.txt
