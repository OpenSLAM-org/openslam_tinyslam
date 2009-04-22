AUTOMAKE_OPTIONS = gnu

SUBDIRS = test

lib_LTLIBRARIES = libCoreSLAM.la

include_HEADERS = CoreSLAM.h 

libCoreSLAM_la_SOURCES = CoreSLAM.c CoreSLAM_ext.c CoreSLAM_random.c CoreSLAM_state.c CoreSLAM_loop_closing.c
libCoreSLAM_la_LDFLAGS = -no-undefined -version-info 0:0:0
libCoreSLAM_la_CFLAGS  = -I.

EXTRA_DIST = CMakeLists.txt
