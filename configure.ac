AC_INIT([CoreSLAM],[0.0.1],[Bruno.Steux@ensmp.fr])
AC_CONFIG_SRCDIR([CoreSLAM.c])
AM_INIT_AUTOMAKE

dnl Checking compilation tools
AC_LANG_C
AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_CPP
AC_PROG_MAKE_SET

dnl Checking for headers
AC_HEADER_STDC

dnl Autotools specifics
AC_PROG_INSTALL
AC_PROG_LIBTOOL

dnl --------------------------------------------------------------------------
dnl Optional Core2 support
dnl --------------------------------------------------------------------------
AC_MSG_CHECKING([Core2 support])
AC_ARG_ENABLE([core2],AC_HELP_STRING([--enable-core2],[core2 support @<:@default=no@:>@]),[enable_core2=$enableval],[enable_core2=no])
if test "x$enable_core2" = xyes; then
   CFLAGS+=" -O3 -march=nocona -fomit-frame-pointer"
fi
AC_MSG_RESULT([$enable_core2])

dnl --------------------------------------------------------------------------
dnl Optional profiling support
dnl --------------------------------------------------------------------------
AC_MSG_CHECKING([profiling support])
AC_ARG_ENABLE([gprof],AC_HELP_STRING([--enable-gprof],[allow profiling @<:@default=no@:>@]),[enable_gprof=$enableval],[enable_gprof=no])
if test "x$enable_gprof" = xyes; then
   CFLAGS+=" -pg "
   LDFLAGS+=" -pg "
fi
AC_MSG_RESULT([$enable_gprof])

dnl --------------------------------------------------------------------------
dnl Optional code coverage support
dnl --------------------------------------------------------------------------
AC_MSG_CHECKING([code coverage support])
AC_ARG_ENABLE(gcov,AC_HELP_STRING([--enable-gcov],[allow test coverage @<:@default=no@:>@]),[enable_gcov=$enableval],[enable_gcov=no])
if test "x$enable_gcov" = xyes; then
   CFLAGS+=" -g -fprofile-arcs -ftest-coverage "
   LDFLAGS+=" -lgcov "
fi
AC_MSG_RESULT([$enable_gcov])

AC_CONFIG_HEADER([config.h])
AC_CONFIG_FILES(Makefile test/Makefile)

dnl generating output
AC_OUTPUT
