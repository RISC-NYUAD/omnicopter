#
# libkdtp autoconf file
#
AC_INIT([libkdtp],[1.3],[openrobots@laas.fr])

AC_CONFIG_AUX_DIR([autoconf])
AC_CONFIG_MACRO_DIR([autoconf])
AC_CONFIG_HEADERS([autoconf/ackdtp.h])
AM_INIT_AUTOMAKE([foreign no-define subdir-objects])

# compilers and tools
AC_PROG_INSTALL
AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_CXX
LT_INIT

# output
AC_CONFIG_FILES([
        libkdtp.pc
        Makefile
])
AC_OUTPUT
