#
# libkdtp automake file
#
ACLOCAL_AMFLAGS=-Iautoconf

# --- library --------------------------------------------------------------
#
lib_LTLIBRARIES= libkdtp.la

libkdtp_la_SOURCES= \
	src/libkdtp.h		\
	src/kdtp.h		\
				\
	src/dof.cc		\
	src/spline.cc		\
	src/localpath.cc	\
	src/poly.cc

libkdtp_la_CPPFLAGS= -I.
libkdtp_la_LDFLAGS= -release $(PACKAGE_VERSION)


# --- data -----------------------------------------------------------------
#
include_HEADERS= src/libkdtp.h

pkgconfigdir=	$(libdir)/pkgconfig
pkgconfig_DATA=	libkdtp.pc
