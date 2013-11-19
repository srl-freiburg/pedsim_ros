#
# this module look for qwt (http://hdf.ncsa.uiuc.edu) support
# it will define the following values
#
# QWT_INCLUDE_DIR  = where qwt.h can be found
# QWT_LIBRARY      = the library to link against qwt
# FOUND_QWT        = set to true after finding the library
#

if(NOT QT4_FOUND)
  include(FindQt4)
endif(NOT QT4_FOUND)

set(QWT_FOUND NO)

if(QT4_FOUND)
  find_path(QWT_INCLUDE_DIR qwt.h
    PATHS /usr/local/qwt /usr/local /usr/include/qwt-qt4 /usr/include/qwt
          /usr/include/qwt5 /usr /opt/local/include/qwt
    HINTS $ENV{QWT_ROOT} ${QWT_ROOT}
    PATH_SUFFIXES include
  )

  set(QWT_NAMES ${QWT_NAMES} qwt-qt4 qwt5 qwt libqwt-qt4 libqwt)
  find_library(QWT_LIBRARY
    NAMES ${QWT_NAMES}
    PATHS /usr/local/qwt /usr/local /usr $ENV{QWT_ROOT} ${QWT_ROOT}
    PATH_SUFFIXES lib
  )

  if(QWT_LIBRARY)
    set(QWT_LIBRARIES ${QWT_LIBRARY})
    set(QWT_FOUND YES)

  endif(QWT_LIBRARY)
endif(QT4_FOUND)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(QWT DEFAULT_MSG QWT_LIBRARIES)

mark_as_advanced(QWT_INCLUDE_DIR QWT_LIBRARY)
