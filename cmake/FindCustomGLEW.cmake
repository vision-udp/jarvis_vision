# Hack necceesary to get Glew work in OSX, taken from https://github.com/fran6co/homebrew-cv/issues/13
IF (APPLE)
  FIND_PATH( GLEW_INCLUDE_DIR GL/glew.h
    /usr/include/GL
    /usr/openwin/share/include
    /usr/openwin/include
    /usr/X11R6/include
    /usr/include/X11
    /opt/graphics/OpenGL/include
    /opt/graphics/OpenGL/contrib/libglew
  )

  FIND_LIBRARY( GLEW_GLEW_LIBRARY GLEW
    /usr/openwin/lib
    /usr/X11R6/lib
  )
ELSE (APPLE)
  find_package(GLEW REQUIRED)
ENDIF (APPLE)

