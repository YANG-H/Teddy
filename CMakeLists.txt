cmake_minimum_required (VERSION 2.8.12)

# project name
set (PROJECT_NAME Teddy)

set (CMAKE_ALLOW_LOOSE_CONSTRUCTS true)


project (${PROJECT_NAME})
enable_testing ()

# set cmake module paths
list (APPEND CMAKE_MODULE_PATH 
	${CMAKE_CURRENT_SOURCE_DIR}/cmake/
)

### compiler setting ###
if (NOT DEFINED MSVC)
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")
endif ()

if (MSVC)
	add_definitions ( "/D_USE_MATH_DEFINES")
	add_definitions ( "/DQT_NO_OPENGL_ES_2")
endif ()


# locate Qt
set (QtDir "" CACHE PATH "Qt root directory")
list (APPEND CMAKE_PREFIX_PATH ${QtDir})

set(CMAKE_AUTOMOC on)
set(CMAKE_AUTOUIC on)
set(CMAKE_AUTORCC on)
set(CMAKE_INCLUDE_CURRENT_DIR on)

find_package (Qt5Widgets REQUIRED)
find_package (Qt5OpenGL REQUIRED)

# locate OpenMesh
find_package (OpenMesh REQUIRED)

# locate GLU and GLUT
find_package (GLUT REQUIRED)
find_package (GLU REQUIRED)

set(REQUIRED_INCLUDES ${OPENMESH_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
set(REQUIRED_LIBS Qt5::Widgets Qt5::OpenGL ${OPENMESH_LIBRARIES} ${GLUT_glut_LIBRARY})
set (REQUIRED_PATHS "")

foreach (i ${REQUIRED_INCLUDES})
	message (STATUS "REQUIRED_INCLUDES: " ${i})
endforeach ()

foreach (i ${REQUIRED_LIBS})
	message (STATUS "REQUIRED_LIBS: " ${i})
endforeach ()

foreach (i ${REQUIRED_PATHS})
	message (STATUS "REQUIRED_PATHS: " ${i})
endforeach ()



# the libs
add_subdirectory (src lib)

