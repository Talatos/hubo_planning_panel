cmake_minimum_required(VERSION 2.8.3)
project(hubo_planning_panel)

find_package( catkin REQUIRED )

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp rospy
    DEPENDS ach QtCore QtGui )

qt4_wrap_cpp(MOC_FILES
  include/hubo_planning.h
  include/FlowLayout.h
)

set(SOURCE_FILES
  src/hubo_planning.cpp
  src/hubo_planning_slots.cpp
  src/FlowLayout.cpp
  include/hubo_planning.h
  include/FlowLayout.h
  ${MOC_FILES}
)

include_directories( include )

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library( ${PROJECT_NAME} ${SOURCE_FILES} )

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ach huboparams)

