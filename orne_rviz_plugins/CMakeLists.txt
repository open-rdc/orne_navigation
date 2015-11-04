cmake_minimum_required(VERSION 2.8.3)
project(orne_rviz_plugins)

find_package(catkin REQUIRED COMPONENTS
  rviz
)

catkin_package()

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
    include/orne_rviz_plugins/state_trigger_panel.h
    src/teleop_panel.h
    src/drive_widget.h
)

set(SOURCE_FILES
    src/state_trigger_panel.cpp
    src/teleop_panel.cpp
    src/drive_widget.cpp
    ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(DIRECTORY media/
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media
#)

#install(DIRECTORY icons/
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons
#)

