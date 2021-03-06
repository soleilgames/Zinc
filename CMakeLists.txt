cmake_minimum_required (VERSION 3.5)

project(zinc-osg)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -W -Wall -Wextra")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# set(OSG_DIR /home/florian/jeux/OpenSceneGraph/build/INST)
find_package(OpenSceneGraph REQUIRED osgDB osgUtil osgViewer osgGA osgParticle osgAnimation) # NO_CMAKE_PATH # osgDB osgUtil

# Used by cloud example:
find_package(OpenGL REQUIRED)
# find_package(GLUT REQUIRED)


# find_package(OpenGL REQUIRED)
# find_library(GLEW_LIB GLEW)
# find_package(X11 REQUIRED)
# find_package (Threads)

# set(OSG_DIR /home/florian/jeux/OpenSceneGraph/build.static/INST)
# set(OPENSCENEGRAPH_INCLUDE_DIRS ${OSG_DIR}/include)
# set(OPENSCENEGRAPH_LIBRARIES_DIR ${OSG_DIR}/lib64)
# set(OPENSCENEGRAPH_PLUGINS_DIR ${OSG_DIR}/lib64/osgPlugins-3.5.1/)
# set(OPENSCENEGRAPH_LIBRARIES
#   ${OPENSCENEGRAPH_PLUGINS_DIR}/libosgdb_ive.a
#   ${OPENSCENEGRAPH_PLUGINS_DIR}/libosgdb_openflight.a
#   ${OPENSCENEGRAPH_PLUGINS_DIR}/libosgdb_osg.a
#   ${OPENSCENEGRAPH_PLUGINS_DIR}/libosgdb_rgb.a
#   ${OPENSCENEGRAPH_PLUGINS_DIR}/libosgdb_serializers_osg.a

#   ${OPENSCENEGRAPH_PLUGINS_DIR}/libosgdb_osg.a
#   ${OPENSCENEGRAPH_LIBRARIES_DIR}/libosg.a
#   ${OPENSCENEGRAPH_LIBRARIES_DIR}/libosgViewer.a
#   ${OPENSCENEGRAPH_LIBRARIES_DIR}/libosgGA.a
#   ${OPENSCENEGRAPH_LIBRARIES_DIR}/libosgDB.a
#   ${OPENSCENEGRAPH_LIBRARIES_DIR}/libosgUtil.a
#   ${OPENSCENEGRAPH_PLUGINS_DIR}/libosgdb_obj.a
#   ${OPENSCENEGRAPH_LIBRARIES_DIR}/libOpenThreads.a

#   ${OPENSCENEGRAPH_PLUGINS_DIR}/libosgdb_osg.a

#   ${OPENSCENEGRAPH_PLUGINS_DIR}/libosgdb_deprecated_osg.a

#   # Required by OSG
#   ${CMAKE_THREAD_LIBS_INIT}
#   ${CMAKE_DL_LIBS}
#   ${X11_LIBRARIES}
#   ${OPENGL_LIBRARIES}
#   ${X11_Xrandr_LIB} 
#   )

# message("TODO check: ${X11_Xrandr_FOUND}")

include_directories(
  ${OPENSCENEGRAPH_INCLUDE_DIRS}
  imgui/

  ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS}
  )


SET(ZINC_ENGINE_SRC
  easing.c
  SkyBox.cpp
  Logger.cpp
  ShootTracer.cpp
  SceneManager.cpp
  EventManager.cpp
  Time.cpp
  EntityGroup.cpp
  Actor.cpp
  AlienCraft.cpp
  ParticleObjects.cpp
  VolumetricFog.cpp

  ImGuiHandler.cpp
  imgui/imgui.cpp
  imgui/imgui_draw.cpp
  imgui/imgui_demo.cpp
  #CloudBlock.cpp
  )

add_executable(zinc-osg
  main.cpp

  ${ZINC_ENGINE_SRC}
  )


target_link_libraries(zinc-osg
  ${OPENSCENEGRAPH_LIBRARIES}
  ${OPENGL_LIBRARIES}
  )



add_executable(steering
  test/steering.cpp

  ${ZINC_ENGINE_SRC}
  )
target_include_directories(steering PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(steering
  ${OPENSCENEGRAPH_LIBRARIES}
  ${OPENGL_LIBRARIES}
  )


add_executable(volumetricfog
  test/volumetricfog.cpp

  ${ZINC_ENGINE_SRC}
  )
target_include_directories(volumetricfog PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(volumetricfog
  ${OPENSCENEGRAPH_LIBRARIES}
  ${OPENGL_LIBRARIES}
  )



add_executable(infinity
  test/infinity.cpp

  ${ZINC_ENGINE_SRC}
  )
target_include_directories(infinity PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(infinity
  ${OPENSCENEGRAPH_LIBRARIES}
  ${OPENGL_LIBRARIES}
  )







# add_executable(clouds
#   test/clouds.cpp

#   ${ZINC_ENGINE_SRC}
#   )
# target_include_directories(clouds PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# target_link_libraries(clouds
#   ${OPENSCENEGRAPH_LIBRARIES}

#   ${OPENGL_LIBRARIES} ${GLUT_LIBRARY}
#   )
