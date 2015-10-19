#add_executable (ball-in-maze tut_ball_in_maze.cpp world.cpp world.h )

#set_target_properties(ball-in-maze PROPERTIES COMPILE_FLAGS -fPIC)


#target_link_libraries(ball-in-maze p3util
#/local/soft/naoqi/naoqi-sdk-2.3.0.14-linux64/lib/libalvision.so
##/local/soft/naoqi/naoqi-sdk-2.1.3.3-linux64/lib/libalvision.so
#        Panda3D::p3framework
#        Panda3D::panda
#        Panda3D::pandafx
#        Panda3D::pandaexpress
#        Panda3D::p3dtool
#        Panda3D::p3dtoolconfig
#        Panda3D::p3pystub
#        Panda3D::p3direct
#${CMAKE_THREAD_LIBS_INIT}
#${VISP_LIBRARIES}

#)


qi_create_bin(ball-in-maze tut_ball_in_maze.cpp world.cpp world.h)
set_target_properties(ball-in-maze PROPERTIES COMPILE_FLAGS -fPIC)

qi_use_lib(ball-in-maze visp_naoqi p3util ALCOMMON ALPROXIES ALVISION )
target_link_libraries(ball-in-maze
        Panda3D::p3framework
        Panda3D::panda
        Panda3D::pandafx
        Panda3D::pandaexpress
        Panda3D::p3dtool
        Panda3D::p3dtoolconfig
        Panda3D::p3pystub
        Panda3D::p3direct
${CMAKE_THREAD_LIBS_INIT}
)


