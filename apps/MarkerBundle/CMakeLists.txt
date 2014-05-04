set(the_description "The UbiTrack MarkerBundle app")
ut_add_app(MarkerBundle DEPS utcore utdataflow utfacade utvision)
ut_app_include_directories(${UBITRACK_CORE_DEPS_INCLUDE_DIR} ${OPENCV_INCLUDE_DIR} ${TBB_INCLUDE_DIR})
ut_glob_app_sources(SOURCES "*.cpp")
ut_create_executable(${OPENGL_LIBRARIES} ${Freeglut_LIBRARIES} ${PTHREAD_LIBRARIES})

#set_target_properties(${the_target} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:atlthunk.lib /NODEFAULTLIB:atlsd.lib /DEBUG")
# ut_module_include_directories(../../src ${CMAKE_CURRENT_BINARY_DIR}/../../src)
