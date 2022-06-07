cmake_minimum_required(VERSION 2.8)
project( simulation )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O7")

#add_compile_options(-ftest-coverage -fprofile-arcs)    # for ut coverage
#add_compile_options(-Wall -Wextra -pedantic)    # for clean code
add_compile_options(
    #-fsanitize=address    # for memory check
    #-fno-omit-frame-pointer
    #-fsanitize=leak
    )
#    add_compile_options(clang-tidy -checks=' ')
#export LD_PRELOAD=/usr/lib/gcc/x86_64-linux-gnu/7/libasan.so

find_package(PkgConfig REQUIRED)
find_package( OpenGL REQUIRED)
find_package(GLEW REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
#link_directories(/usr/local/Cellar/glfw/3.2.1/lib)
find_package(GLUT REQUIRED)
find_package( X11 REQUIRED )

include_directories(${OPENGL_INCLUDE_DIRS} 
     ${GLUT_INCLUDE_DIR} 
     ${GLEW_INCLUDE_DIRS} 
     ${GLFW_INCLUDE_DIRS}
    header/include
    header/env/include
    header/utils/include
    )

add_executable( simulation 
    main.cpp
    #header/src/vec2D.cpp
    )

target_link_libraries(simulation 
        ${OPENGL_LIBRARY} 
        ${GLUT_LIBRARY} 
        ${GLFW_LIBRARIES}
        /usr/local/lib/libglfw3.a
        /usr/lib/x86_64-linux-gnu/libX11.so
        -lpthread
        -lgcov
        #-lasan
        ${CMAKE_DL_LIBS}
)
#target_compile_features(simulation PRIVATE cxx_range_for)
