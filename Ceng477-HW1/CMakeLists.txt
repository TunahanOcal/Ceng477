cmake_minimum_required(VERSION 3.15)
project(Ceng477_Hw1_2019)

set(CMAKE_CXX_STANDARD 11)

include_directories(.)

add_executable(Ceng477_Hw1_2019
        inputs/input01.xml
        inputs/input02.xml
        inputs/input03.xml
        inputs/input04.xml
        inputs/input05.xml
        Camera.cpp
        Camera.h
        defs.h
        Image.cpp
        Image.h
        Light.cpp
        Light.h
        main.cpp
        Material.cpp
        Material.h
        Ray.cpp
        Ray.h
        Scene.cpp
        Scene.h
        Shape.cpp
        Shape.h
        tinyxml2.cpp
        tinyxml2.h)
