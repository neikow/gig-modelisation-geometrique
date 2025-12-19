cmake_minimum_required(VERSION 3.22)
project(gig_modelisation_geometrique)

set(CMAKE_CXX_STANDARD 20)

find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt6)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS io common visualization filters segmentation surface)

file(
        GLOB cppfiles
        RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
foreach (cppfile ${cppfiles})
    get_filename_component(target_name ${cppfile} NAME_WE)
    create_single_source_cgal_program("${cppfile}")
    target_include_directories(${target_name} PRIVATE ${PCL_INCLUDE_DIRS})
    target_link_libraries(${target_name} PRIVATE ${PCL_LIBRARIES})
    add_definitions(${PCL_DEFINITIONS})

    if (CGAL_Qt6_FOUND)
        target_link_libraries(${target_name} PRIVATE CGAL::CGAL_Basic_viewer)
    endif ()
endforeach ()
