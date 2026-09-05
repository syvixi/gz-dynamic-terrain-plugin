cmake_minimum_required(VERSION 3.16)
project(gz_dynamic_terrain_plugin VERSION 0.19.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD)
endif()

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-rendering8 REQUIRED COMPONENTS ogre2)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OGRE2 REQUIRED IMPORTED_TARGET OGRE-2.3)
find_package(CURL REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(Protobuf REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick Qml)

protobuf_generate_cpp(GUI_PROTO_SOURCES GUI_PROTO_HEADERS src/GuiTerrain.proto)

add_library(gz-dynamic-terrain-core SHARED
    src/TerrainTypes.cc
    src/TileStore.cc
    src/PersistentTerrain.cc
    src/CollisionTerrain.cc
    src/Ogre2ResourceCleanup.cc
    src/GuiTerrain.cc
    ${GUI_PROTO_SOURCES}
)

target_compile_features(gz-dynamic-terrain-core PRIVATE cxx_std_17)

target_include_directories(gz-dynamic-terrain-core
    PRIVATE ${OpenCV_INCLUDE_DIRS}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

target_link_libraries(gz-dynamic-terrain-core PRIVATE
    gz-sim8::gz-sim8
    gz-rendering8::ogre2
    PkgConfig::OGRE2
    CURL::libcurl
    ${OpenCV_LIBS}
)

target_compile_options(gz-dynamic-terrain-core PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

set_target_properties(gz-dynamic-terrain-core PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

add_library(gz-dynamic-terrain-system SHARED src/DynamicTerrainSystem.cc)

target_include_directories(gz-dynamic-terrain-system PRIVATE ${OpenCV_INCLUDE_DIRS})

target_link_libraries(gz-dynamic-terrain-system PUBLIC
    gz-dynamic-terrain-core
    gz-sim8::gz-sim8
)

set_target_properties(gz-dynamic-terrain-system PROPERTIES
    OUTPUT_NAME "gz-dynamic-terrain-system"
    INSTALL_RPATH "$ORIGIN"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

add_library(DynamicTerrainGui SHARED
    src/DynamicTerrainGui.cc
    src/DynamicTerrainGui.qrc
)

target_include_directories(DynamicTerrainGui PRIVATE ${OpenCV_INCLUDE_DIRS})

target_link_libraries(DynamicTerrainGui PRIVATE
    gz-dynamic-terrain-core
    gz-gui8::gz-gui8
    gz-sim8::gz-sim8
    Qt5::Quick
)

set_target_properties(DynamicTerrainGui PROPERTIES
    AUTORCC ON
    INSTALL_RPATH "$ORIGIN"
)

if(UNIX AND NOT APPLE)
    target_link_options(DynamicTerrainGui PRIVATE "LINKER:-Bsymbolic-functions")
    target_link_options(gz-dynamic-terrain-system PRIVATE "LINKER:-Bsymbolic-functions")
endif()

install(TARGETS
    gz-dynamic-terrain-system
    gz-dynamic-terrain-core
    DynamicTerrainGui
    LIBRARY DESTINATION lib
)
