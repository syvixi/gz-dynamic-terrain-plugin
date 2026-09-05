cmake_minimum_required(VERSION 3.16)
project(gz_dynamic_terrain_plugin VERSION 0.19.0 LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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

add_library(gz-dynamic-terrain-system SHARED src/DynamicTerrainSystem.cc)
target_include_directories(gz-dynamic-terrain-system PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(gz-dynamic-terrain-system PUBLIC gz-dynamic-terrain-core gz-sim8::gz-sim8)

set_target_properties(gz-dynamic-terrain-system PROPERTIES
    OUTPUT_NAME "gz-dynamic-terrain-system"
    INSTALL_RPATH "$ORIGIN"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

target_link_libraries(gz-dynamic-terrain-core PRIVATE
    gz-sim8::gz-sim8
    gz-rendering8::ogre2
    PkgConfig::OGRE2
    CURL::libcurl
    ${OpenCV_LIBS}
)

target_include_directories(gz-dynamic-terrain-core PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)
target_include_directories(gz-dynamic-terrain-core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

add_library(DynamicTerrainGui SHARED src/DynamicTerrainGui.cc src/DynamicTerrainGui.qrc)
target_include_directories(DynamicTerrainGui PRIVATE ${OpenCV_INCLUDE_DIRS})
set_target_properties(DynamicTerrainGui PROPERTIES AUTORCC ON INSTALL_RPATH "$ORIGIN")
target_link_libraries(DynamicTerrainGui PRIVATE gz-dynamic-terrain-core
    gz-gui8::gz-gui8 gz-sim8::gz-sim8 Qt5::Quick)
if(UNIX AND NOT APPLE)
    target_link_options(DynamicTerrainGui PRIVATE "LINKER:-Bsymbolic-functions")
    target_link_options(gz-dynamic-terrain-system PRIVATE "LINKER:-Bsymbolic-functions")
endif()

set_target_properties(gz-dynamic-terrain-core PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
target_compile_options(gz-dynamic-terrain-core PRIVATE
    -Wall -Wextra -Wpedantic
)

if(BUILD_TESTING)
    add_executable(dynamic-terrain-gui-plugin-test tests/GuiPlugin_TEST.cc)
    target_include_directories(dynamic-terrain-gui-plugin-test PRIVATE src ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(dynamic-terrain-gui-plugin-test PRIVATE gz-dynamic-terrain-core
        gz-gui8::gz-gui8 gz-sim8::gz-sim8 Qt5::Quick)
    add_test(NAME terrain_gui_plugin COMMAND dynamic-terrain-gui-plugin-test
        $<TARGET_FILE_DIR:DynamicTerrainGui>)
    set_tests_properties(terrain_gui_plugin PROPERTIES TIMEOUT 30
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen;QT_QUICK_BACKEND=software"
        RESOURCE_LOCK ogre2)
    add_executable(dynamic-terrain-gui-test tests/GuiTerrain_TEST.cc)
    target_include_directories(dynamic-terrain-gui-test PRIVATE src ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(dynamic-terrain-gui-test PRIVATE gz-dynamic-terrain-system gz-sim8::gz-sim8)
    add_test(NAME terrain_gui_bridge COMMAND dynamic-terrain-gui-test)
    set_tests_properties(terrain_gui_bridge PROPERTIES TIMEOUT 30)
    add_executable(dynamic-terrain-types-test
        tests/TerrainTypes_TEST.cc
    )
    target_include_directories(dynamic-terrain-types-test PRIVATE src)
    target_link_libraries(dynamic-terrain-types-test PRIVATE
        gz-dynamic-terrain-system
        gz-sim8::gz-sim8
    )
    add_test(NAME terrain_types COMMAND dynamic-terrain-types-test)

    add_executable(dynamic-terrain-builder-test
        tests/PersistentTerrainBuilder_TEST.cc
    )
    target_include_directories(dynamic-terrain-builder-test PRIVATE
        src
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(dynamic-terrain-builder-test PRIVATE
        gz-dynamic-terrain-system
        gz-sim8::gz-sim8
        ${OpenCV_LIBS}
    )
    add_test(NAME terrain_builder_mapping COMMAND dynamic-terrain-builder-test)

    add_executable(dynamic-terrain-ogre2-lifecycle-test
        tests/Ogre2TextureLifecycle_TEST.cc
    )
    target_include_directories(dynamic-terrain-ogre2-lifecycle-test PRIVATE
        src
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(dynamic-terrain-ogre2-lifecycle-test PRIVATE
        gz-dynamic-terrain-system
        gz-sim8::gz-sim8
        gz-rendering8::ogre2
        PkgConfig::OGRE2
        ${OpenCV_LIBS}
    )
    add_test(NAME ogre2_resource_lifecycle
             COMMAND dynamic-terrain-ogre2-lifecycle-test)
    set_tests_properties(ogre2_resource_lifecycle PROPERTIES
        ENVIRONMENT "LIBGL_ALWAYS_SOFTWARE=1"
        RESOURCE_LOCK ogre2
        SKIP_RETURN_CODE 77
        TIMEOUT 30
    )

    add_test(
        NAME example_plugin_filenames
        COMMAND ${CMAKE_COMMAND}
            -DEXAMPLES_DIR=${CMAKE_CURRENT_SOURCE_DIR}/examples
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/CheckPluginFilenames.cmake
    )
endif()

install(TARGETS gz-dynamic-terrain-system gz-dynamic-terrain-core DynamicTerrainGui
    LIBRARY DESTINATION lib
)
