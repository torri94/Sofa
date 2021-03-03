cmake_minimum_required(VERSION 3.12)
project(Dani VERSION 1.0 LANGUAGES CXX)

# Find and load CMake configuration of packages containing this plugin's dependencies
## Mandatory dependencies
find_package(SofaBase REQUIRED) # Dependency to SofaBaseVisual
## Optional dependencies
sofa_find_package(SofaGui) # Dependency to SofaGuiQt
sofa_find_package(Qt5 COMPONENTS Core) # Dependency to Qt5Core (needed for qt5_wrap_cpp)

# List all files
set(DANI_SRC_DIR src/Dani)
set(HEADER_FILES
    ${DANI_SRC_DIR}/config.h.in
    ${DANI_SRC_DIR}/MyBehaviorModel.h
    ${DANI_SRC_DIR}/MyVisualModel.h
    ${DANI_SRC_DIR}/EasyCollision1.h
    ${DANI_SRC_DIR}/EasyCollision2.h
    ${DANI_SRC_DIR}/EasyCollision3.h
    ${DANI_SRC_DIR}/EasyCollision4.h
    ${DANI_SRC_DIR}/EasyCollision5.h
    ${DANI_SRC_DIR}/EasyCollision6.h
    ${DANI_SRC_DIR}/EasyCollision7.h
    ${DANI_SRC_DIR}/EasyCollision8.h
    ${DANI_SRC_DIR}/EasyCollision9.h
    ${DANI_SRC_DIR}/EasyCollision10.h
    ${DANI_SRC_DIR}/EasyCollision11.h
    ${DANI_SRC_DIR}/EasyCollision12.h
    ${DANI_SRC_DIR}/EasyCollision13.h
    ${DANI_SRC_DIR}/EasyCollision14.h
    ${DANI_SRC_DIR}/EasyCollision15.h
    ${DANI_SRC_DIR}/EasyCollision16.h
    ${DANI_SRC_DIR}/EasyCollision17.h
    ${DANI_SRC_DIR}/EasyCollision18.h
    ${DANI_SRC_DIR}/EasyCollision19.h
    ${DANI_SRC_DIR}/EasyCollision20.h
    ${DANI_SRC_DIR}/EasyCollision21.h
    ${DANI_SRC_DIR}/EasyCollision22.h
    ${DANI_SRC_DIR}/EasyCollision23.h
    ${DANI_SRC_DIR}/EasyCollision24.h
    ${DANI_SRC_DIR}/EasyCollision25.h
    ${DANI_SRC_DIR}/EasyCollision26.h
    ${DANI_SRC_DIR}/EasyCollision27.h
    ${DANI_SRC_DIR}/EasyCollision28.h
    ${DANI_SRC_DIR}/EasyCollision29.h
    ${DANI_SRC_DIR}/EasyCollision30.h
    ${DANI_SRC_DIR}/EasyCollision31.h
    ${DANI_SRC_DIR}/EasyCollision32.h
    ${DANI_SRC_DIR}/EasyCollision33.h
    ${DANI_SRC_DIR}/EasyCollision34.h
    ${DANI_SRC_DIR}/EasyCollision35.h
    ${DANI_SRC_DIR}/EasyCollision36.h
    ${DANI_SRC_DIR}/EasyCollision37.h
    ${DANI_SRC_DIR}/EasyCollision38.h
    ${DANI_SRC_DIR}/EasyCollision39.h
    ${DANI_SRC_DIR}/EasyCollision40.h
    ${DANI_SRC_DIR}/EasyCollision41.h
    ${DANI_SRC_DIR}/EasyCollision42.h
    ${DANI_SRC_DIR}/EasyCollision43.h
    ${DANI_SRC_DIR}/EasyCollision44.h
    ${DANI_SRC_DIR}/EasyCollision45.h
    ${DANI_SRC_DIR}/EasyCollision46.h
    ${DANI_SRC_DIR}/EasyCollision47.h
    ${DANI_SRC_DIR}/EasyCollision48.h
    ${DANI_SRC_DIR}/EasyCollision49.h
    ${DANI_SRC_DIR}/EasyCollision50.h
    ${DANI_SRC_DIR}/MyMappingPendulumInPlane.h
    ${DANI_SRC_DIR}/MyMappingPendulumInPlane.inl
    ${DANI_SRC_DIR}/MyProjectiveConstraintSet.h
    ${DANI_SRC_DIR}/MyProjectiveConstraintSet.inl
    )
set(HEADER_FILES_TO_MOC
    )
set(SOURCE_FILES
    ${DANI_SRC_DIR}/initPluginExample.cpp
    ${DANI_SRC_DIR}/MyBehaviorModel.cpp
    ${DANI_SRC_DIR}/MyVisualModel.cpp
    ${DANI_SRC_DIR}/EasyCollision1.cpp
    ${DANI_SRC_DIR}/EasyCollision2.cpp
    ${DANI_SRC_DIR}/EasyCollision3.cpp
    ${DANI_SRC_DIR}/EasyCollision4.cpp
    ${DANI_SRC_DIR}/EasyCollision5.cpp
    ${DANI_SRC_DIR}/EasyCollision6.cpp
    ${DANI_SRC_DIR}/EasyCollision7.cpp
    ${DANI_SRC_DIR}/EasyCollision8.cpp
    ${DANI_SRC_DIR}/EasyCollision9.cpp
    ${DANI_SRC_DIR}/EasyCollision10.cpp
    ${DANI_SRC_DIR}/EasyCollision11.cpp
    ${DANI_SRC_DIR}/EasyCollision12.cpp
    ${DANI_SRC_DIR}/EasyCollision13.cpp
    ${DANI_SRC_DIR}/EasyCollision14.cpp
    ${DANI_SRC_DIR}/EasyCollision15.cpp
    ${DANI_SRC_DIR}/EasyCollision16.cpp
    ${DANI_SRC_DIR}/EasyCollision17.cpp
    ${DANI_SRC_DIR}/EasyCollision18.cpp
    ${DANI_SRC_DIR}/EasyCollision19.cpp
    ${DANI_SRC_DIR}/EasyCollision20.cpp
    ${DANI_SRC_DIR}/EasyCollision21.cpp
    ${DANI_SRC_DIR}/EasyCollision22.cpp
    ${DANI_SRC_DIR}/EasyCollision23.cpp
    ${DANI_SRC_DIR}/EasyCollision24.cpp
    ${DANI_SRC_DIR}/EasyCollision25.cpp
    ${DANI_SRC_DIR}/EasyCollision26.cpp
    ${DANI_SRC_DIR}/EasyCollision27.cpp
    ${DANI_SRC_DIR}/EasyCollision28.cpp
    ${DANI_SRC_DIR}/EasyCollision29.cpp
    ${DANI_SRC_DIR}/EasyCollision30.cpp
    ${DANI_SRC_DIR}/EasyCollision31.cpp
    ${DANI_SRC_DIR}/EasyCollision32.cpp
    ${DANI_SRC_DIR}/EasyCollision33.cpp
    ${DANI_SRC_DIR}/EasyCollision34.cpp
    ${DANI_SRC_DIR}/EasyCollision35.cpp
    ${DANI_SRC_DIR}/EasyCollision36.cpp
    ${DANI_SRC_DIR}/EasyCollision37.cpp
    ${DANI_SRC_DIR}/EasyCollision38.cpp
    ${DANI_SRC_DIR}/EasyCollision39.cpp
    ${DANI_SRC_DIR}/EasyCollision40.cpp
    ${DANI_SRC_DIR}/EasyCollision41.cpp
    ${DANI_SRC_DIR}/EasyCollision42.cpp
    ${DANI_SRC_DIR}/EasyCollision43.cpp
    ${DANI_SRC_DIR}/EasyCollision44.cpp
    ${DANI_SRC_DIR}/EasyCollision45.cpp
    ${DANI_SRC_DIR}/EasyCollision46.cpp
    ${DANI_SRC_DIR}/EasyCollision47.cpp
    ${DANI_SRC_DIR}/EasyCollision48.cpp
    ${DANI_SRC_DIR}/EasyCollision49.cpp
    ${DANI_SRC_DIR}/EasyCollision50.cpp
    ${DANI_SRC_DIR}/MyDataWidgetUnsigned.cpp
    ${DANI_SRC_DIR}/MyMappingPendulumInPlane.cpp
    ${DANI_SRC_DIR}/MyProjectiveConstraintSet.cpp
    )
set(README_FILES
    Dani.md
    )

if(Qt5Core_FOUND AND SofaGui_FOUND)
    message(STATUS "GUI components have been detected, enable GUI-related components")
    list(APPEND HEADER_FILES_TO_MOC
        ${DANI_SRC_DIR}/MyDataWidgetUnsigned.h
    )
    list(APPEND SOURCE_FILES
        ${DANI_SRC_DIR}/MyDataWidgetUnsigned.cpp
    )

    # [Qt] Create moc code.
    qt5_wrap_cpp(MOCCED_HEADER_FILES ${HEADER_FILES_TO_MOC})
endif()


# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES_TO_MOC} ${MOCCED_HEADER_FILES} ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

# Link the plugin library to its dependency(ies).
target_link_libraries(${PROJECT_NAME} SofaBaseVisual)
# Link with the optional GUI dependencies.
if(Qt5Core_FOUND AND SofaGui_FOUND)
    target_link_libraries(${PROJECT_NAME} SofaGuiQt)
endif()

# Create package Config, Version & Target files.
# Deploy the headers, resources, scenes & examples.
# Set the plugin 'relocatable' if built within SOFA.
# --> see SofaMacros.cmake
sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )

# Tests
# If SOFA_BUILD_TESTS exists and is OFF, then these tests will be auto-disabled
cmake_dependent_option(DANI_BUILD_TESTS "Compile the automatic tests" ON "SOFA_BUILD_TESTS OR NOT DEFINED SOFA_BUILD_TESTS" OFF)
if(DANI_BUILD_TESTS)
    enable_testing()
    add_subdirectory(Dani_test)
endif()
