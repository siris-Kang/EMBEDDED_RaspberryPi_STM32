cmake_minimum_required(VERSION 3.19)
project(qtRCcar LANGUAGES CXX)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets Mqtt)

qt_standard_project_setup()

qt_add_executable(qtRCcar
    WIN32 MACOSX_BUNDLE
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    app_types.h drawcamera.h drawmap.h gaugewidget.h mainwindow.h mymqtt.h
    drawcamera.cpp drawmap.cpp gaugewidget.cpp main.cpp mainwindow.cpp mymqtt.cpp
)

target_link_libraries(qtRCcar
    PRIVATE
        Qt::Core
        Qt::Widgets
        Qt6::Mqtt
)

include(GNUInstallDirs)

install(TARGETS qtRCcar
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET qtRCcar
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})
