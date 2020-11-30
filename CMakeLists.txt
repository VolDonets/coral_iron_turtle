cmake_minimum_required(VERSION 3.13)
project(coral_iron_turtle)

set(CMAKE_CXX_STANDARD 20)

include_directories(/home/mendel/pose_detection/tensorflow_special_commit/tensorflow/tensorflow/lite/tools/make/downloads/flatbuffers/include)
include_directories(/home/mendel/pose_detection/tensorflow_special_commit/tensorflow/tensorflow/lite/)
include_directories(/home/mendel/pose_detection/tensorflow_special_commit/tensorflow)

function(dep_edgetpu_tsl EXECUTABLE)
    target_link_libraries(${EXECUTABLE} /home/mendel/pose_detection/libedgetpu-local/libedgetpu/direct/aarch64/libedgetpu.so.1.0)
    target_link_libraries(${EXECUTABLE} /home/mendel/pose_detection/tensorflow_special_commit/tensorflow/tensorflow/lite/tools/make/gen/aarch64_armv8-a/lib/libtensorflow-lite.a)

    target_link_libraries(${EXECUTABLE} pthread)
    target_link_libraries(${EXECUTABLE} m)
    target_link_libraries(${EXECUTABLE} dl)
    target_link_libraries(${EXECUTABLE} rt)
endfunction()

add_executable(coral_iron_turtle
###      source file with the project begin function
        main.cpp

###      web server
        web_server/web_server.h
        web_server/web_server.cpp

###      class for the handling process in other thread for the WebServer events
        web_server_worker.h
        web_server_worker.cpp

###      my own handling sub system
        web_server/lib_my_event_handler/delegate_ws.h
        web_server/lib_my_event_handler/delegate_ws.cpp
        web_server/lib_my_event_handler/event_ws.h
        web_server/lib_my_event_handler/event_ws.cpp
        web_server/lib_my_event_handler/handler_ws.h

###      binary protocol wrapper
        turtle_manager/binary_com_manager/bipropellant-api/hbprotocol/protocol.h
        turtle_manager/binary_com_manager/bipropellant-api/hbprotocol/protocol_private.h
        turtle_manager/binary_com_manager/bipropellant-api/hbprotocol/ascii_protocol.h

        turtle_manager/binary_com_manager/bipropellant-api/hbprotocol/protocol.c
        turtle_manager/binary_com_manager/bipropellant-api/hbprotocol/machine_protocol.c
        turtle_manager/binary_com_manager/bipropellant-api/hbprotocol/ascii_protocol.c

        turtle_manager/binary_com_manager/bipropellant-api/protocolFunctions.h
        turtle_manager/binary_com_manager/bipropellant-api/protocolFunctions.c
        turtle_manager/binary_com_manager/bipropellant-api/HoverboardAPI.h
        turtle_manager/binary_com_manager/bipropellant-api/HoverboardAPI.cpp

        turtle_manager/binary_com_manager/binary_com_lim.h
        turtle_manager/binary_com_manager/serial_manager.h
        turtle_manager/binary_com_manager/serial_manager.cpp

        turtle_manager/smooth_turtle_manager.h
        turtle_manager/smooth_turtle_manager.cpp

        pursuit_processor/pursuit_processor.h
        pursuit_processor/pursuit_processor.cpp

###      this is the source code for the image processing in the gst pipeline
        process_camera/rear_sight_processor/image_processing.h
        process_camera/rear_sight_processor/rear_sight_processor.h
        process_camera/rear_sight_processor/rear_sight_processor_keep_frame_proportionality.cpp
        process_camera/rear_sight_webrtc_manipulation.cpp
        process_camera/rear_sight_webrtc_manipulation.h

###      this is the source code for the pose point detection with Google Coral TPU accelerator
        pose_processing/posenet/posenet_decoder.h
        pose_processing/posenet/posenet_decoder.cc
        pose_processing/posenet/posenet_decoder_op.h
        pose_processing/posenet/posenet_decoder_op.cc

        pose_processing/pose_param_detection_src/pose_param_engine.h
        pose_processing/pose_param_detection_src/pose_param_engine.cpp

        pose_processing/pose_detector_src/pose_detector_wrapper.h
        pose_processing/pose_detector_src/pose_detector_wrapper.cpp
)


dep_edgetpu_tsl(coral_iron_turtle)
target_compile_definitions(coral_iron_turtle PRIVATE MY_DEBUG_DEF)

target_compile_definitions(coral_iron_turtle PRIVATE UBUNTU_PC)
#target_compile_definitions(coral_iron_turtle PRIVATE RASPBERRY_PI)
#set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} -latomic")

###       this is a target compile definitions for the DEBUG development steps
target_compile_definitions(coral_iron_turtle PRIVATE MY_DEBUG)

###       Adding to the project the 'seasocks' library for the WebServer which works as WebSocket server
target_link_libraries(coral_iron_turtle seasocks "${ZLIB_LIBRARIES}")
###       Adding to the project
target_link_libraries(coral_iron_turtle pthread z)

###       Adding to the project the 'opencv' library it used for the frame processing (and, +- as an object detector) and as an object tracker
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(coral_iron_turtle ${OpenCV_LIBS})

find_package(PkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(JSON "json-glib-1.0")
    if (JSON_FOUND)
        target_link_libraries(coral_iron_turtle ${JSON_LIBRARIES})
        add_definitions(${JSON_CFLAGS} ${JSON_CFLAGS_OTHER})
    endif()
endif()

find_package(PkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(GST "gstreamer-1.0")
    if (GST_FOUND)
        target_link_libraries(coral_iron_turtle ${GST_LIBRARIES})
        add_definitions(${GST_CFLAGS} ${GST_CFLAGS_OTHER})
    endif()
endif()

find_package(PkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(SPD "gstreamer-sdp-1.0")
    if (SPD_FOUND)
        target_link_libraries(coral_iron_turtle ${SPD_LIBRARIES})
        add_definitions(${SPD_CFLAGS} ${SPD_CFLAGS_OTHER})
    endif()
endif()


###       Copying the source files for the runnable file
add_custom_command(TARGET coral_iron_turtle POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${PROJECT_SOURCE_DIR}/src $<TARGET_FILE_DIR:coral_iron_turtle>/src)