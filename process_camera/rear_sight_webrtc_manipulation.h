//
// Created by biba_bo on 2020-08-21.
//

#ifndef IRON_TURTLE_REAR_SIGHT_WEBRTC_MANIPULATION_H
#define IRON_TURTLE_REAR_SIGHT_WEBRTC_MANIPULATION_H

#include "rear_sight_processor/image_processing.h"
#include "rear_sight_processor/rear_sight_processor.h"

#include <locale.h>
#include <glib.h>
#include <gst/gst.h>
#include <gst/sdp/sdp.h>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <iostream>

#ifdef G_OS_UNIX
#include <glib-unix.h>
#endif

#define STR_IP          "192.168.1.12"
#define STR_PORT        "5000"
#define STR_AUTO_MULTICAST "FALSE"


/** This .h file created in the C++ class style, but, cause some functions are called in other places, I cannot use a class structure,
 * so this one just initialize variables for processing a frame, and an object recognition object (here is also the thread for this processing).*/
void init_rear_sight_processor();

/// This function is manage a cropping process, and this one DECREASE a cropped image size
void on_zoom_plus_processor();

/// This function is manage a cropping process, and this one INCREASE a cropped image size
void on_zoom_minus_processor();

/// This function is manage a cropping process, and this one MOVE a cropped window to the LEFT side
void on_move_left_processor();

/// This function is manage a cropping process, and this one MOVE a cropped window to the RIGHT side
void on_move_right_processor();

/// This function is manage a cropping process, and this one MOVE a cropped window to the TOP side
void on_move_up_processor();

/// This function is manage a cropping process, and this one MOVE a cropped window to the BOTTOM side
void on_move_down_processor();

/// This function is sets a speed values for printing into pipeline
void set_speed_values_gst_pipeline_info(double currentLeftSpeed, double currentRightSpeed);

/// This function is callback function on an event of the data receiving on the videoconvert sink.
static GstPadProbeReturn cb_have_data(GstPad *pad, GstPadProbeInfo *info, gpointer user_data);

/// This function is starts a gst pipeline for sharing videostream
void create_and_run_pipeline();

#ifdef G_OS_UNIX
/// Exit function for linux system, when pipeline brakes
gboolean exit_sighandler (gpointer user_data);
#endif //G_OS_UNIX

/// This function starts gst loop for active approximately realtime video stream
int start_gst_loop();


/// This is a smart pointer to the FrameParameters - this is used for the cropping window.
static std::shared_ptr<FrameParameters> frame_param;
/// This is a smart pointer to the RearSightProcessor - this is used for modification a currant frame and calculating bounding box
static std::shared_ptr<RearSightProcessor> rear_sight_processor;
/// This is frame counter - this one is says us how many frames we should skip, before send a frame to the FormDetectionProcessor
static int count_frames;

/// This is old detected ROI - used for smoothing an behavior of the detection rectangle on the frame
static cv::Rect old_rectangle(0, 0, 0, 0);

/// this is an object with a speed value (here is not an actual speed, this is values for creating speed commands)
static std::string current_speed = "Speed LW=0, RW=0";

#endif //IRON_TURTLE_REAR_SIGHT_WEBRTC_MANIPULATION_H
