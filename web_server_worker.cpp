//
// Created by biba_bo on 2020-08-18.
//

#include "web_server_worker.h"

#include <zconf.h>


WebServerWorker::WebServerWorker() {
    std::shared_ptr<SerialManager> serialManager = std::make_shared<SerialManager>();
    // creating an object of TurtleManager with connecting throw COM-port
    turtle_manager = make_shared<SmoothTurtleManager>(serialManager);
    pursuit_turtle_processor = make_shared<PursuitProcessor>(serialManager);
    set_pursuit_processor(pursuit_turtle_processor);

    // starting the iron turtle moving processing thread
    // by default the iron turtle moves by custom commands
    turtle_manager->restart_processing_thread();
    _isEnabledPursuitProcessing = false;

    // starting web-server
    this->startServer();
    // this thread sleeps for 1 seconds for avoiding bugs and errors
    sleep(1);
    // staring processing the server.
    this->processServer();
}

void WebServerWorker::startServer() {
    // creating a new thread for the web-server
    this->server_thread = std::thread([this]() {
        // creating a new object of PrintfLogger for possibility to log working process
        logger = std::make_shared<PrintfLogger>();
        // creating a new object of MyServer and adding here a logger
        ws_server = make_shared<MyServer>(logger);
        // creating a new object of MyHandler for handling events from web-client (onConnect, onDisconnect, onMessage)
        handler = std::make_shared<MyHandler>(ws_server.get());
        // adding a page handler for working with web-pages
        ws_server->addPageHandler(std::make_shared<MyAuthHandler>());
        // added a socket handler
        ws_server->addWebSocketHandler("/chart", handler);
        // starting webserver
        ws_server->serve("src/server_files", 56778);
    });
}

void WebServerWorker::processServer() {
    // still nothing works here
}

void WebServerWorker::joinServerTread() {
    // like a feature for avoiding bugs
    if (server_thread.joinable())
        server_thread.join();
}

void WebServerWorker::handleEventWS(std::shared_ptr<EventWS> event) {
    // handling events by the event code
    switch (event->getEventID()) {
        case EVENT_MOVE_FORWARD:
            if (_isEnabledPursuitProcessing)
                pursuit_turtle_processor->resume_moving();
            else
                turtle_manager->move_forward();
            break;
        case EVENT_MOVE_BACK:
            if (_isEnabledPursuitProcessing)
                pursuit_turtle_processor->resume_moving();
            else
                turtle_manager->move_backward();
            break;
        case EVENT_MOVE_LEFTER:
            if (_isEnabledPursuitProcessing) {
                //pursuit_turtle_processor->resume_moving();
                pursuit_turtle_processor->add_aim_for_processing(std::pair<float, float>(0.52, 0));
            } else
                turtle_manager->move_lefter();
            break;
        case EVENT_MOVE_RIGHTER:
            if (_isEnabledPursuitProcessing) {
                //pursuit_turtle_processor->resume_moving();
                pursuit_turtle_processor->add_aim_for_processing(std::pair<float, float>(-0.52, 0));
            } else
                turtle_manager->move_righter();
            break;
        case EVENT_STOP_MOVING:
            if (_isEnabledPursuitProcessing)
                pursuit_turtle_processor->stop_moving();
            else
                turtle_manager->stop_moving();
            break;

        case EVENT_ENABLE_PURSUIT:
            if (!_isEnabledPursuitProcessing) {
                turtle_manager->stop_processing_thread();
                pursuit_turtle_processor->restart_processing_thread();
                _isEnabledPursuitProcessing = true;
            }
            break;
        case EVENT_DISABLE_PURSUIT:
            if (_isEnabledPursuitProcessing) {
                pursuit_turtle_processor->stop_processing_thread();
                turtle_manager->restart_processing_thread();
                _isEnabledPursuitProcessing = false;
            }
            break;

        case EVENT_CAM_ZM:
            on_zoom_minus_processor();
            break;
        case EVENT_CAM_ZP:
            on_zoom_plus_processor();
            break;
        case EVENT_CAM_UP:
            on_move_up_processor();
            break;
        case EVENT_CAM_DOWN:
            on_move_down_processor();
            break;
        case EVENT_CAM_LEFT:
            on_move_left_processor();
            break;
        case EVENT_CAM_RIGHT:
            on_move_right_processor();
            break;
        case EVENT_CLIENT_CONNECTED:
            turtle_manager->say_server_here();
            break;
        case EVENT_CLIENT_DISCONNECTED:
            turtle_manager->say_server_leave();
            break;
    }
}
