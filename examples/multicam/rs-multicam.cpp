// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 RealSense, Inc. All Rights Reserved.

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include "example.hpp"              // Include short list of convenience functions for rendering

#include <chrono>
#include <map>
#include <thread>
#include <vector>

int main(int argc, char * argv[]) try
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 960, "CPP Multi-Camera Example");

    rs2::context                          ctx;        // Create librealsense context for managing devices

    std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)

    std::vector<rs2::pipeline>            pipelines;

    // Capture serial numbers before opening streaming.
    //
    // WHY the wait loop: rs2::context enumeration is ASYNCHRONOUS on the macOS
    // RSUSB backend. Constructing `ctx` kicks off USB discovery on a background
    // thread, but query_devices() returns whatever is registered AT THAT INSTANT.
    // Called immediately after the ctx constructor (as the stock example did),
    // it frequently races device bring-up and returns only the first camera that
    // happened to finish enumerating -- so a second device silently never enters
    // the loop below, and the startup stagger never runs for it. (rs-enumerate-
    // devices doesn't hit this: by the time a human runs it, the bus has settled,
    // and it queries once.) Here we poll until the device count holds steady for
    // a short settle window before snapshotting serials, so all connected cameras
    // are picked up regardless of enumeration order/latency.
    auto current_count = []( rs2::context& c ) {
        return static_cast<int>( c.query_devices().size() );
    };

    const auto   poll_interval = std::chrono::milliseconds(250);
    const int    stable_polls  = 6;    // count must hold for ~1.5 s (6 * 250 ms)
    const int    max_polls     = 40;   // give up waiting after ~10 s
    int last_count = -1, stable = 0;
    for (int i = 0; i < max_polls; ++i)
    {
        int n = current_count(ctx);
        if (n > 0 && n == last_count) { if (++stable >= stable_polls) break; }
        else                          { stable = 0; }
        last_count = n;
        std::this_thread::sleep_for(poll_interval);
    }

    std::vector<std::string>              serials;
    for (auto&& dev : ctx.query_devices())
        serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    std::cout << "Detected " << serials.size() << " device(s)" << std::endl;

    // Start a streaming pipe per each connected device
    for (auto&& serial : serials)
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(serial);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 15);
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        // Map from each device's serial number to a different colorizer
        colorizers[serial] = rs2::colorizer();

        // Stagger camera startup. WHY: all pipelines above share the single
        // rs2::context `ctx`, so they also share one libusb context and one USB
        // event/dispatcher thread. Calling pipe.start() back-to-back fires the
        // device-init sequence (control transfers, UVC stream negotiation,
        // firmware handshakes) for several cameras concurrently through those
        // shared, locked structures. On macOS (RSUSB backend) this races IOKit
        // USB power negotiation and enumeration: the host tries to bring up
        // multiple bus-powered cameras at once, and inits intermittently wedge
        // or fail to ever produce frames. Single-device examples like rs-capture
        // never hit this because each process drives one device in its own
        // context. The fixed pause serializes each camera's bring-up so power
        // delivery and enumeration settle before the next start() — this is the
        // main reason stock rs-multicam "struggles" to establish feeds while
        // rs-capture does not. ~1.5 s is conservative; lower it if bring-up is
        // reliable on your hardware.
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }

    // We'll keep track of the last frame of each stream available to make the presentation persistent
    std::map<int, rs2::frame> render_frames;

    // DIAGNOSTIC: per-device frame counters. WHY: when "Detected N device(s)"
    // is correct but only one device renders, we need to know whether the silent
    // device delivers ZERO frames (USB power/streaming-endpoint failure -> nothing
    // arrives from poll_for_frames) or delivers frames that fail to render (a
    // render/unique_id problem). Printing per-serial frame counts once a second
    // distinguishes the two. Remove once the cause is identified.
    std::map<std::string, uint64_t> frame_counts;
    for (auto& s : serials) frame_counts[s] = 0;
    auto last_report = std::chrono::steady_clock::now();

    // Main app loop
    while (app)
    {
        // Collect the new frames from all the connected devices
        std::vector<rs2::frame> new_frames;
        for (size_t i = 0; i < pipelines.size(); ++i)
        {
            rs2::frameset fs;
            if (pipelines[i].poll_for_frames(&fs))
            {
                frame_counts[serials[i]] += fs.size();
                for (const rs2::frame& f : fs)
                    new_frames.emplace_back(f);
            }
        }

        // DIAGNOSTIC: report per-device frame totals once a second.
        auto now = std::chrono::steady_clock::now();
        if (now - last_report >= std::chrono::seconds(1))
        {
            std::cerr << "[frames]";
            for (auto& s : serials) std::cerr << " " << s << "=" << frame_counts[s];
            std::cerr << std::endl;
            last_report = now;
        }

        // Convert the newly-arrived frames to render-friendly format
        for (const auto& frame : new_frames)
        {
            // Get the serial number of the current frame's device
            auto serial = rs2::sensor_from_frame(frame)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            // Apply the colorizer of the matching device and store the colorized frame
            render_frames[frame.get_profile().unique_id()] = colorizers[serial].process(frame);
        }

        // Present all the collected frames with openGl mosaic
        app.show(render_frames);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
