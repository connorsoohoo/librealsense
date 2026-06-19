// License: Apache 2.0. See LICENSE file in root directory.
//
// rs-multicam-connor : headless RealSense multi-camera shared-memory publisher.
//
// Opens the COLOR stream of every connected (or selected) RealSense device once
// and writes each camera's latest frame into a per-serial memory-mapped file
// (/tmp/rs_<serial>.shm). Other processes -- e.g. a Python VLA policy -- can then
// read the freshest frame on demand WITHOUT claiming the USB device, so the heavy
// macOS init happens once and the model can be cycled on/off against a live feed.
//
// Synchronization uses a seqlock: the writer bumps a 64-bit counter to an odd
// value before writing and to the next even value after, so a reader can detect a
// torn read by checking the counter before and after its copy. No locks, no IPC
// daemon handshake.
//
// Build: part of the librealsense examples (see CMakeLists.txt). Does NOT depend
// on OpenGL/imgui, so it builds and runs headless.

#include <librealsense2/rs.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

// ---- Shared-memory layout (little-endian) ----------------------------------
//   offset 0  : uint32 magic
//   offset 4  : uint32 width
//   offset 8  : uint32 height
//   offset 12 : uint32 channels
//   offset 16 : uint64 seq            (odd = write in progress; even = stable)
//   offset 24 : double timestamp_ms   (camera hardware timestamp)
//   offset 32 : reserved
//   offset 64 : pixel data (width*height*channels bytes, RGB8)
static const uint32_t SHM_MAGIC   = 0x52534D31; // arbitrary tag, matched by reader
static const size_t   HEADER_SIZE = 64;
static const int      WIDTH       = 640;
static const int      HEIGHT      = 480;
static const int      CHANNELS    = 3;

struct ShmCamera
{
    std::string serial;
    std::string path;
    uint8_t*    base = nullptr;
    size_t      size = 0;
    int         fd   = -1;

    bool init(const std::string& s)
    {
        serial = s;
        path   = "/tmp/rs_" + s + ".shm";
        size   = HEADER_SIZE + static_cast<size_t>(WIDTH) * HEIGHT * CHANNELS;

        fd = ::open(path.c_str(), O_CREAT | O_RDWR, 0666);
        if (fd < 0) { std::cerr << "[ERROR] open " << path << " failed\n"; return false; }
        if (::ftruncate(fd, static_cast<off_t>(size)) != 0)
        {
            std::cerr << "[ERROR] ftruncate " << path << " failed\n";
            return false;
        }
        void* m = ::mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (m == MAP_FAILED) { std::cerr << "[ERROR] mmap " << path << " failed\n"; return false; }
        base = static_cast<uint8_t*>(m);

        // Initialize header (seq stays 0 = "no frame yet" until first write).
        auto* h = reinterpret_cast<uint32_t*>(base);
        h[0] = SHM_MAGIC;
        h[1] = WIDTH;
        h[2] = HEIGHT;
        h[3] = CHANNELS;
        *reinterpret_cast<uint64_t*>(base + 16) = 0;
        *reinterpret_cast<double*>(base + 24)   = 0.0;

        std::cout << "[INFO] Publishing " << serial << " -> " << path << "\n";
        return true;
    }

    void write_frame(const void* pixels, double ts_ms)
    {
        auto* seq = reinterpret_cast<uint64_t*>(base + 16);
        uint64_t s = *seq;
        __atomic_store_n(seq, s + 1, __ATOMIC_RELAXED); // mark write in progress (odd)
        __atomic_thread_fence(__ATOMIC_RELEASE);
        std::memcpy(base + HEADER_SIZE, pixels, static_cast<size_t>(WIDTH) * HEIGHT * CHANNELS);
        *reinterpret_cast<double*>(base + 24) = ts_ms;
        __atomic_thread_fence(__ATOMIC_RELEASE);
        __atomic_store_n(seq, s + 2, __ATOMIC_RELAXED); // mark stable (even)
    }

    void close_unmap()
    {
        if (base) ::munmap(base, size);
        if (fd >= 0) ::close(fd);
        base = nullptr;
        fd   = -1;
    }
};

static std::atomic<bool> g_run{true};
static void on_signal(int) { g_run = false; }

int main(int argc, char** argv) try
{
    int fps = 15;
    std::vector<std::string> want_serials;

    for (int i = 1; i < argc; ++i)
    {
        std::string a = argv[i];
        if (a == "--fps" && i + 1 < argc)         fps = std::atoi(argv[++i]);
        else if (a == "--serial" && i + 1 < argc) want_serials.push_back(argv[++i]);
        else if (a == "-h" || a == "--help")
        {
            std::cout << "Usage: rs-multicam-connor [--fps N] [--serial S]...\n"
                      << "  Publishes each camera's color stream to /tmp/rs_<serial>.shm\n"
                      << "  With no --serial, publishes every connected device.\n";
            return 0;
        }
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    rs2::context ctx;
    std::vector<std::string> serials;
    for (auto&& dev : ctx.query_devices())
        serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    if (!want_serials.empty())
    {
        std::vector<std::string> filtered;
        for (auto& s : serials)
            if (std::find(want_serials.begin(), want_serials.end(), s) != want_serials.end())
                filtered.push_back(s);
        serials.swap(filtered);
    }

    if (serials.empty())
    {
        std::cerr << "[ERROR] No RealSense devices found (matching the requested serials).\n";
        return EXIT_FAILURE;
    }

    std::vector<rs2::pipeline> pipes;
    std::map<std::string, ShmCamera> shms;

    for (auto& s : serials)
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(s);
        cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_RGB8, fps);
        pipe.start(cfg);
        pipes.push_back(pipe);

        ShmCamera cam;
        if (!cam.init(s)) return EXIT_FAILURE;
        shms[s] = cam;

        // Stagger camera startup to avoid USB power-negotiation crashes on macOS.
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }

    std::cout << "[INFO] Streaming " << serials.size() << " camera(s) at " << fps
              << " FPS (RGB8 " << WIDTH << "x" << HEIGHT << "). Press Ctrl+C to stop.\n";

    while (g_run)
    {
        bool any = false;
        for (size_t i = 0; i < pipes.size(); ++i)
        {
            rs2::frameset fs;
            if (pipes[i].poll_for_frames(&fs))
            {
                if (rs2::video_frame color = fs.get_color_frame())
                {
                    shms[serials[i]].write_frame(color.get_data(), color.get_timestamp());
                    any = true;
                }
            }
        }
        // Avoid a 100% busy-spin when no new frames are ready.
        if (!any) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "\n[INFO] Shutting down...\n";
    for (auto& p : pipes) p.stop();
    for (auto& kv : shms) kv.second.close_unmap();
    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "("
              << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
