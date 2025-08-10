package spectro

import "core:flags"
import "core:fmt"
import "core:thread"
import "core:sync"
import "base:runtime"
import rl "vendor:raylib"
import "../../rtlsdr-odin/rtlsdr"

rtlsdr_dev :: rtlsdr.rtlsdr_dev

SharedData :: struct {
    mutex: sync.Mutex, // Mutex for thread safety
    u64_values: map[string]u64 // Example of a map to store additional data
}

App :: struct { // Mutex for thread safety
    shared_data: SharedData,
}

get_shared_u64 :: proc (key: string) -> u64 {
    sync.mutex_lock(&app.shared_data.mutex)
    value := app.shared_data.u64_values[key]
    sync.mutex_unlock(&app.shared_data.mutex)
    return value
}

set_shared_u64 :: proc (key: string, value: u64) {
    sync.mutex_lock(&app.shared_data.mutex)
    app.shared_data.u64_values[key] = value
    sync.mutex_unlock(&app.shared_data.mutex)
}
app := App{
    shared_data = SharedData{
        mutex = sync.Mutex{}, // Initialize mutex
        u64_values = map[string]u64{}, // Initialize the map
    },
}

setup_radio :: proc (device_id: u32, rate_hz: u32, center_freq_mhz: f64) -> (^rtlsdr_dev, i32) {
    using rtlsdr
    dev: ^rtlsdr_dev
    error := rtlsdr_open(&dev, device_id)
    rtlsdr_set_center_freq(dev, mhz_to_hz(center_freq_mhz))
    rtlsdr_reset_buffer(dev)

    return dev, error
}

on_samples :: proc "c" (buf: ^u8, len: u32, ctx: rawptr) {
    context = runtime.default_context()
    app_ptr := cast(^App)ctx
    kb := len / 1024
    fmt.println("got:", len, "bytes", "(", kb, "KB)")
    current_total := get_shared_u64("total_samples")
    set_shared_u64("total_samples", current_total + u64(len))
    total := get_shared_u64("total_samples")
    total_mb := total / 1024000
    fmt.println("Total samples processed:", total, "(", total_mb, "MB)")

    // Now you can access app_ptr fields like: app_ptr.some_field
}

recv_worker :: proc (dev: ^rtlsdr_dev, chunk_size: u32) {
    using rtlsdr
    rtlsdr_read_async(dev, on_samples, &app, 0, chunk_size)
}

main :: proc() {
    using rl
    using rtlsdr
    sample_rate: u32 = 1024000
    chunk_size: u32 = 1024 * 128 // 16KB
    center_freq := 101.1

    dev, error := setup_radio(0, sample_rate, center_freq)
    if error != 0 {
        fmt.println("Failed to open RTL-SDR")
    } else {
        fmt.println("Opened SDR")
    }


    recv_thread := thread.create_and_start_with_poly_data2(dev, chunk_size, fn = recv_worker)
    fmt.println("Receiving samples...\n")
    thread.join(recv_thread)


    // fmt.println(dev)
    // InitWindow(1200, 600, "Spectro")
    // SetTargetFPS(60)

    // for !WindowShouldClose() {
    //     BeginDrawing()
    //     ClearBackground(RAYWHITE)
    //     DrawText("Hello", 10, 10, 10, RED)
    //     GuiButton(Rectangle{10, 20, 60, 20}, "Click me")
    //     EndDrawing()
    // }

}