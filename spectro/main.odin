package spectro
// NOTE: See fft/fft.odin algorithm comment block for optimization roadmap
// (radix-2 Cooley–Tukey with unrolled early stages; further SIMD/split-radix TBD).

import "core:flags"
import "core:fmt"
import "core:thread"
import "core:sync"
import "core:slice"
import "core:time"
import "core:os"

import "base:runtime"
import rl "vendor:raylib"

import "../vendor/rtlsdr"
import "../vendor/kissfft/"

import "utils"
import "fft"

rtlsdr_dev :: rtlsdr.rtlsdr_dev

SharedData :: struct {
    mutex: sync.Mutex, // Mutex for thread safety
    u64_values: map[string]u64 // Example of a map to store additional data
}

App :: struct { // Mutex for thread safety
    shared_data: SharedData,
    dev: ^rtlsdr_dev
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
    rtlsdr_set_center_freq(dev, utils.mhz_to_hz(center_freq_mhz))
    rtlsdr_reset_buffer(dev)

    return dev, error
}

tune_mhz :: proc(dev: ^rtlsdr_dev, freq_mhz: f64) {
    freq_hz := utils.mhz_to_hz(freq_mhz)
    rtlsdr.rtlsdr_set_center_freq(dev, freq_hz)
    fmt.println("tuned to", freq_hz)
}

on_samples :: proc "c" (buf: ^u8, size: u32, ctx: rawptr) {
    // FLOW: raw interleaved uint8 IQ -> (A) KISS FFT timing -> (B) convert to Complex32
    // -> build plan -> custom FFT timing -> print first bins -> exit.
    // Exits after first processed block for benchmarking.
    context = runtime.default_context()
    kb := size / 1024
    app := cast(^App)ctx
    // fmt.println("got:", size, "bytes", "(", kb, "KB)")
    buffer_slice := slice.from_ptr(buf, int(size))
    first_few_samples := buffer_slice[:min(10, size)]
    // fmt.println("First few samples:", first_few_samples)
    current_total := get_shared_u64("total_samples")
    set_shared_u64("total_samples", current_total + u64(size))
    total := get_shared_u64("total_samples")
    total_mb := total / 1024000
    // fmt.println("Total samples processed:", total, "(", total_mb, "MB)")

    freq_hz := rtlsdr.rtlsdr_get_center_freq(app.dev)
    fmt.println("center freq:", freq_hz)

    // Convert u8 buffer to kiss_fft_cpx
    ffi_in := kissfft.u8_to_kiss_fft_cpx(buf, size)
    start_time := time.now()
    // fmt.println(len(ffi_in), "complex samples")
    // fmt.println("First complex samples:", ffi_in[:10])
    // fmt.println("Converting u8 to kiss_fft_cpx took:", time.duration_milliseconds(time.since(start_time)), "ms")

    // Create kiss fft cfg with correct size
    fft_size := 4096
    if len(ffi_in) >= fft_size {
        fft_cfg := kissfft.kiss_fft_alloc(i32(fft_size), 0, nil, nil)
        
        // Perform FFT on first chunk
        output := make([]kissfft.kiss_fft_cpx, fft_size)
        start_time = time.now()
        kissfft.kiss_fft(fft_cfg, raw_data(ffi_in[:fft_size]), raw_data(output))
        fmt.println("KISS TIME:", time.duration_milliseconds(time.since(start_time)), "ms")
        fmt.println("KISS OUTPUT:", output[:10])
    }

    // Perform FFT with our own implementation
    samples := fft.cu8_interleaved_to_cf32(buffer_slice)
    n_samples := len(samples)
    
    // Only process the first fft_size samples to match our plan
    if n_samples >= fft_size {
        fft_plan := fft.fft_make_plan(fft_size, 0)
        samples_chunk := samples[:fft_size]// Copy to ensure we have a slice of the correct size
        
        // Test without window first
        // samples_no_window := make([]fft.Complex32, fft_size)
        // copy(samples_no_window, samples_chunk)
        // fft.fft_inplace(&fft_plan, samples_no_window)
        // fmt.println("First few samples (ours, no window):", samples_no_window[:10])
        
        // Apply Hann window
        start_time = time.now()
        // fft.fft_apply_hann(fft_plan.window, samples_chunk)
        
        fft.fft_inplace(&fft_plan, samples_chunk)
        fmt.println("OURS TIME:", time.duration_milliseconds(time.since(start_time)), "ms")
        first_ours := samples_chunk[:10]
        fmt.println("OURS OUTPUT:", first_ours)

        os.exit(0)
    }
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

    app.dev = dev


    recv_thread := thread.create_and_start_with_poly_data2(dev, chunk_size, fn = recv_worker)
    fmt.println("Receiving samples...\n")

    time.sleep(time.Second)
    tune_mhz(dev, 102.1)
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