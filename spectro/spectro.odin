package spectro
// Visual spectral waterfall display for RTL-SDR
// Advanced implementation with dynamic FFT sizing, frequency tuning, and performance features

import "core:fmt"
import "core:math"
import "core:mem"
import "core:slice"
import "core:strconv"
import "core:strings"
import "core:sync"
import "core:thread"
import "core:time"

import "base:runtime"
import rl "vendor:raylib"

import "../../rtlsdr-odin/rtlsdr"
// import fft "../../offt"
import kissfft "../vendor/kissfft"
import "utils"

// Configuration constants
INIT_FFT_SIZE :: 2048
// MAX_FFT_SIZE :: 8192
HISTORY_ROWS :: 1024
BLOCK_SIZE :: 16384 // Number of samples per read
SAMPLE_RATE :: 2048000 // 2.048 MHz
CENTER_FREQ_HZ_DEFAULT :: 101100000 // 101.1 MHz
// CENTER_FREQ_HZ_DEFAULT :: 136975000 // 101.1 MHz
// CENTER_FREQ_HZ_DEFAULT :: 136975000 // 101.1 MHz
GAIN_TENTHSDB :: -1 // -1 for auto-gain
DEFAULT_OVERLAP_ENABLED :: true // Enable 50% FFT overlap by default

// Available FFT sizes
FFT_SIZES := [8]int{64, 128, 256, 512, 1024, 2048, 4096, 8192}

// Simple ring buffer for IQ samples
RingBuffer :: struct {
	buf:        []u8,
	head, tail: int,
	mutex:      sync.Mutex,
}

// Application state
App :: struct {
	dev:                  ^rtlsdr.rtlsdr_dev,
	ring_buffer:          RingBuffer,
	center_freq:          u32,

	// Dynamic FFT state
	fft_n:                int,
	// fft_plan:             fft.FFTPlan,
	fft_cfg:              kissfft.kiss_fft_cfg,
	hann_window:          []f32,

	// Processing buffers  
	iq_samples:           []kissfft.kiss_fft_cpx,
	fft_output:           []kissfft.kiss_fft_cpx,
	windowed:             []kissfft.kiss_fft_cpx,
	power_spectrum:       []f32,
	db_array:             []f32,
	db_tmp:               []f32, // for quickselect
	waterfall_line:       []rl.Color,

	// Texture resources
	ring_texture:         rl.Texture2D,
	lut_texture:          rl.Texture2D,

	// GUI state
	write_head:           int,
	frame_fill:           int,
	frame_counter:        int,

	// Frequency tuning
	freq_input:           [32]u8,
	freq_edit:            bool,
	pending_retune:       bool,
	cursor_set_retune: bool,

	// Frame skipping for waterfall speed
	frame_skip:           int,
	skip_counter:         int,

	// FFT size selection
	fft_size_index:       int,
	fft_dropdown_edit:    bool,

	// Performance timing
	fft_time_acc:         f64,
	tex_time_acc:         f64,
	fft_batches:          int,
	tex_batches:          int,
	stats_time_last:      f64,

	// Spectrum smoothing
	smoothing_alpha:      f32,
	spectrum_initialized: bool,

	// FFT overlap control
	overlap_enabled:      bool,
	prev_overlap_enabled: bool,

	// IQ correction parameters
	dc_i_avg:             f32, // Running average of DC offset for I
	dc_q_avg:             f32, // Running average of DC offset for Q
	dc_alpha:             f32, // EMA factor for DC tracking (0.995 = slow adaptation)

	// Thread management
	recv_thread:          ^thread.Thread,
	should_stop_recv:     bool,

	// Gui
	sidebar_width:        i32, // Width of the sidebar for controls
}

app: App


// Initialize ring buffer
ring_buffer_init :: proc(rb: ^RingBuffer, size: int) {
	rb.buf = make([]u8, size)
	rb.head = 0
	rb.tail = 0
	rb.mutex = sync.Mutex{}
}

// Write to ring buffer (non-blocking, drops entire frame if won't fit)
ring_buffer_write :: proc(rb: ^RingBuffer, data: []u8) {
	sync.mutex_lock(&rb.mutex)
	defer sync.mutex_unlock(&rb.mutex)

	// Check if entire frame will fit
	available_space := (rb.tail - rb.head - 1 + len(rb.buf)) % len(rb.buf)
	if len(data) > available_space do return // drop entire frame

	// Write entire frame
	for byte in data {
		rb.buf[rb.head] = byte
		rb.head = (rb.head + 1) % len(rb.buf)
	}
}

// Read from ring buffer (non-blocking)
ring_buffer_read :: proc(rb: ^RingBuffer, out: []u8) -> int {
	sync.mutex_lock(&rb.mutex)
	defer sync.mutex_unlock(&rb.mutex)

	n := 0
	for n < len(out) && rb.tail != rb.head {
		out[n] = rb.buf[rb.tail]
		rb.tail = (rb.tail + 1) % len(rb.buf)
		n += 1
	}
	return n
}

// Convert u8 to f32 normalized to [-1, 1]
u8_to_f32_centered :: #force_inline proc "contextless" (x: u8) -> f32 {
	return (f32(x) - 127.5) / 127.5
}

// Setup RTL-SDR device
setup_radio :: proc(
	device_id: u32,
	sample_rate: u32,
	center_freq: u32,
) -> (
	^rtlsdr.rtlsdr_dev,
	i32,
) {
	using rtlsdr
	dev: ^rtlsdr_dev
	error := rtlsdr_open(&dev, device_id)
	if error != 0 do return nil, error

	rtlsdr_set_sample_rate(dev, sample_rate)
	rtlsdr_set_center_freq(dev, center_freq)
	rtlsdr_set_tuner_gain_mode(dev, 0) // disable manual gain
	rtlsdr_set_agc_mode(dev, 1) // enable AGC
	rtlsdr_set_direct_sampling(dev, 0) // disable direct sampling
	rtlsdr_set_bias_tee(dev, 1) // disable bias tee
	rtlsdr_set_offset_tuning(dev, 0) // ENABLE offset tuning to avoid DC spike
	rtlsdr_reset_buffer(dev)

	return dev, error
}

// Quickselect algorithm to find k-th smallest element (0-based)
select_k :: proc(arr: []f32, k: int) -> f32 {
	if len(arr) == 0 do return 0
	if k >= len(arr) do return arr[len(arr) - 1]
	if k < 0 do return arr[0]

	// Make a copy since quickselect modifies the array
	temp := make([]f32, len(arr))
	defer delete(temp)
	copy(temp, arr)

	l := 0
	r := len(temp) - 1

	for {
		if l == r do return temp[l]

		pivot := temp[k]
		i := l
		j := r

		for i <= j {
			for temp[i] < pivot do i += 1
			for temp[j] > pivot do j -= 1
			if i <= j {
				temp[i], temp[j] = temp[j], temp[i]
				i += 1
				j -= 1
			}
		}

		if j < k && k < i do return temp[k]
		if k <= j {
			r = j
		} else if k >= i {
			l = i
		}
	}
}

// Clear ring buffer
ring_buffer_clear :: proc(rb: ^RingBuffer) {
	sync.mutex_lock(&rb.mutex)
	defer sync.mutex_unlock(&rb.mutex)
	rb.head = 0
	rb.tail = 0
}

// Retune device helper - now handles thread restart
retune_device :: proc(dev: ^rtlsdr.rtlsdr_dev, cur: ^u32, new_cf: u32) -> i32 {
	using rtlsdr
	if dev == nil do return -1

	fmt.printf("Starting retune from %.3f MHz to %.3f MHz\n", f64(cur^) / 1e6, f64(new_cf) / 1e6)

	// Stop the current async read by canceling it
	fmt.println("Stopping async read...")
	rtlsdr_cancel_async(dev)

	// Wait for thread to finish with timeout
	if app.recv_thread != nil {
		fmt.println("Waiting for receive thread to finish...")
		thread.join(app.recv_thread)
		app.recv_thread = nil
		fmt.println("Receive thread stopped")
	}

	// Small delay to ensure everything is settled
	// time.sleep(20 * time.Millisecond)

	// Now safely retune
	r := rtlsdr_set_center_freq(dev, new_cf)
	if r == 0 {
		rtlsdr_reset_buffer(dev)
		cur^ = rtlsdr_get_center_freq(dev) // read back actual
		fmt.printf("Hardware retuned to %.3f MHz\n", f64(cur^) / 1e6)

		// Clear our buffers after successful retune
		ring_buffer_clear(&app.ring_buffer)
		app.frame_fill = 0
		app.spectrum_initialized = false
		app.frame_counter = 0 // Reset frame counter too

		// Additional delay before restarting
		// time.sleep(20 * time.Millisecond)

		// Restart the async read thread
		fmt.println("Restarting async read...")
		app.recv_thread = thread.create_and_start_with_poly_data2(dev, BLOCK_SIZE, fn = recv_worker)

		// Give the new thread time to start
		// time.sleep(20 * time.Millisecond)

		fmt.printf("Retune completed successfully\n")
	} else {
		fmt.printf("Retune failed (%d) for %.3f MHz\n", r, f64(new_cf) / 1e6)
		// Restart thread even if retune failed to keep receiving
		fmt.println("Restarting async read after failed retune...")
		app.recv_thread = thread.create_and_start_with_poly_data2(dev, BLOCK_SIZE, fn = recv_worker)
	}
	return r
}

// Initialize app resources for given FFT size
init_app_resources :: proc(app: ^App, fft_n: int) {
	app.fft_n = fft_n
	app.sidebar_width = 300 // Default sidebar width

	// Create FFT plan
	app.fft_cfg = kissfft.kiss_fft_alloc(i32(fft_n), 0, nil, nil)

	// Allocate buffers
	app.hann_window = make([]f32, fft_n)
	app.iq_samples = make([]kissfft.kiss_fft_cpx, fft_n)
	app.fft_output = make([]kissfft.kiss_fft_cpx, fft_n)
	app.windowed = make([]kissfft.kiss_fft_cpx, fft_n)
	app.power_spectrum = make([]f32, fft_n)
	app.db_array = make([]f32, fft_n)
	app.db_tmp = make([]f32, fft_n)
	// Full waterfall line for all frequencies
	app.waterfall_line = make([]rl.Color, fft_n)

	// Make Hann window with proper normalization for FFT
	for i in 0 ..< fft_n {
		app.hann_window[i] = 0.5 - 0.5 * math.cos(2.0 * math.PI * f32(i) / f32(fft_n - 1))
	}

	// Create textures - full width for all frequencies
	blank_image := rl.GenImageColor(i32(fft_n), HISTORY_ROWS, rl.BLACK)
	app.ring_texture = rl.LoadTextureFromImage(blank_image)
	rl.UnloadImage(blank_image)
	rl.SetTextureFilter(app.ring_texture, rl.TextureFilter.POINT)

	// Create LUT texture for Viridis colormap
	lut_pixels := make([]rl.Color, 256)
	defer delete(lut_pixels)
	for i in 0 ..< 256 {
		rgb := VIRIDIS_COLORS[i]
		lut_pixels[i] = rl.Color{rgb[0], rgb[1], rgb[2], 255}
	}
	lut_image := rl.Image {
		data    = raw_data(lut_pixels),
		width   = 256,
		height  = 1,
		mipmaps = 1,
		format  = rl.PixelFormat.UNCOMPRESSED_R8G8B8A8,
	}
	app.lut_texture = rl.LoadTextureFromImage(lut_image)

	// Reset state
	app.frame_fill = 0
	app.write_head = 0

	// Reset DC tracking
	app.dc_i_avg = 0.0
	app.dc_q_avg = 0.0
	app.dc_alpha = 0.995 // Slow adaptation for DC tracking
}

// Clean up app resources
cleanup_app_resources :: proc(app: ^App) {
	if app.hann_window != nil do delete(app.hann_window)
	if app.iq_samples != nil do delete(app.iq_samples)
	if app.fft_output != nil do delete(app.fft_output)
	if app.power_spectrum != nil do delete(app.power_spectrum)
	if app.db_array != nil do delete(app.db_array)
	if app.db_tmp != nil do delete(app.db_tmp)
	if app.waterfall_line != nil do delete(app.waterfall_line)
	if app.windowed != nil do delete(app.windowed)

	// Free KissFFT configuration
	// if app.fft_cfg != nil {
	//     kissfft.kiss_fft_free(app.fft_cfg)
	//     app.fft_cfg = nil
	// }

	rl.UnloadTexture(app.ring_texture)
	rl.UnloadTexture(app.lut_texture)
}

// RTL-SDR callback
on_samples :: proc "c" (buf: ^u8, size: u32, ctx: rawptr) {
	context = runtime.default_context()
	app_ptr := cast(^App)ctx

	buffer_slice := slice.from_ptr(buf, int(size))
	ring_buffer_write(&app_ptr.ring_buffer, buffer_slice)
}

// RTL-SDR worker thread
recv_worker :: proc(dev: ^rtlsdr.rtlsdr_dev, chunk_size: int) {
	using rtlsdr
	fmt.println("Starting RTL-SDR async read thread")
	result := rtlsdr_read_async(dev, on_samples, &app, 0, u32(chunk_size))
	fmt.printf("RTL-SDR async read thread exited with code: %d\n", result)
}

main :: proc() {
	using rl

	// Initialize app state
	app.center_freq = CENTER_FREQ_HZ_DEFAULT
	ring_buffer_init(&app.ring_buffer, 4 * 1024 * 1024)
	app.should_stop_recv = false

	// Initialize frequency input
	freq_str := fmt.tprintf("%.3f", f64(app.center_freq) / 1e6)
	copy(app.freq_input[:], transmute([]u8)freq_str)

	// Setup RTL-SDR
	dev, error := setup_radio(0, SAMPLE_RATE, app.center_freq)
	if error != 0 {
		fmt.println("Failed to open RTL-SDR device")
		return
	}
	app.dev = dev
	defer rtlsdr.rtlsdr_close(dev)

	fmt.println("Opened RTL-SDR device")

	// Start receiving thread
	app.recv_thread = thread.create_and_start_with_poly_data2(dev, BLOCK_SIZE, fn = recv_worker)
	defer {
		if app.recv_thread != nil {
			rtlsdr.rtlsdr_cancel_async(dev)
			thread.join(app.recv_thread)
		}
	}

	// Initialize Raylib
	rl.SetConfigFlags(rl.ConfigFlags{.WINDOW_RESIZABLE})
	InitWindow(1400, 1024, "Spectro Analyzer")
	SetTargetFPS(90)
	defer CloseWindow()
	rl.GuiLoadStyle("spectro/resources/style_cyber.rgs")

	// Initialize app resources with default FFT size
	init_app_resources(&app, INIT_FFT_SIZE)
	defer cleanup_app_resources(&app)

	// Default smoothing
	app.smoothing_alpha = 0.0 // default off; enable via UI
	app.spectrum_initialized = false

	// Default overlap setting
	app.overlap_enabled = DEFAULT_OVERLAP_ENABLED
	app.prev_overlap_enabled = DEFAULT_OVERLAP_ENABLED

	// Create shader for scrolling waterfall with LUT
	frag_shader := `#version 330
in vec2 fragTexCoord;
out vec4 finalColor;
uniform sampler2D ring_tex;
uniform sampler2D lut_tex;
uniform float write_head;
uniform float tex_h;
void main(){
    float y = fract(fragTexCoord.y + (write_head/tex_h));
    float v = texture(ring_tex, vec2(fragTexCoord.x, y)).r;
    finalColor = texture(lut_tex, vec2(v, 0.5));
}`


	scroll_shader := LoadShaderFromMemory(nil, cstring(raw_data(frag_shader)))
	defer UnloadShader(scroll_shader)

	write_head_loc := GetShaderLocation(scroll_shader, "write_head")
	tex_height_loc := GetShaderLocation(scroll_shader, "tex_h")
	ring_tex_loc := GetShaderLocation(scroll_shader, "ring_tex")
	lut_tex_loc := GetShaderLocation(scroll_shader, "lut_tex")

	// Set constant shader values
	tex_height := f32(HISTORY_ROWS)
	SetShaderValue(scroll_shader, tex_height_loc, &tex_height, ShaderUniformDataType.FLOAT)

	// Processing state
	read_buf := make([]u8, BLOCK_SIZE)
	defer delete(read_buf)

	// GUI state
	auto_levels := true
	black_level: f32 = -100.0
	white_level: f32 = -30.0
	span_db: f32 = 60.0
	gamma_val: f32 = 1.0
	frame_skip_display: f32 = 0.0

	// Performance timing
	app.stats_time_last = GetTime()

	// FFT dropdown options
	fft_options := "64;128;256;512;1024;2048;4096;8192"

	prev_fft_size_index := app.fft_size_index

	for !WindowShouldClose() {
		// Handle FFT size changes
		// if app.fft_size_index != prev_fft_size_index {
		// 	new_fft_size := FFT_SIZES[app.fft_size_index]
		// 	fmt.printf("Changing FFT size from %d to %d\n", app.fft_n, new_fft_size)

		// 	// Clean up old resources
		// 	if app.fft_cfg != nil {
		// 		kissfft.kiss_fft_free(app.fft_cfg)
		// 	}
		// 	cleanup_app_resources(&app)

		// 	// Initialize with new FFT size
		// 	init_app_resources(&app, new_fft_size)

		// 	// Reset processing state
		// 	app.frame_fill = 0
		// 	app.spectrum_initialized = false

		// 	prev_fft_size_index = app.fft_size_index
		// }

		// Handle overlap mode changes - reset buffer to avoid artifacts
		if app.overlap_enabled != app.prev_overlap_enabled {
			app.frame_fill = 0
			app.prev_overlap_enabled = app.overlap_enabled
		}

		// Handle frequency retuning
		if app.pending_retune {
			// Find the null terminator to get proper string length
			null_pos := 0
			for i in 0 ..< len(app.freq_input) {
				if app.freq_input[i] == 0 {
					null_pos = i
					break
				}
			}
			if null_pos == 0 {
				null_pos = len(app.freq_input)
			}

			freq_str := string(app.freq_input[:null_pos])
			freq_str = strings.trim_space(freq_str)

			if len(freq_str) > 0 {
				if freq_mhz, ok := strconv.parse_f64(freq_str); ok && freq_mhz > 0.01 {
					new_cf := u32(math.round(freq_mhz * 1e6))
					fmt.printf("Retuning to %.3f MHz\n", freq_mhz)
					retune_device(app.dev, &app.center_freq, new_cf)
				} else {
					fmt.println("Invalid frequency input, not retuning")
				}
			}
			app.pending_retune = false
		}

		// Read samples from ring buffer - use blocking read like C version
		bytes_read: int
		if app.freq_edit {
			bytes_read = ring_buffer_read(&app.ring_buffer, read_buf) // non-blocking when editing
		} else {
			// For blocking behavior, we need to loop until we get data
			// Add timeout to prevent infinite blocking after retune
			timeout_start := time.now()
			for bytes_read == 0 {
				bytes_read = ring_buffer_read(&app.ring_buffer, read_buf)
				if bytes_read == 0 {
					time.sleep(time.Millisecond) // brief sleep to avoid busy wait
					// Add timeout check to prevent infinite blocking
					if time.since(timeout_start) > 3 * time.Second {
						fmt.println(
							"Warning: Timeout waiting for samples, checking thread status...",
						)
						// Check if our receive thread is still alive
						if app.recv_thread == nil {
							fmt.println("Receive thread is dead, attempting restart...")
							app.recv_thread = thread.create_and_start_with_poly_data2(
								app.dev,
								BLOCK_SIZE,
								fn = recv_worker,
							)
							timeout_start = time.now() // Reset timeout
						} else {
							break
						}
					}
				}
			}
		}

		// Skip processing if we have no data (e.g., right after retune)
		if bytes_read == 0 {
			// Draw the UI even if we have no data to process
			BeginDrawing()
			ClearBackground(rl.Color{68, 1, 84, 255})
			DrawFPS(10, 10)

			// Show thread status
			thread_status := "UNKNOWN"
			if app.recv_thread == nil {
				thread_status = "DEAD"
			} else {
				thread_status = "ALIVE"
			}

			EndDrawing()
			continue
		}

		// Process IQ samples into FFT frames
		for i := 0; i + 1 < bytes_read; i += 2 {
			// Safety check: ensure frame_fill is within valid bounds
			if app.frame_fill < 0 || app.frame_fill > app.fft_n {
				fmt.printf(
					"ERROR: frame_fill out of bounds: %d (should be 0-%d), resetting\n",
					app.frame_fill,
					app.fft_n,
				)
				app.frame_fill = 0
			}

			// Add new sample first
			if app.frame_fill < app.fft_n {
				I := u8_to_f32_centered(read_buf[i])
				Q := u8_to_f32_centered(read_buf[i + 1])
				app.iq_samples[app.frame_fill] = kissfft.kiss_fft_cpx {
					re = I,
					im = Q,
				}
				app.frame_fill += 1
			}

			if app.frame_fill >= app.fft_n {
				// Decide if we skip processing this frame
				if app.skip_counter < app.frame_skip {
					app.skip_counter += 1
					// Handle frame advancement even when skipping
					if app.overlap_enabled {
						// 50% overlap: shift existing samples by half the frame size
						overlap_size := app.fft_n / 2
						for j in 0 ..< overlap_size {
							app.iq_samples[j] = app.iq_samples[j + overlap_size]
						}
						app.frame_fill = overlap_size
					} else {
						// No overlap: reset frame completely
						app.frame_fill = 0
					}
					continue
				}
				app.skip_counter = 0

				t_fft_start := GetTime()

				// Calculate current frame DC offset
				dc_i: f32 = 0.0
				dc_q: f32 = 0.0
				for i in 0 ..< app.fft_n {
					dc_i += app.iq_samples[i].re
					dc_q += app.iq_samples[i].im
				}
				dc_i /= f32(app.fft_n)
				dc_q /= f32(app.fft_n)

				// Update running DC average with exponential moving average
				// This helps track slow DC drift without removing signal
				if app.frame_counter == 0 {
					// Initialize on first frame
					app.dc_i_avg = dc_i
					app.dc_q_avg = dc_q
				} else {
					// EMA update for slow tracking
					app.dc_i_avg = app.dc_i_avg * app.dc_alpha + dc_i * (1.0 - app.dc_alpha)
					app.dc_q_avg = app.dc_q_avg * app.dc_alpha + dc_q * (1.0 - app.dc_alpha)
				}

				// Apply window and remove tracked DC offset
				for i in 0 ..< app.fft_n {
					s := app.iq_samples[i]
					w := app.hann_window[i]
					// Use the slowly-adapting DC estimate
					app.windowed[i] = kissfft.kiss_fft_cpx {
						re = (s.re - app.dc_i_avg) * w,
						im = (s.im - app.dc_q_avg) * w,
					}
				}

				// Perform FFT on windowed buffer
				kissfft.kiss_fft(
					app.fft_cfg,
					&app.windowed[0],
					// &app.iq_samples[0],
					&app.fft_output[0],
				)

				// Calculate power spectrum with proper normalization
				eps: f32 = 1e-20
				alpha := app.smoothing_alpha

				// Scale factor for FFT - KissFFT doesn't normalize
				fft_scale := 1.0 / f32(app.fft_n)

				for j in 0 ..< app.fft_n {
					re := app.fft_output[j].re * fft_scale
					im := app.fft_output[j].im * fft_scale

					// Calculate power (magnitude squared)
					power := re * re + im * im

					// More aggressive DC bin suppression and neighbor attenuation
					if j == 0 {
						// Strong DC suppression
						power *= 0.01
					} else if j == 1 || j == app.fft_n - 1 {
						// Attenuate immediate neighbors of DC
						power *= 0.5
					}

					// Convert to dB
					raw_db := 10.0 * math.log10_f32(math.max(power, eps))

					// Apply smoothing
					if !app.spectrum_initialized || alpha <= 0.0 {
						app.power_spectrum[j] = raw_db
					} else {
						prev := app.power_spectrum[j]
						// Use smoothed value for display to reduce noise
						app.power_spectrum[j] = prev * (1.0 - alpha) + raw_db * alpha
					}
					app.db_array[j] = app.power_spectrum[j]
				}
				app.spectrum_initialized = true

				t_fft_end := GetTime()
				app.fft_time_acc += (t_fft_end - t_fft_start)
				app.fft_batches += 1

				// Auto-level: use more robust statistics
				if auto_levels {
					// Copy for statistics
					copy(app.db_tmp, app.db_array)

					// Find 25th percentile for noise floor (more stable than 10th)
					p25_idx := int(math.round(0.25 * f32(app.fft_n - 1)))
					if p25_idx < 0 do p25_idx = 0
					if p25_idx >= app.fft_n do p25_idx = app.fft_n - 1

					floor_db := select_k(app.db_tmp, p25_idx)

					// Find 75th percentile for signal level
					copy(app.db_tmp, app.db_array)
					p75_idx := int(math.round(0.75 * f32(app.fft_n - 1)))
					if p75_idx >= app.fft_n do p75_idx = app.fft_n - 1

					signal_db := select_k(app.db_tmp, p75_idx)

					// Set levels with some headroom
					black_level = floor_db - 5.0

					// Use dynamic range between floor and signal
					dynamic_range := signal_db - floor_db
					if dynamic_range < 20.0 {
						// If low dynamic range, use fixed span
						white_level = black_level + span_db
					} else {
						// Otherwise adapt to signal
						white_level = signal_db + 10.0
					}

				}

				// Convert to colors with proper frequency shifting (fftshift)
				lo := black_level
				hi := white_level
				if hi - lo < 0.5 do hi = lo + 0.5 // ensure minimum span

				inv := 1.0 / (hi - lo)
				invg := gamma_val != 0.0 ? 1.0 / gamma_val : 1.0

				for x in 0 ..< app.fft_n {
					// For complex FFT of real-valued input that's been frequency-shifted,
					// the arrangement is:
					// bins 0 to N/2-1: positive frequencies (0 to Nyquist)
					// bins N/2 to N-1: negative frequencies (-Nyquist to 0)
					// We want DC in the center of display

					bin: int
					half_n := app.fft_n / 2

					// Rearrange for display with DC in center:
					if x < half_n {
						// Left half of display: negative frequencies
						// Map from bins N/2 to N-1
						bin = x + half_n
					} else {
						// Right half of display: positive frequencies  
						// Map from bins 0 to N/2-1
						bin = x - half_n
					}

					// Additional check: mirror correction for complex conjugate symmetry
					// Since we're dealing with complex IQ data, not real data,
					// we shouldn't have conjugate symmetry issues, but let's be safe
					if bin >= app.fft_n {
						bin = app.fft_n - 1
					}

					db_val := app.db_array[bin]

					// Normalize to [0, 1]
					v := (db_val - lo) * inv
					v = math.clamp(v, 0.0, 1.0)

					// Apply gamma correction
					if gamma_val != 1.0 {
						v = math.pow(v, invg)
					}

					// Map to grayscale value (0-255) and store in R channel
					gray_val := u8(math.clamp(v * 255.0, 0.0, 255.0))
					app.waterfall_line[x] = Color{gray_val, gray_val, gray_val, 255}
				}

				t_tex_start := GetTime()
				// Update texture - full width
				UpdateTextureRec(
					app.ring_texture,
					Rectangle{0, f32(app.write_head), f32(app.fft_n), 1},
					raw_data(app.waterfall_line),
				)

				app.write_head = (app.write_head + 1) % HISTORY_ROWS
				app.frame_counter += 1

				t_tex_end := GetTime()
				app.tex_time_acc += (t_tex_end - t_tex_start)
				app.tex_batches += 1

				// Handle frame advancement (overlap vs no overlap)
				if app.overlap_enabled {
					// 50% overlap: shift existing samples by half the frame size
					overlap_size := app.fft_n / 2
					// Safety check
					if overlap_size > 0 && overlap_size < app.fft_n {
						// Important: Use memmove semantics for overlapping copy
						// Copy to temporary buffer first to avoid corruption
						temp := make([]kissfft.kiss_fft_cpx, overlap_size)
						defer delete(temp)
						for j in 0 ..< overlap_size {
							temp[j] = app.iq_samples[j + overlap_size]
						}
						for j in 0 ..< overlap_size {
							app.iq_samples[j] = temp[j]
						}
						app.frame_fill = overlap_size
					} else {
						fmt.printf("ERROR: Invalid overlap_size: %d\n", overlap_size)
						app.frame_fill = 0
					}
				} else {
					// No overlap: reset frame completely
					app.frame_fill = 0
				}
			}
		}

		ui_x := f32(rl.GetScreenWidth() - app.sidebar_width + 10)
		ui_y := f32(10)
		slider_x := ui_x + 80

		// Render
		BeginDrawing()
		ClearBackground(rl.Color{68, 1, 84, 255})

		// Draw waterfall FIRST, before GUI elements
		// Also fix the rectangle to not overlap with GUI area
		BeginShaderMode(scroll_shader)

		// Bind textures while shader is active (matches C version approach)
		write_head_float := f32(app.write_head)
		SetShaderValue(
			scroll_shader,
			write_head_loc,
			&write_head_float,
			ShaderUniformDataType.FLOAT,
		)
		SetShaderValueTexture(scroll_shader, ring_tex_loc, app.ring_texture)
		SetShaderValueTexture(scroll_shader, lut_tex_loc, app.lut_texture)

		// Draw waterfall starting at y=300 to leave room for GUI controls
		DrawTexturePro(
			app.ring_texture,
			Rectangle{0, 0, f32(app.ring_texture.width), f32(app.ring_texture.height)},
			Rectangle{0, 0, f32(GetScreenWidth() - app.sidebar_width), f32(GetScreenHeight())},
			Vector2{0, 0},
			0.0,
			WHITE,
		)
		EndShaderMode()

		// Draw a red line in the middle to indicate DC
		dc_line_x := f32((GetScreenWidth() - app.sidebar_width) / 2)
		DrawLine(
			i32(dc_line_x),
			0,
			i32(dc_line_x),
			GetScreenHeight(),
			rl.Color{245, 191, 100, 255},
		)

		// Draw a line where the cursor is and a label with the hovered frequency in MHz
		draw_cursor_freq()

		// Frequency input box
		freq_box := Rectangle{ui_x, ui_y, 120, 24}
		toggled := GuiTextBox(
			freq_box,
			cstring(raw_data(app.freq_input[:])),
			len(app.freq_input),
			app.freq_edit,
		)

		if toggled {
			app.freq_edit = !app.freq_edit
			if !app.freq_edit {
				// When exiting edit mode, just set the pending flag
				app.pending_retune = true
			}
		}

		ui_y = next_row(ui_y)
		tune_btn := Rectangle{ui_x, ui_y, 60, 24}
		if GuiButton(tune_btn, "Tune") {
			fmt.println("Tune button pressed")
			app.pending_retune = true
		}

		if app.freq_edit &&
		   (IsKeyPressed(KeyboardKey.ENTER) || IsKeyPressed(KeyboardKey.KP_ENTER)) {
			app.freq_edit = false
			app.pending_retune = true
		}

		// Auto levels checkbox
		ui_y = next_row(ui_y)
		auto_box := Rectangle{ui_x, ui_y, 20, 20}
		if GuiCheckBox(auto_box, "Auto Levels", &auto_levels) {
			// Checkbox was clicked, auto_levels is already updated by GuiCheckBox
		}

		if !auto_levels {
			// Manual black/white level sliders
			ui_y = next_row(ui_y)
			black_rect := Rectangle{slider_x, ui_y, 200, 20}
			GuiSliderBar(black_rect, "Black dB", nil, &black_level, -140.0, 140.0)

			ui_y = next_row(ui_y)
			white_rect := Rectangle{slider_x, ui_y, 200, 20}
			GuiSliderBar(white_rect, "White dB", nil, &white_level, -140.0, 140.0)

			if white_level <= black_level + 0.5 {
				white_level = black_level + 0.5
			}

			ui_y = next_row(ui_y) // add some space before next controls
			gamma_rect := Rectangle{slider_x, ui_y, 200, 20}
			GuiSliderBar(gamma_rect, "Gamma", nil, &gamma_val, 0.3, 2.5)
		} else {
			// Auto mode: span and gamma controls
			ui_y = next_row(ui_y)
			span_rect := Rectangle{slider_x, ui_y, 200, 20}
			GuiSliderBar(span_rect, "Span dB", nil, &span_db, 10.0, 120.0)

			ui_y = next_row(ui_y)
			gamma_rect := Rectangle{slider_x, ui_y, 200, 20}
			GuiSliderBar(gamma_rect, "Gamma", nil, &gamma_val, 0.3, 2.5)
		}

		// Frame skip slider
		ui_y = next_row(ui_y)
		skip_rect := Rectangle{slider_x, ui_y, 200, 20}
		GuiSliderBar(skip_rect, "Skip", nil, &frame_skip_display, 0.0, 20.0)
		app.frame_skip = int(frame_skip_display + 0.5)

		// Smoothing control (placed below FFT selector)
		ui_y = next_row(ui_y) // add some space
		smooth_rect := Rectangle{slider_x, ui_y, 200, 20}
		prev_alpha := app.smoothing_alpha
		GuiSliderBar(smooth_rect, "Avg (EMA)", nil, &app.smoothing_alpha, 0.0, 0.9)
		if math.abs(prev_alpha - app.smoothing_alpha) > 0.001 {
			// Reset initialization if user increases smoothing significantly
			app.spectrum_initialized = false
		}

		// FFT overlap control
		ui_y = next_row(ui_y) // add some space
		overlap_box := Rectangle{ui_x, ui_y, 20, 20}
		GuiCheckBox(overlap_box, "50% Overlap", &app.overlap_enabled)

		// Draw FPS in bottom right corner

		fps := GetFPS()
		fps_text := fmt.tprintf("FPS: %d", fps)
		DrawText(cstring(raw_data(fps_text)), GetScreenWidth() - 100, GetScreenHeight() - 30, 20, rl.Color{245, 191, 100, 255})

		EndDrawing()

		if IsKeyPressed(KeyboardKey.Q) {
			break // user-requested quit
		}
	}

	// cleanup_app_resources(&app)  // clean up app resources
	fmt.println("Exiting RTL-SDR Spectral Waterfall")

}


next_row :: proc(y: f32, skip: f32 = 30.0) -> f32 {
	return y + skip
}

cols :: proc(
	start_y: f32,
	available_space: f32,
	col_ratios: []f32,
	height: f32 = 30.0,
	margin: f32 = 10.0,
) -> []rl.Rectangle {
	total_ratio: f32 = 0
	for ratio in col_ratios {
		total_ratio += ratio
	}
	cols := make([]rl.Rectangle, len(col_ratios))
	defer delete(cols)

	x := margin
	for i in 0 ..< len(col_ratios) {
		width := (available_space - 2 * margin) * col_ratios[i] / total_ratio
		cols[i] = rl.Rectangle{x, start_y, width, height}
		x += width + margin
	}

	return cols
}

draw_cursor_freq :: proc() {
	using rl
	mouse_x := GetMouseX()
	waterfall_width := GetScreenWidth() - app.sidebar_width
	if mouse_x < waterfall_width {
		// Draw vertical line at mouse X position
		DrawRectangle(i32(mouse_x) - 4, 0, 8, GetScreenHeight(), rl.Color{245, 191, 100, 100})
		DrawLine(i32(mouse_x), 0, i32(mouse_x), GetScreenHeight(), rl.Color{245, 191, 100, 255})
		// Calculate frequency at this X position
		// The waterfall spans from -sample_rate/2 to +sample_rate/2 around center_freq
		// with DC in the center
		normalized_x := f64(mouse_x) / f64(waterfall_width) // 0.0 to 1.0 across waterfall
		freq_offset := (normalized_x - 0.5) * f64(SAMPLE_RATE) // -sample_rate/2 to +sample_rate/2
		freq_hz := f64(app.center_freq) + freq_offset
		freq_mhz := freq_hz / 1e6
		freq_str := fmt.tprintf("%.3f MHz", freq_mhz)

		// Draw frequency label, keeping it within the waterfall area
		label_x := mouse_x + 10
		if label_x + 100 > waterfall_width {
			label_x = mouse_x - 100
		}
		if label_x < 0 do label_x = 10

		DrawText(cstring(raw_data(freq_str)), i32(label_x), GetScreenHeight() - 60, 20, rl.Color{245, 191, 100, 255})

		if IsMouseButtonPressed(rl.MouseButton.LEFT) {
			// Set new center frequency based on mouse click
			new_freq := u32(math.round(freq_hz))
			fmt.printf("Setting new center frequency: %.3f MHz\n", freq_mhz)
			app.center_freq = new_freq
			retune_device(app.dev, &app.center_freq, new_freq)
		}
	}

}
