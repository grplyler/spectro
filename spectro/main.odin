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

import "../vendor/rtlsdr"
import "fft"
import "utils"

// Configuration constants
INIT_FFT_SIZE :: 2048
MAX_FFT_SIZE :: 8192
HISTORY_ROWS :: 1024
SAMPLE_RATE :: 2048000
CENTER_FREQ_HZ_DEFAULT :: 101100000 // 101.1 MHz
// CENTER_FREQ_HZ_DEFAULT :: 136975000 // 101.1 MHz
GAIN_TENTHSDB :: -1 // -1 for auto-gain

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
	fft_plan:             fft.FFTPlan,
	hann_window:          []f32,

	// Processing buffers  
	iq_samples:           []fft.Complex32,
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
}

app: App


// Initialize ring buffer
ring_buffer_init :: proc(rb: ^RingBuffer, size: int) {
	rb.buf = make([]u8, size)
	rb.head = 0
	rb.tail = 0
	rb.mutex = sync.Mutex{}
}

// Write to ring buffer (non-blocking, drops on overflow)
ring_buffer_write :: proc(rb: ^RingBuffer, data: []u8) {
	sync.mutex_lock(&rb.mutex)
	defer sync.mutex_unlock(&rb.mutex)

	for byte in data {
		next := (rb.head + 1) % len(rb.buf)
		if next == rb.tail do break // buffer full, drop data
		rb.buf[rb.head] = byte
		rb.head = next
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
u8_to_f32_centered :: proc(x: u8) -> f32 {
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
	rtlsdr_set_agc_mode(dev, 1) // enable AGC
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

// Retune device helper
retune_device :: proc(dev: ^rtlsdr.rtlsdr_dev, cur: ^u32, new_cf: u32) -> i32 {
	using rtlsdr
	if dev == nil do return -1
	if cur^ == new_cf do return 0

	r := rtlsdr_set_center_freq(dev, new_cf)
	if r == 0 {
		rtlsdr_reset_buffer(dev)
		cur^ = rtlsdr_get_center_freq(dev) // read back actual
		fmt.printf("Retuned to %.3f MHz\n", f64(cur^) / 1e6)
	} else {
		fmt.printf("Retune failed (%d) for %.3f MHz\n", r, f64(new_cf) / 1e6)
	}
	return r
}

// Initialize app resources for given FFT size
init_app_resources :: proc(app: ^App, fft_n: int) {
	app.fft_n = fft_n

	// Create FFT plan
	app.fft_plan = fft.fft_make_plan(fft_n, 0)

	// Allocate buffers
	app.hann_window = make([]f32, fft_n)
	app.iq_samples = make([]fft.Complex32, fft_n)
	app.power_spectrum = make([]f32, fft_n)
	app.db_array = make([]f32, fft_n)
	app.db_tmp = make([]f32, fft_n)
	app.waterfall_line = make([]rl.Color, fft_n)

	// Make Hann window
	for i in 0 ..< fft_n {
		app.hann_window[i] = 0.5 - 0.5 * math.cos(2.0 * math.PI * f32(i) / f32(fft_n - 1))
	}

	// Create textures
	blank_image := rl.GenImageColor(i32(fft_n), HISTORY_ROWS, rl.BLACK)
	app.ring_texture = rl.LoadTextureFromImage(blank_image)
	rl.UnloadImage(blank_image)
	rl.SetTextureFilter(app.ring_texture, rl.TextureFilter.BILINEAR)

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
}

// Clean up app resources
cleanup_app_resources :: proc(app: ^App) {
	if app.hann_window != nil do delete(app.hann_window)
	if app.iq_samples != nil do delete(app.iq_samples)
	if app.power_spectrum != nil do delete(app.power_spectrum)
	if app.db_array != nil do delete(app.db_array)
	if app.db_tmp != nil do delete(app.db_tmp)
	if app.waterfall_line != nil do delete(app.waterfall_line)

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
	rtlsdr_read_async(dev, on_samples, &app, 0, u32(chunk_size))
}

main :: proc() {
	using rl

	// Initialize app state
	app.center_freq = CENTER_FREQ_HZ_DEFAULT
	ring_buffer_init(&app.ring_buffer, 4 * 1024 * 1024)

	// Initialize frequency input
	freq_str := fmt.tprintf("%.3f", f64(app.center_freq) / 1e6)
	copy(app.freq_input[:], transmute([]u8)freq_str)

	// Find initial FFT size index
	app.fft_size_index = 4
	for i, size in FFT_SIZES {
		if size == INIT_FFT_SIZE {
			app.fft_size_index = i
			break
		}
	}

	// Setup RTL-SDR
	dev, error := setup_radio(0, SAMPLE_RATE, app.center_freq)
	if error != 0 {
		fmt.println("Failed to open RTL-SDR device")
		return
	}
	app.dev = dev
	defer rtlsdr.rtlsdr_close(dev)

	// Set gain - skip since commented out in bindings
	// if GAIN_TENTHSDB >= 0 {
	//     rtlsdr.rtlsdr_set_tuner_gain_mode(dev, 1)
	//     rtlsdr.rtlsdr_set_tuner_gain(dev, GAIN_TENTHSDB)
	// } else {
	//     rtlsdr.rtlsdr_set_tuner_gain_mode(dev, 0)  // auto-gain
	// }

	fmt.println("Opened RTL-SDR device")

	// Start receiving thread
	recv_thread := thread.create_and_start_with_poly_data2(dev, 16384, fn = recv_worker)
	defer thread.join(recv_thread)

	// Initialize Raylib
	rl.SetConfigFlags(rl.ConfigFlags{.MSAA_4X_HINT, .WINDOW_RESIZABLE})
	InitWindow(1200, 600, "RTL-SDR Spectral Waterfall")
	SetTargetFPS(60)
	defer CloseWindow()

	// Initialize app resources with default FFT size
	init_app_resources(&app, INIT_FFT_SIZE)
	defer cleanup_app_resources(&app)

	// Default smoothing
	app.smoothing_alpha = 0.0 // default off; enable via UI
	app.spectrum_initialized = false

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
	read_buf := make([]u8, 16384)
	defer delete(read_buf)

	// GUI state
	auto_levels := false
	black_level: f32 = -100.0
	white_level: f32 = -30.0
	span_db: f32 = 60.0
	gamma_val: f32 = 1.0
	frame_skip_display: f32 = 0.0

	// Performance timing
	app.stats_time_last = GetTime()

	// FFT dropdown options
	fft_options := "64;128;256;512;1024;2048;4096;8192"

	for !WindowShouldClose() {
		// Handle FFT size changes
		selected_fft_n := FFT_SIZES[app.fft_size_index]
		if selected_fft_n != app.fft_n {
			cleanup_app_resources(&app)
			init_app_resources(&app, selected_fft_n)
		}

		// Handle frequency retuning
		if app.pending_retune {
			freq_mhz_str := string(app.freq_input[:])
			if freq_mhz, ok := strconv.parse_f64(freq_mhz_str); ok && freq_mhz > 0.01 {
				new_cf := u32(math.round(freq_mhz * 1e6))
				retune_device(app.dev, &app.center_freq, new_cf)
			}
			app.pending_retune = false
		}

		// Read samples from ring buffer - use blocking read like C version
		bytes_read: int
		if app.freq_edit {
			bytes_read = ring_buffer_read(&app.ring_buffer, read_buf) // non-blocking when editing
		} else {
			// For blocking behavior, we need to loop until we get data
			for bytes_read == 0 {
				bytes_read = ring_buffer_read(&app.ring_buffer, read_buf)
				if bytes_read == 0 {
					time.sleep(time.Millisecond) // brief sleep to avoid busy wait
				}
			}
		}

		// Debug output occasionally
		if app.frame_counter % 120 == 0 {
			fmt.printf(
				"Bytes read: %d, frame_fill: %d, write_head: %d, auto_levels: %t\n",
				bytes_read,
				app.frame_fill,
				app.write_head,
				auto_levels,
			)
		}

		// Process IQ samples into FFT frames
		for i := 0; i + 1 < bytes_read; i += 2 {
			if app.frame_fill >= app.fft_n {
				// Decide if we skip processing this frame
				if app.skip_counter < app.frame_skip {
					app.skip_counter += 1
					// Reset frame without FFT processing
					app.frame_fill = 0
					continue
				}
				app.skip_counter = 0

				t_fft_start := GetTime()

				// --- Pre-processing: DC offset removal & window ---
				// Compute DC (mean) to remove tuner DC spike & IQ bias
				sum_re: f32 = 0.0
				sum_im: f32 = 0.0
				for j in 0 ..< app.fft_n {
					sum_re += app.iq_samples[j].re
					sum_im += app.iq_samples[j].im
				}
				inv_n := 1.0 / f32(app.fft_n)
				dc_re := sum_re * inv_n
				dc_im := sum_im * inv_n

				// Apply DC removal + Hann window (coherent gain correction ~2.0 for Hann)
				// We fold coherent gain correction (1/0.5) & 1/N FFT normalization together
				// so overall scale ~ (2 / N).
				norm_scale := 2.0 * inv_n
				for j in 0 ..< app.fft_n {
					w := app.hann_window[j]
					s := fft.Complex32 {
						re = (app.iq_samples[j].re - dc_re),
						im = (app.iq_samples[j].im - dc_im),
					}
					s = fft.complex_scale(s, w * norm_scale)
					app.iq_samples[j] = s
				}

				// Perform FFT in-place
				fft.fft_inplace(&app.fft_plan, app.iq_samples)

				// Calculate power spectrum (already normalized) & dB with simple smoothing
				eps: f32 = 1e-20
				alpha := app.smoothing_alpha // smoothing factor 0..0.9
				for j in 0 ..< app.fft_n {
					re := app.iq_samples[j].re
					im := app.iq_samples[j].im
					power := re * re + im * im // normalized power
					raw_db := 10.0 * math.log10(math.max(power, eps))
					if !app.spectrum_initialized || alpha <= 0.0 {
						app.power_spectrum[j] = raw_db
					} else {
						prev := app.power_spectrum[j]
						app.power_spectrum[j] = prev + alpha * (raw_db - prev)
					}
					// Use raw (unsmoothed) for immediate visual responsiveness
					app.db_array[j] = raw_db
				}
				app.spectrum_initialized = true

				t_fft_end := GetTime()
				app.fft_time_acc += (t_fft_end - t_fft_start)
				app.fft_batches += 1

				// Auto-level: use 10th percentile as noise floor
				if auto_levels {
					// Simple approach: use min + small offset as black level
					min_db := slice.min(app.db_array[:])
					max_db := slice.max(app.db_array[:])

					// Try percentile approach
					p10_idx := int(math.round(0.10 * f32(app.fft_n - 1)))
					if p10_idx < 0 do p10_idx = 0
					if p10_idx >= app.fft_n do p10_idx = app.fft_n - 1

					copy(app.db_tmp, app.db_array)
					floor_candidate := select_k(app.db_tmp, p10_idx)

					// Use floor candidate if reasonable, otherwise use min + offset
					if floor_candidate > min_db && floor_candidate < max_db {
						black_level = floor_candidate
					} else {
						black_level = min_db + 5.0 // 5 dB above minimum
					}
					white_level = black_level + span_db

					// Debug output occasionally
					if app.frame_counter % 120 == 0 {
						fmt.printf(
							"Auto levels: min=%.1f max=%.1f floor=%.1f black=%.1f white=%.1f span=%.1f\n",
							min_db,
							max_db,
							floor_candidate,
							black_level,
							white_level,
							span_db,
						)
					}
				}

				// Convert to colors with frequency shifting (DC in center)
				lo := black_level
				hi := white_level
				if hi - lo < 0.5 do hi = lo + 0.5 // ensure minimum span

				inv := 1.0 / (hi - lo)
				invg := gamma_val != 0.0 ? 1.0 / gamma_val : 1.0

				for x in 0 ..< app.fft_n {
					bin := (x + app.fft_n / 2) % app.fft_n // frequency shift
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
				// Update texture
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

				// Reset frame for next complete frame (no overlap)
				app.frame_fill = 0
			}

			// Add new sample
			if app.frame_fill < app.fft_n {
				I := u8_to_f32_centered(read_buf[i])
				Q := u8_to_f32_centered(read_buf[i + 1])
				app.iq_samples[app.frame_fill] = fft.Complex32 {
					re = I,
					im = Q,
				}
				app.frame_fill += 1
			}
		}

		// Render
		BeginDrawing()
		ClearBackground(BLACK)

		// Draw waterfall with scrolling shader
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

		DrawTexturePro(
			app.ring_texture,
			Rectangle{0, 0, f32(app.ring_texture.width), f32(app.ring_texture.height)},
			Rectangle{0, 0, f32(GetScreenWidth()), f32(GetScreenHeight())},
			Vector2{0, 0},
			0.0,
			WHITE,
		)
		EndShaderMode()

		// Draw info and FPS
		DrawFPS(10, 10)

		freq_actual := rtlsdr.rtlsdr_get_center_freq(app.dev)
		DrawText(
			fmt.ctprintf(
				"%.1f MHz (hw %.1f)  %.2f MS/s  B=%.0f W=%.0f %s G=%.2f N=%d",
				f64(app.center_freq) / 1e6,
				f64(freq_actual) / 1e6,
				f64(SAMPLE_RATE) / 1e6,
				black_level,
				white_level,
				auto_levels ? "AUTO" : "MAN",
				gamma_val,
				app.fft_n,
			),
			10,
			30,
			16,
			GREEN,
		)

		// Frequency input box
		freq_box := Rectangle{10, 54, 120, 24}
		app.freq_edit = GuiTextBox(
			freq_box,
			cstring(raw_data(app.freq_input[:])),
			len(app.freq_input),
			app.freq_edit,
		)

		tune_btn := Rectangle{140, 54, 60, 24}
		if GuiButton(tune_btn, "Tune") {
			app.pending_retune = true
		}
		DrawText("Freq MHz", 205, 58, 16, LIGHTGRAY)

		if app.freq_edit &&
		   (IsKeyPressed(KeyboardKey.ENTER) || IsKeyPressed(KeyboardKey.KP_ENTER)) {
			app.freq_edit = false
			app.pending_retune = true
		}

		// Auto levels checkbox
		auto_box := Rectangle{10, 84, 20, 20}
		if GuiCheckBox(auto_box, "Auto", &auto_levels) {
			// Checkbox was clicked, auto_levels is already updated by GuiCheckBox
		}

		if !auto_levels {
			// Manual black/white level sliders
			black_rect := Rectangle{100, 110, 200, 20}
			GuiSliderBar(black_rect, "Black dB", nil, &black_level, -140.0, 140.0)

			white_rect := Rectangle{100, 136, 200, 20}
			GuiSliderBar(white_rect, "White dB", nil, &white_level, -140.0, 140.0)

			if white_level <= black_level + 0.5 {
				white_level = black_level + 0.5
			}

			gamma_rect := Rectangle{100, 162, 200, 20}
			GuiSliderBar(gamma_rect, "Gamma", nil, &gamma_val, 0.3, 2.5)
		} else {
			// Auto mode: span and gamma controls
			span_rect := Rectangle{100, 110, 200, 20}
			GuiSliderBar(span_rect, "Span dB", nil, &span_db, 10.0, 120.0)

			gamma_rect := Rectangle{100, 136, 200, 20}
			GuiSliderBar(gamma_rect, "Gamma", nil, &gamma_val, 0.3, 2.5)
		}

		// Frame skip slider
		skip_rect := Rectangle{100, auto_levels ? 162 : 188, 200, 20}
		GuiSliderBar(skip_rect, "Skip", nil, &frame_skip_display, 0.0, 20.0)
		app.frame_skip = int(frame_skip_display + 0.5)

		// FFT size dropdown
		fft_rect := Rectangle{100, auto_levels ? 188 : 214, 200, 24}
		fft_size_index_i32 := i32(app.fft_size_index)
		GuiDropdownBox(
			fft_rect,
			cstring(raw_data(fft_options)),
			&fft_size_index_i32,
			app.fft_dropdown_edit,
		)
		app.fft_size_index = int(fft_size_index_i32)

		// Smoothing control (placed below FFT selector)
		smooth_rect := Rectangle{100, auto_levels ? 216 : 242, 200, 20}
		prev_alpha := app.smoothing_alpha
		GuiSliderBar(smooth_rect, "Avg (EMA)", nil, &app.smoothing_alpha, 0.0, 0.9)
		if math.abs(prev_alpha - app.smoothing_alpha) > 0.001 {
			// Reset initialization if user increases smoothing significantly
			app.spectrum_initialized = false
		}

		// Performance stats
		now := GetTime()
		if now - app.stats_time_last >= 1.0 && app.fft_batches > 0 {
			avg_fft_ms := (app.fft_time_acc / f64(app.fft_batches)) * 1000.0
			avg_tex_ms := (app.tex_time_acc / f64(app.tex_batches)) * 1000.0
			fmt.printf(
				"AVG FFT %.3f ms  AVG Upload %.3f ms  batches=%d\n",
				avg_fft_ms,
				avg_tex_ms,
				app.fft_batches,
			)
			app.fft_time_acc = 0.0
			app.tex_time_acc = 0.0
			app.fft_batches = 0
			app.tex_batches = 0
			app.stats_time_last = now
		}

		EndDrawing()

		if IsKeyPressed(KeyboardKey.Q) {
			break // user-requested quit
		}
	}

	CloseWindow() // clean up Raylib window
	// cleanup_app_resources(&app)  // clean up app resources
	fmt.println("Exiting RTL-SDR Spectral Waterfall")
}
