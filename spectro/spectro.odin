package spectro
// Visual spectral waterfall display for RTL-SDR
// Advanced implementation with dynamic FFT sizing, frequency tuning, and performance features

import "base:runtime"
import "core:fmt"
import "core:math"
import "core:mem"
import "core:slice"
import "core:strconv"
import "core:strings"
import "core:sync"
import "core:thread"
import "core:time"


import lay "../../raylay" // Custom RTL-SDR bindings
import fft "../vendor/offt"
import rtlsdr "../vendor/rtlsdr-odin/rtlsdr"
import rl "vendor:raylib"

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
COLOR_HIGHLIGHT :: rl.Color{245, 191, 100, 255}
COLOR_SPECTRUM :: rl.Color{0, 255, 255, 255} // Bright cyan for cyber theme
COLORMAP := VIRIDIS_COLORS

// Available FFT sizes
FFT_SIZES := [8]int{64, 128, 256, 512, 1024, 2048, 4096, 8192}

// Simple ring buffer for IQ samples
RingBuffer :: struct {
	buf:        []u8,
	head, tail: int,
	mutex:      sync.Mutex,
}

FrequencyControlState :: struct {
	thou_left:      u8,
	hund_left:      u8,
	tens_left:      u8,
	ones_left:      u8,
	ones_right:     u8,
	tens_right:     u8,
	hund_right:     u8,
	pending_retune: bool, // Flag to indicate if retune is pending
}

// freq_control_to_hz :: proc(fcs: ^FrequencyControlState) -> u32 {
// 	// Convert frequency control state to Hz
// 	left_part := strings.concatenate({fcs.thou_left, fcs.hund_left, fcs.tens_left, fcs.ones_left})
// 	right_part := strings.concatenate({fcs.tens_right, fcs.hund_right, fcs.thou_right})

// 	left_freq, left_ok := strconv.parse_int(left_part, 10)
// 	right_freq, right_ok := strconv.parse_int(right_part, 10)

// 	if !left_ok || !right_ok || left_freq < 0 || right_freq < 0 {
// 		return 0 // Invalid frequency
// 	}

// 	return u32(left_freq * 1000 + right_freq)
// }

Settings :: struct {
	// FFT Settings
	fft_size_options:           cstring,
	fft_size_index:             i32,
	fft_size:                   i32,

	// Spectrum Settings
	spectrum_height:            f32,
	spectrum_min_db:            f32,
	spectrum_max_db:            f32,
	spectrum_show_grid:         bool,
	spectrum_show_peaks:        bool,
	spectrum_peak_hold_time:    f32,
	spectrum_smooth:            bool,
	spectrum_smooth_frames:     f32,
	spectrum_frame_skip:        f32,

	// Radio Settings
	radio_sample_rate:          [32]u8,
	radio_sample_rate_editing:  bool,
	radio_agc:                  bool,
	radio_gain:                 f32,
	radio_tune_delay_ms:        f32,
	radio_sample_rate_options:  cstring,
	radio_sample_rate_index: i32,
	radio_device_options:       cstring,
	radio_device_index:         i32,

	// Waterfall Settings
	waterfall_min_db:           f32,
	waterfall_max_db:           f32,
	waterfall_gamma:            f32,
	waterfall_50pct_overlap:    bool,
	waterfall_autolevel: bool,

	// UI Settings
	ui_settings_width:          f32,
	ui_settings_tab: SettingsTab,
	ui_text_size_small: i32,
	ui_text_size_med: i32,
	ui_text_size_large: i32
}

SettingsTab :: enum {
	RADIO,
	SPECTRUM, 
	WATERFALL,
	UI
}


default_settings :: proc() -> Settings {
	s := Settings{}
	// Initialize default values
	s.fft_size_options = "256;512;1024;2048;4096;8192;16384"
	s.fft_size_index = 3
	s.fft_size = 2048

	s.spectrum_height = 250
	s.spectrum_min_db = -120
	s.spectrum_max_db = 60
	s.spectrum_show_grid = true
	s.spectrum_show_peaks = true
	s.spectrum_peak_hold_time = 5.0
	s.spectrum_smooth = true
	s.spectrum_smooth_frames = 5
	s.spectrum_frame_skip = 0

	s.radio_agc = true
	s.radio_gain = 0.0
	s.radio_tune_delay_ms = 5.0
	s.radio_sample_rate_index = 1
	s.radio_sample_rate_options = "300000;1024000;2048000"
	s.radio_device_options = "0;1"

	s.waterfall_min_db = -120
	s.waterfall_max_db = 60
	s.waterfall_gamma = 1.0
	s.waterfall_50pct_overlap = true
	s.waterfall_autolevel = true

	s.ui_settings_width = 300
	s.ui_settings_tab = .SPECTRUM
	s.ui_text_size_small = 10
	s.ui_text_size_med = 20
	s.ui_text_size_large = 40
	return s
}

// Application state
App :: struct {
	settings:                      Settings,
	dev:                           ^rtlsdr.rtlsdr_dev,
	ring_buffer:                   RingBuffer,
	center_freq:                   u32,

	// Dynamic FFT state
	fft_n:                         int,
	fft_plan:                      fft.FFTPlan,
	// fft_cfg:              kissfft.kiss_fft_cfg,
	hann_window:                   []f32,

	// Processing buffers  
	iq_samples:                    []fft.Complex32,
	fft_output:                    []fft.Complex32,
	windowed:                      []fft.Complex32,
	power_spectrum:                []f32,
	db_array:                      []f32,
	db_tmp:                        []f32, // for quickselect
	waterfall_line:                []rl.Color,

	// Texture resources
	ring_texture:                  rl.Texture2D,
	lut_texture:                   rl.Texture2D,
	scroll_shader:                 rl.Shader,
	write_head_loc:                i32,
	ring_tex_loc:                  i32,
	lut_tex_loc:                   i32,
	tex_height_loc:                i32,

	// GUI state
	write_head:                    int,
	frame_fill:                    int,
	frame_counter:                 int,

	// Frequency tuning
	freq_input:                    [32]u8,
	freq_edit:                     bool,
	pending_retune:                bool,
	cursor_set_retune:             bool,

	// Frame skipping for waterfall speed
	// frame_skip:                    int,
	skip_counter:                  int,

	// FFT size selection
	fft_size_index:                int,
	fft_dropdown_edit:             bool,

	// Performance timing
	fft_time_acc:                  f64,
	tex_time_acc:                  f64,
	fft_batches:                   int,
	tex_batches:                   int,
	stats_time_last:               f64,

	// Spectrum smoothing
	smoothing_alpha:               f32,
	spectrum_initialized:          bool,

	// Plot averaging
	// plot_averaging_enabled:        bool,
	// averaging_frames:              int,
	frame_history:                 [][]f32, // circular buffer of power spectra
	history_write_index:           int,
	averaged_spectrum:             []f32,

	// Waterfall averaging (separate from plot averaging)
	// waterfall_averaging_enabled:   bool,
	// waterfall_averaging_frames:    int,
	waterfall_frame_history:       [][]f32, // circular buffer for waterfall data
	waterfall_history_write_index: int,
	averaged_waterfall_spectrum:   []f32,

	// FFT overlap control
	overlap_enabled:               bool,
	prev_overlap_enabled:          bool,

	// IQ correction parameters
	dc_i_avg:                      f32, // Running average of DC offset for I
	dc_q_avg:                      f32, // Running average of DC offset for Q
	dc_alpha:                      f32, // EMA factor for DC tracking (0.995 = slow adaptation)

	// Thread management
	recv_thread:                   ^thread.Thread,
	should_stop_recv:              bool,

	// Gui
	sidebar_width:                 i32, // Width of the sidebar for controls
	waterfall_top_offset:          f32, // Height of the waterfall display
	max_db_waterfall:              f32, // dB high limit for spectrum display
	min_db_waterfall:              f32, // dB low limit for spectrum display
	min_db_spectrum:               f32, // Min dB for spectrum plot
	max_db_spectrum:               f32, // Max dB for spectrum plot

	// Peak detection and hold
	// peaks_enabled:                 bool, // Enable peak detection and display
	peak_spectrum:                 []f32, // Peak values for each frequency bin
	peak_timestamps:               []f64, // Timestamp when each peak was last updated
	fcs:                           FrequencyControlState, // Frequency control state for input
}

app: App

fcs_init :: proc() -> FrequencyControlState {
	return FrequencyControlState {
		thou_left = 0,
		hund_left = 1,
		tens_left = 0,
		ones_left = 1,
		ones_right = 1,
		tens_right = 0,
		hund_right = 0,
	}
}

fcs_get_frequency_hz :: proc(fcs: ^FrequencyControlState) -> u32 {
	// Convert MHz to Hz
	// Left side: thousands, hundreds, tens, ones of MHz
	whole_mhz :=
		u32(fcs.thou_left) * 1000 +
		u32(fcs.hund_left) * 100 +
		u32(fcs.tens_left) * 10 +
		u32(fcs.ones_left)
	// Right side: tenths, hundredths, thousandths of MHz  
	frac_mhz := u32(fcs.ones_right) * 100 + u32(fcs.tens_right) * 10 + u32(fcs.hund_right)

	// Convert to Hz: whole MHz * 1M + fractional MHz * 1000
	final_hz := whole_mhz * 1_000_000 + frac_mhz * 1000
	return final_hz
}

fcs_from_hz :: proc(fcs: ^FrequencyControlState, hz: u32) {
	// Convert Hz to MHz with proper rounding
	mhz_scaled := f64(hz) / 1000.0 // Convert to kHz first
	mhz_rounded := math.round(mhz_scaled) // Round to nearest kHz

	// Split into whole MHz and fractional kHz parts
	whole_mhz := u32(mhz_rounded / 1000.0)
	frac_khz := u32(mhz_rounded) % 1000

	// Set frequency control state
	fcs.thou_left = u8(whole_mhz / 1000)
	fcs.hund_left = u8((whole_mhz / 100) % 10)
	fcs.tens_left = u8((whole_mhz / 10) % 10)
	fcs.ones_left = u8(whole_mhz % 10)

	fcs.ones_right = u8(frac_khz / 100)
	fcs.tens_right = u8((frac_khz / 10) % 10)
	fcs.hund_right = u8(frac_khz % 10)
}

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
	// rtlsdr_set_bias_tee(dev, 1) // disable bias tee
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
	delay_ms := 5 * time.Millisecond

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
	time.sleep(delay_ms)

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
		time.sleep(delay_ms)

		// Restart the async read thread
		fmt.println("Restarting async read...")
		app.recv_thread = thread.create_and_start_with_poly_data2(
			dev,
			BLOCK_SIZE,
			fn = recv_worker,
		)

		// Give the new thread time to start
		time.sleep(delay_ms)

		fmt.printf("Retune completed successfully\n")
	} else {
		fmt.printf("Retune failed (%d) for %.3f MHz\n", r, f64(new_cf) / 1e6)
		// Restart thread even if retune failed to keep receiving
		fmt.println("Restarting async read after failed retune...")
		app.recv_thread = thread.create_and_start_with_poly_data2(
			dev,
			BLOCK_SIZE,
			fn = recv_worker,
		)
	}

	// Update frequency control state
	fcs_from_hz(&app.fcs, new_cf)

	app.fcs.pending_retune = false // Reset pending retune flag

	return r
}

// Initialize app resources for given FFT size
init_app_resources :: proc(app: ^App, fft_n: int) {
	app.settings = default_settings()
	app.fft_n = fft_n
	app.sidebar_width = 300 // Default sidebar width
	app.waterfall_top_offset = 400 // Height of the waterfall display

	// Create FFT plan
	// app.fft_cfg = kissfft.kiss_fft_alloc(i32(fft_n), 0, nil, nil)
	app.fft_plan = fft.fft_make_plan(fft_n, SAMPLE_RATE)

	// Frequency control state
	app.fcs = fcs_init()

	// Allocate buffers
	app.hann_window = make([]f32, fft_n)
	app.iq_samples = make([]fft.Complex32, fft_n)
	app.fft_output = make([]fft.Complex32, fft_n)
	app.windowed = make([]fft.Complex32, fft_n)
	app.power_spectrum = make([]f32, fft_n)
	app.db_array = make([]f32, fft_n)
	app.db_tmp = make([]f32, fft_n)
	// Full waterfall line for all frequencies
	app.waterfall_line = make([]rl.Color, fft_n)

	// Initialize plot averaging
	// app.plot_averaging_enabled = false
	// app.averaging_frames = 10 // Default to 10 frames
	app.frame_history = make([][]f32, i32(app.settings.spectrum_smooth_frames))
	for i in 0 ..< i32(app.settings.spectrum_smooth_frames) {
		app.frame_history[i] = make([]f32, fft_n)
	}
	app.history_write_index = 0
	app.averaged_spectrum = make([]f32, fft_n)


	// Initialize peak detection
	app.peak_spectrum = make([]f32, fft_n)
	app.peak_timestamps = make([]f64, fft_n)

	// Initialize peak arrays with very low values
	for i in 0 ..< fft_n {
		app.peak_spectrum[i] = -200.0 // Very low dB value
		app.peak_timestamps[i] = 0.0
	}

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
		rgb := COLORMAP[i]
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

	// Cleanup plot averaging buffers
	if app.frame_history != nil {
		for frame in app.frame_history {
			if frame != nil do delete(frame)
		}
		delete(app.frame_history)
	}
	if app.averaged_spectrum != nil do delete(app.averaged_spectrum)

	// Cleanup waterfall averaging buffers
	if app.waterfall_frame_history != nil {
		for frame in app.waterfall_frame_history {
			if frame != nil do delete(frame)
		}
		delete(app.waterfall_frame_history)
	}
	if app.averaged_waterfall_spectrum != nil do delete(app.averaged_waterfall_spectrum)

	// Cleanup peak detection buffers
	if app.peak_spectrum != nil do delete(app.peak_spectrum)
	if app.peak_timestamps != nil do delete(app.peak_timestamps)

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
	using lay
	using GuiControl
	using GuiDefaultProperty


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
	InitWindow(1200, 720, "Spectro Analyzer")
	SetTargetFPS(60)
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

	app.write_head_loc = GetShaderLocation(scroll_shader, "write_head")
	app.tex_height_loc = GetShaderLocation(scroll_shader, "tex_h")
	app.ring_tex_loc = GetShaderLocation(scroll_shader, "ring_tex")
	app.lut_tex_loc = GetShaderLocation(scroll_shader, "lut_tex")

	// Set constant shader values
	tex_height := f32(HISTORY_ROWS)
	SetShaderValue(scroll_shader, app.tex_height_loc, &tex_height, ShaderUniformDataType.FLOAT)
	app.scroll_shader = scroll_shader

	// Processing state
	read_buf := make([]u8, BLOCK_SIZE)
	defer delete(read_buf)

	// GUI state
	// auto_levels := true
	min_db_waterfall: f32 = -100.0
	max_db_waterfall: f32 = 20.0
	min_db_spectrum: f32 = -100.0
	max_db_spectrum: f32 = 20.0
	span_db: f32 = 60.0
	// gamma_val: f32 = app.settings.waterfall_gamma
	// frame_skip_display: f32 = 0.0

	// Performance timing
	app.stats_time_last = GetTime()

	// FFT dropdown options
	fft_options := "64;128;256;512;1024;2048;4096;8192"

	prev_fft_size_index := app.fft_size_index

	for !WindowShouldClose() {
		// Handle Retune on Enter
		if IsKeyPressed(KeyboardKey.ENTER) && app.fcs.pending_retune && GetMouseY() < 60 {
			retune_device(app.dev, &app.center_freq, fcs_get_frequency_hz(&app.fcs))
		}

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
			ClearBackground(GetColor(u32(GuiGetStyle(DEFAULT, i32(GuiDefaultProperty.BACKGROUND_COLOR)))))
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
				app.iq_samples[app.frame_fill] = fft.Complex32 {
					re = I,
					im = Q,
				}
				app.frame_fill += 1
			}

			if app.frame_fill >= app.fft_n {
				// Decide if we skip processing this frame
				if app.skip_counter < int(app.settings.spectrum_frame_skip) {
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
					app.windowed[i] = fft.Complex32 {
						re = (s.re - app.dc_i_avg) * w,
						im = (s.im - app.dc_q_avg) * w,
					}
				}

				// Perform FFT on windowed buffer
				// fft.kiss_fft(
				// 	app.fft_cfg,
				// 	&app.windowed[0],
				// 	// &app.iq_samples[0],
				// 	&app.fft_output[0],
				// )

				// Apply Hann Window
				// fft.fft_apply_hann(
				// 	app.iq_samples[0],
				// 	&app.hann_window[0],
				// 	app.fft_n,
				// )


				fft.fft_execute(&app.fft_plan, app.windowed, app.fft_output)

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

					// Store raw values in plot averaging history if enabled (always store raw data)
					if app.settings.spectrum_smooth {
						// Store this frame in history for frame-based averaging
						app.frame_history[app.history_write_index][j] = raw_db
					}

					// Always update power_spectrum with either raw or EMA-smoothed values
					// This ensures waterfall has access to current data regardless of plot averaging
					if !app.spectrum_initialized || alpha <= 0.0 {
						app.power_spectrum[j] = raw_db
					} else {
						prev := app.power_spectrum[j]
						// Use smoothed value for display to reduce noise
						app.power_spectrum[j] = prev * (1.0 - alpha) + raw_db * alpha
					}
				}

				// Handle waterfall averaging FIRST (using raw spectrum data)
				waterfall_data: []f32
				// Use current raw spectrum data for waterfall
				waterfall_data = app.power_spectrum

				// Handle plot averaging AFTER waterfall (modifies power_spectrum for plot display only)
				if app.settings.spectrum_smooth {
					// Advance write index in circular buffer
					app.history_write_index = (app.history_write_index + 1) % int(app.settings.spectrum_smooth_frames)

					// Calculate averaged spectrum for plot display
					for j in 0 ..< app.fft_n {
						sum: f32 = 0.0
						for frame_idx in 0 ..< int(app.settings.spectrum_smooth_frames) {
							sum += app.frame_history[frame_idx][j]
						}
						app.averaged_spectrum[j] = sum / app.settings.spectrum_smooth_frames
					}

					// Copy averaged values to power_spectrum for plot display
					copy(app.power_spectrum, app.averaged_spectrum)
				}

				// Copy to db_array for statistics (uses power_spectrum which may be plot-averaged)
				for j in 0 ..< app.fft_n {
					app.db_array[j] = app.power_spectrum[j]
				}

				// Update peak detection if enabled
				if app.settings.spectrum_show_peaks {
					current_time := GetTime()
					for j in 0 ..< app.fft_n {
						current_db := app.power_spectrum[j]

						// Check if current value is higher than stored peak or if peak has expired
						time_since_peak := current_time - app.peak_timestamps[j]
						if current_db > app.peak_spectrum[j] ||
						   time_since_peak > f64(app.settings.spectrum_peak_hold_time) {
							app.peak_spectrum[j] = current_db
							app.peak_timestamps[j] = current_time
						}
					}
				}

				app.spectrum_initialized = true

				t_fft_end := GetTime()
				app.fft_time_acc += (t_fft_end - t_fft_start)
				app.fft_batches += 1

				// Auto-level: use more robust statistics
				if app.settings.waterfall_autolevel {
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
					min_db_waterfall = floor_db - 5.0

					// Use dynamic range between floor and signal
					dynamic_range := signal_db - floor_db
					if dynamic_range < 20.0 {
						// If low dynamic range, use fixed span
						max_db_waterfall = min_db_waterfall + span_db
					} else {
						// Otherwise adapt to signal
						max_db_waterfall = signal_db + 10.0
					}

				} else {
					min_db_waterfall = app.settings.waterfall_min_db
					max_db_waterfall = app.settings.waterfall_max_db
				}

				// Convert to colors with proper frequency shifting (fftshift)
				lo := min_db_waterfall
				hi := max_db_waterfall
				app.max_db_waterfall = hi
				app.min_db_waterfall = lo
				app.min_db_spectrum = min_db_spectrum
				app.max_db_spectrum = max_db_spectrum

				if hi - lo < 0.5 do hi = lo + 0.5 // ensure minimum span

				inv := 1.0 / (hi - lo)
				invg := app.settings.waterfall_gamma != 0.0 ? 1.0 / app.settings.waterfall_gamma : 1.0

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

					db_val := waterfall_data[bin]

					// Normalize to [0, 1]
					v := (db_val - lo) * inv
					v = math.clamp(v, 0.0, 1.0)

					// Apply gamma correction
					if app.settings.waterfall_gamma != 1.0 {
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
						temp := make([]fft.Complex32, overlap_size)
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
		slider_x := ui_x + 110
		slider_width: f32 = f32(app.sidebar_width) - 130

		// Render
		BeginDrawing()
		ClearBackground(GetColor(u32(GuiGetStyle(DEFAULT, i32(GuiDefaultProperty.BACKGROUND_COLOR)))))

		// Draw waterfall FIRST, before GUI elements
		// draw_waterfall(&app)
		// draw_freq_bar()
		// draw_spectrum_plot()
		// draw_cursor_freq()
		// draw_freq_control(&app.fcs)
		// draw_dc_line()

		// Draw Gui Layout
		draw_gui(&app)

		// BEGIN OLD GUI
		// if !auto_levels {
		// 	// Manual black/white level sliders
		// 	slider_min: f32 = -160.0
		// 	slider_max: f32 = 20.0
		// 	ui_y = next_row(ui_y)
		// 	black_rect := Rectangle{slider_x, ui_y, slider_width, 20}
		// 	GuiSliderBar(black_rect, "Min dB Plot", nil, &min_db_spectrum, slider_min, slider_max)

		// 	ui_y = next_row(ui_y)
		// 	white_rect := Rectangle{slider_x, ui_y, slider_width, 20}
		// 	GuiSliderBar(white_rect, "Max dB Plot", nil, &max_db_spectrum, slider_min, slider_max)

		// 	ui_y = next_row(ui_y)
		// 	black_rect = Rectangle{slider_x, ui_y, slider_width, 20}
		// 	GuiSliderBar(
		// 		black_rect,
		// 		"Min dB Waterfall",
		// 		nil,
		// 		&min_db_waterfall,
		// 		slider_min,
		// 		slider_max,
		// 	)

		// 	ui_y = next_row(ui_y)
		// 	white_rect = Rectangle{slider_x, ui_y, slider_width, 20}
		// 	GuiSliderBar(
		// 		white_rect,
		// 		"Max dB Waterfall",
		// 		nil,
		// 		&max_db_waterfall,
		// 		slider_min,
		// 		slider_max,
		// 	)


		// 	if max_db_waterfall <= min_db_waterfall + 0.5 {
		// 		max_db_waterfall = min_db_waterfall + 0.5
		// 	}

		// 	ui_y = next_row(ui_y) // add some space before next controls
		// 	gamma_rect := Rectangle{slider_x, ui_y, slider_width, 20}
		// 	GuiSliderBar(gamma_rect, "Gamma", nil, &gamma_val, 0.3, 2.5)
		// } else {
		// 	// Auto mode: span and gamma controls
		// 	ui_y = next_row(ui_y)
		// 	span_rect := Rectangle{slider_x, ui_y, slider_width, 20}
		// 	GuiSliderBar(span_rect, "Span dB", nil, &span_db, 10.0, 120.0)

		// 	ui_y = next_row(ui_y)
		// 	gamma_rect := Rectangle{slider_x, ui_y, slider_width, 20}
		// 	GuiSliderBar(gamma_rect, "Gamma", nil, &gamma_val, 0.3, 2.5)
		// }

		// // Frame skip slider
		// ui_y = next_row(ui_y)
		// skip_rect := Rectangle{slider_x, ui_y, slider_width, 20}
		// GuiSliderBar(skip_rect, "Skip", nil, &frame_skip_display, 0.0, 20.0)
		// app.frame_skip = int(frame_skip_display + 0.5)

		// // Smoothing control (placed below FFT selector)
		// ui_y = next_row(ui_y) // add some space
		// smooth_rect := Rectangle{slider_x, ui_y, slider_width, 20}
		// prev_alpha := app.smoothing_alpha
		// GuiSliderBar(smooth_rect, "Avg (EMA)", nil, &app.smoothing_alpha, 0.0, 0.9)
		// if math.abs(prev_alpha - app.smoothing_alpha) > 0.001 {
		// 	// Reset initialization if user increases smoothing significantly
		// 	app.spectrum_initialized = false
		// }

		// // Plot averaging controls
		// ui_y = next_row(ui_y) // add some space
		// averaging_box := Rectangle{ui_x, ui_y, 20, 20}
		// prev_averaging_enabled := app.plot_averaging_enabled
		// GuiCheckBox(averaging_box, "Plot Averaging", &app.plot_averaging_enabled)

		// // Show averaging frames slider only when averaging is enabled
		// if app.plot_averaging_enabled {
		// 	ui_y = next_row(ui_y)
		// 	frames_rect := Rectangle{slider_x, ui_y, slider_width, 20}
		// 	prev_frames := app.averaging_frames
		// 	frames_float := f32(app.averaging_frames)
		// 	GuiSliderBar(frames_rect, "Plot Frames", nil, &frames_float, 2.0, 50.0)
		// 	app.averaging_frames = int(frames_float + 0.5)

		// 	// Reallocate frame history if number of frames changed
		// 	if app.averaging_frames != prev_frames {
		// 		// Clean up old buffers
		// 		if app.frame_history != nil {
		// 			for frame in app.frame_history {
		// 				if frame != nil do delete(frame)
		// 			}
		// 			delete(app.frame_history)
		// 		}

		// 		// Allocate new buffers
		// 		app.frame_history = make([][]f32, app.averaging_frames)
		// 		for i in 0 ..< app.averaging_frames {
		// 			app.frame_history[i] = make([]f32, app.fft_n)
		// 		}
		// 		app.history_write_index = 0
		// 	}
		// }

		// // If averaging was just disabled, make sure EMA smoothing can work
		// if prev_averaging_enabled && !app.plot_averaging_enabled {
		// 	app.spectrum_initialized = false
		// }

		// // Waterfall averaging controls (separate from plot averaging)
		// ui_y = next_row(ui_y) // add some space
		// waterfall_averaging_box := Rectangle{ui_x, ui_y, 20, 20}
		// prev_waterfall_averaging_enabled := app.waterfall_averaging_enabled
		// GuiCheckBox(
		// 	waterfall_averaging_box,
		// 	"Waterfall Averaging",
		// 	&app.waterfall_averaging_enabled,
		// )

		// // Show waterfall averaging frames slider only when averaging is enabled
		// if app.waterfall_averaging_enabled {
		// 	ui_y = next_row(ui_y)
		// 	waterfall_frames_rect := Rectangle{slider_x, ui_y, slider_width, 20}
		// 	prev_waterfall_frames := app.waterfall_averaging_frames
		// 	waterfall_frames_float := f32(app.waterfall_averaging_frames)
		// 	GuiSliderBar(
		// 		waterfall_frames_rect,
		// 		"Waterfall Frames",
		// 		nil,
		// 		&waterfall_frames_float,
		// 		2.0,
		// 		20.0,
		// 	)
		// 	app.waterfall_averaging_frames = int(waterfall_frames_float + 0.5)

		// 	// Reallocate waterfall frame history if number of frames changed
		// 	if app.waterfall_averaging_frames != prev_waterfall_frames {
		// 		// Clean up old buffers
		// 		if app.waterfall_frame_history != nil {
		// 			for frame in app.waterfall_frame_history {
		// 				if frame != nil do delete(frame)
		// 			}
		// 			delete(app.waterfall_frame_history)
		// 		}

		// 		// Allocate new buffers
		// 		app.waterfall_frame_history = make([][]f32, app.waterfall_averaging_frames)
		// 		for i in 0 ..< app.waterfall_averaging_frames {
		// 			app.waterfall_frame_history[i] = make([]f32, app.fft_n)
		// 		}
		// 		app.waterfall_history_write_index = 0
		// 	}
		// }

		// // Peak detection controls
		// ui_y = next_row(ui_y) // add some space
		// peaks_box := Rectangle{ui_x, ui_y, 20, 20}
		// prev_peaks_enabled := app.peaks_enabled
		// GuiCheckBox(peaks_box, "Show Peaks", &app.peaks_enabled)

		// // Show peak hold time slider only when peaks are enabled
		// if app.peaks_enabled {
		// 	ui_y = next_row(ui_y)
		// 	peak_hold_rect := Rectangle{slider_x, ui_y, slider_width, 20}
		// 	GuiSliderBar(peak_hold_rect, "Peak Hold (s)", nil, &app.peak_hold_time, 0.5, 10.0)
		// }

		// // If peaks were just enabled, reset all peak data
		// if !prev_peaks_enabled && app.peaks_enabled {
		// 	current_time := GetTime()
		// 	for i in 0 ..< app.fft_n {
		// 		app.peak_spectrum[i] = -200.0 // Very low dB value
		// 		app.peak_timestamps[i] = current_time
		// 	}
		// }

		// // FFT overlap control
		// ui_y = next_row(ui_y) // add some space
		// overlap_box := Rectangle{ui_x, ui_y, 20, 20}
		// GuiCheckBox(overlap_box, "50% Overlap", &app.overlap_enabled)
		// END OLD GUI

		// Draw FPS in bottom right corner
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

		fps := GetFPS()
		fps_text := fmt.tprintf("FPS: %d", fps)
		DrawText(
			cstring(raw_data(fps_text)),
			GetScreenWidth() - 100,
			GetScreenHeight() - 40,
			20,
			rl.Color{245, 191, 100, 255},
		)

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

draw_dc_line :: proc() {
	using rl
	// Draw a red line in the middle to indicate DC
	dc_line_x := f32((GetScreenWidth() - app.sidebar_width) / 2)
	DrawLine(i32(dc_line_x), 60, i32(dc_line_x), GetScreenHeight(), rl.Color{245, 191, 100, 255})
}

draw_gui :: proc(app: ^App) {
	using lay
	using rl
	
	// TODO
	// Draw waterfall FIRST, before GUI elements // done
	// draw_waterfall(&app) // done
	// draw_freq_bar() // done
	// draw_spectrum_plot() // done
	// draw_cursor_freq()
	// draw_freq_control(&app.fcs)
	// draw_dc_line()

	s := &app.settings

	// Configure global defaults
	RLSetDefaultGap(10)
	RLSetDefaultPadAll(0)
	
	// Main layout: Fixed left sidebar, flexible center, fixed right sidebar
	panel := rl.Rectangle{0, 0, f32(rl.GetScreenWidth()), f32(rl.GetScreenHeight())}
	RLBeginRow(panel, plan = {-1, s.ui_settings_width}, pad = PadAll(10))
		RLBeginColumn(RLNext(-1, -1), plan = {app.settings.spectrum_height, -1})
			// Draw Spectrum 
			spectrum_rect := RLNext(-1, -1)
			draw_spectrum_plot(spectrum_rect)

			// Draw freq bar
			draw_freq_bar(spectrum_rect)
			// draw_dc_line()

			// Draw Waterfall
			draw_waterfall(app, RLNext(-1, -1))
		RLEnd()
		RLBeginColumn(RLNext(-1, -1))
			RLBeginRow(RLNext(30, -1), plan = {-1, -1, -1})
				if GuiButton(RLNext(-1, -1), "Radio") do s.ui_settings_tab = .RADIO
				if GuiButton(RLNext(-1, -1), "Spectrum") do s.ui_settings_tab = .SPECTRUM
				if GuiButton(RLNext(-1, -1), "Waterfall") do s.ui_settings_tab = .WATERFALL
				// if GuiButton(RLNext(-1, -1), "UI") do settings_tab = .UI
			RLEnd()

			// Spectrum Settings
				if s.ui_settings_tab == .RADIO {
				settings_panel, panel_pad := RLNextPanel(-1, -1)
				GuiPanel(settings_panel, "Radio Settings")
				RLBeginColumn(settings_panel, pad = panel_pad)
					GuiComboBoxHelper("Device ID", s.radio_device_options, &s.radio_device_index)
					GuiComboBoxHelper("Sample Rate", s.radio_sample_rate_options, &s.radio_sample_rate_index)
					GuiCheckBox(RLNext(20, 20), "Automatic Gain Control", &s.radio_agc)
					if !s.radio_agc {
						// Additional controls for peak visualization
						GuiSliderBarHelper("Gain", &s.radio_gain, 1, 40)
					}
					GuiSliderBarHelper("Tune Delay (ms)", &s.radio_tune_delay_ms, 0, 100)
					
				RLEnd()
			} else if s.ui_settings_tab == .SPECTRUM {
				panel_rect, panel_pad := RLNextPanel(-1, -1)
					GuiPanel(panel_rect, "Spectrum Settings")
					RLBeginColumn(panel_rect, pad = panel_pad)
						GuiComboBoxHelper("FFT Size", s.fft_size_options, &s.fft_size_index)
						GuiSliderBarHelper("Height", &s.spectrum_height, 10, 800)
						GuiSliderBarHelper("Min dB", &s.spectrum_min_db, -120, 60)
						GuiSliderBarHelper("Max dB", &s.spectrum_max_db, -120, 60)
						GuiSliderBarHelper("Frame Skip", &s.spectrum_frame_skip, 0, 30)
						GuiCheckBox(RLNext(20, 20), "Show Grid", &s.spectrum_show_grid)
						GuiCheckBox(RLNext(20, 20), "Show Peaks", &s.spectrum_show_peaks)
						if s.spectrum_show_peaks {
							// Additional controls for peak visualization
							GuiSliderBarHelper("Peak Hold Time (sec)", &s.spectrum_peak_hold_time, 0, 60)
						}
						GuiCheckBox(RLNext(20, 20), "Smooth", &s.spectrum_smooth)
						if s.spectrum_smooth {
							GuiSliderBarHelper("Smooth Frames", &s.spectrum_smooth_frames, 1, 60)
						}
					RLEnd()


			} else if s.ui_settings_tab == .WATERFALL {
				panel_rect, panel_pad := RLNextPanel(-1, -1)
				GuiPanel(panel_rect, "Waterfall Settings")
				RLBeginColumn(panel_rect, pad = panel_pad)
					GuiCheckBox(RLNext(20, 20), "Autolevel", &s.waterfall_autolevel)
					if !s.waterfall_autolevel {
						GuiSliderBarHelper("Min dB", &s.waterfall_min_db, -120, 60)
						GuiSliderBarHelper("Max dB", &s.waterfall_max_db, -120, 60)
					}
					GuiSliderBarHelper("Gamma", &s.waterfall_gamma, 0.1, 3.0)
					GuiCheckBox(RLNext(20, 20), "50% Overlap", &s.waterfall_50pct_overlap)
				RLEnd()
			} else if s.ui_settings_tab == .UI {
				panel_rect, panel_pad := RLNextPanel(-1, -1)
				GuiPanel(panel_rect, "UI Settings")
				RLBeginColumn(panel_rect, pad = panel_pad)
					GuiSliderBarHelper("Settings Panel Width", &s.ui_settings_width, 50, 500)
				RLEnd()
			}
			
			// End Spectrum Settings
		RLEnd()
	RLEnd() // End main row layout
}

draw_cursor_freq :: proc() {
	using rl
	mouse_x := GetMouseX()
	waterfall_width := GetScreenWidth() - app.sidebar_width
	if mouse_x < waterfall_width {
		// Draw vertical line at mouse X position
		DrawRectangle(i32(mouse_x) - 10, 60, 20, GetScreenHeight(), rl.Color{245, 191, 100, 100})
		DrawLine(i32(mouse_x), 60, i32(mouse_x), GetScreenHeight(), rl.Color{245, 191, 100, 255})
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

		DrawText(
			cstring(raw_data(freq_str)),
			i32(label_x),
			GetScreenHeight() - 60,
			20,
			rl.Color{245, 191, 100, 255},
		)

		if IsMouseButtonPressed(rl.MouseButton.LEFT) && GetMouseY() > 60 {
			// Set new center frequency based on mouse click
			new_freq := u32(math.round(freq_hz))
			fmt.printf("Setting new center frequency: %.3f MHz\n", freq_mhz)
			app.center_freq = new_freq
			retune_device(app.dev, &app.center_freq, new_freq)
		}
	}

}

// Configuration constants for frequency bar display
FREQ_BAR_MAJOR_TICK_SPACING_MHZ :: 1.0 // Major tick spacing in MHz
FREQ_BAR_MINOR_TICK_SPACING_MHZ :: 0.1 // Minor tick spacing in MHz
FREQ_BAR_SHOW_MINOR_LABELS :: true // Whether to show text labels on minor ticks

draw_freq_bar :: proc(rect: rl.Rectangle) {
	using rl
	using GuiControl
	using GuiDefaultProperty

	// Calculate frequency range and scale
	waterfall_width := i32(rect.width)
	// DrawRectangleRec(rect, Fade(ORANGE, 0.1))

	// Check if we need to redraw (frequency or window size changed)


	half_sample_rate := f64(SAMPLE_RATE) / 2.0
	center_freq_mhz := f64(app.center_freq) / 1e6
	start_freq_mhz := center_freq_mhz - half_sample_rate / 1e6
	end_freq_mhz := center_freq_mhz + half_sample_rate / 1e6

	// Frequency bar parameters
	bar_height: i32 = 50
	bar_y: i32 = 35
	text_height: i32 = 20
	minor_text_height: i32 = 10
	tick_height: i32 = 8

	// Draw background bar
	// DrawRectangle(i32(rect.x), bar_y, waterfall_width, bar_height, GetColor(u32(GuiGetStyle(DEFAULT, i32(GuiDefaultProperty.BACKGROUND_COLOR)))))

	// Use configured major tick spacing
	nice_step := f64(FREQ_BAR_MAJOR_TICK_SPACING_MHZ)

	// Pre-calculate constants for performance
	freq_span_mhz := end_freq_mhz - start_freq_mhz
	inv_freq_span := 1.0 / freq_span_mhz
	first_tick := math.floor(start_freq_mhz / nice_step) * nice_step

	// Draw major ticks and labels
	freq := first_tick
	tick_count := 0
	max_ticks := 20 // Safety limit to prevent excessive drawing
	for freq <= end_freq_mhz + nice_step && tick_count < max_ticks {
		if freq >= start_freq_mhz - nice_step / 2.0 {
			// Calculate x position (optimized)
			normalized_pos := (freq - start_freq_mhz) * inv_freq_span
			x := i32(normalized_pos * f64(waterfall_width))

			if x >= 0 && x <= waterfall_width {
				// Draw major tick
				DrawLine(x, bar_y, x, bar_y + tick_height, rl.Color{200, 200, 200, 255})

				// Use a pre-formatted string approach to avoid allocations every frame
				// Round frequency for cleaner display
				freq_rounded := utils.round_to_decimals(freq, 1)

				// Simple integer check for cleaner labels
				if math.abs(freq_rounded - math.round(freq_rounded)) < 0.01 {
					// Display as integer
					freq_str := fmt.tprintf("%.0f", freq_rounded)
					// defer delete(freq_str) // Clean up allocation

					text_width := MeasureText(cstring(raw_data(freq_str)), text_height)
					label_x := x - text_width / 2
					if label_x < 0 do label_x = 0
					if label_x + text_width > waterfall_width do label_x = waterfall_width - text_width

					DrawText(
						cstring(raw_data(freq_str)),
						label_x,
						bar_y + tick_height + 2,
						text_height,
						rl.Color{200, 200, 200, 255},
					)

					// Draw Mhz Label to the bottom right of the number
					// DrawText(
					// 	"MHz",
					// 	label_x + text_width + 10,
					// 	bar_y + bar_height - 20,
					// 	20,
					// 	rl.Color{180, 180, 180, 255},
					// )
				} else {
					// Display with one decimal place
					freq_str := fmt.tprintf("%.3f", freq_rounded)
					// defer delete(freq_str) // Clean up allocation

					text_width := MeasureText(cstring(raw_data(freq_str)), text_height)
					label_x := x - text_width / 2
					if label_x < 0 do label_x = 0
					if label_x + text_width > waterfall_width do label_x = waterfall_width - text_width

					DrawText(
						cstring(raw_data(freq_str)),
						label_x,
						bar_y + tick_height + 2,
						text_height,
						rl.Color{200, 200, 200, 255},
					)

					// Draw Mhz Label to the bottom right of the number
					// DrawText(
					// 	"MHz",
					// 	label_x + text_width + 10,
					// 	bar_y + bar_height - 20,
					// 	20,
					// 	rl.Color{180, 180, 180, 255},
					// )
				}
				tick_count += 1
			}
		}
		freq += nice_step
	}

	// Use configured minor tick spacing
	minor_step := f64(FREQ_BAR_MINOR_TICK_SPACING_MHZ)
	freq = math.floor(start_freq_mhz / minor_step) * minor_step
	minor_count := 0
	max_minor := 50 // Limit minor ticks for performance
	for freq <= end_freq_mhz + minor_step && minor_count < max_minor {
		if freq >= start_freq_mhz - minor_step / 2.0 {
			normalized_pos := (freq - start_freq_mhz) * inv_freq_span
			x := i32(normalized_pos * f64(waterfall_width))

			if x >= 0 && x <= waterfall_width {
				// Fixed check: only draw if not close to a major tick
				remainder := math.mod(freq, nice_step)
				is_major := remainder < 0.01 || remainder > (nice_step - 0.01)
				if !is_major {
					DrawLine(x, bar_y, x, bar_y + tick_height / 2, rl.Color{150, 150, 150, 128})

					// Draw minor tick labels if enabled
					if FREQ_BAR_SHOW_MINOR_LABELS {
						freq_rounded := utils.round_to_decimals(freq, 1)
						freq_str := fmt.tprintf("%.1f", freq_rounded)

						text_width := MeasureText(cstring(raw_data(freq_str)), minor_text_height)
						label_x := x - text_width / 2
						if label_x < 0 do label_x = 0
						if label_x + text_width > waterfall_width do label_x = waterfall_width - text_width

						DrawText(
							cstring(raw_data(freq_str)),
							label_x,
							bar_y + tick_height / 2 + 2,
							minor_text_height,
							rl.Color{150, 150, 150, 180},
						)
					}

					minor_count += 1
				}
			}
		}
		freq += minor_step
	}

	// Draw center frequency marker
	// center_x := waterfall_width / 2
	// DrawLine(center_x, bar_y - 5, center_x, bar_y + bar_height, rl.Color{255, 255, 0, 255})

}

draw_spectrum_plot :: proc(bounds: rl.Rectangle) {
	using rl
	using lay
	

	if !app.spectrum_initialized || len(app.power_spectrum) == 0 {
		return
	}

	GuiPanel(bounds, "Spectrum")

	waterfall_width := i32(bounds.width) + 10
	plot_height: i32 = i32(bounds.height)
	plot_y: i32 = i32(bounds.y)
	plot_x: i32 = 10
	// Draw background
	// DrawRectangle(0, plot_y, waterfall_width, plot_height, rl.Color{20, 20, 30, 180})

	// Draw y label background
	// Draw background bar

	// Calculate dB range for scaling
	min_db: f32 = app.settings.spectrum_min_db
	max_db: f32 = app.settings.spectrum_max_db

	db_range := max_db - min_db
	if db_range < 1.0 do db_range = 1.0

	// Cyber theme highlight color (bright cyan)
	spectrum_color := COLOR_SPECTRUM // Cyan
	peak_color := COLOR_HIGHLIGHT // Yellow for peaks
	grid_color := rl.Color{100, 100, 120, 100}


	// Draw spectrum line with fftshift
	if len(app.db_array) >= 2 {
		half_n := app.fft_n / 2

		for x in plot_x ..< waterfall_width - 1 {
			// Map x position to frequency bin (with fftshift)
			bin_f := f32(x) * f32(app.fft_n) / f32(waterfall_width)
			bin := int(bin_f)

			// Apply fftshift mapping (same as waterfall)
			actual_bin: int
			if bin < half_n {
				actual_bin = bin + half_n
			} else {
				actual_bin = bin - half_n
			}

			// Clamp to valid range
			if actual_bin >= app.fft_n do actual_bin = app.fft_n - 1
			if actual_bin < 0 do actual_bin = 0

			// Get next bin for line drawing
			next_bin_f := f32(x + 1) * f32(app.fft_n) / f32(waterfall_width)
			next_bin := int(next_bin_f)
			next_actual_bin: int
			if next_bin < half_n {
				next_actual_bin = next_bin + half_n
			} else {
				next_actual_bin = next_bin - half_n
			}
			if next_actual_bin >= app.fft_n do next_actual_bin = app.fft_n - 1
			if next_actual_bin < 0 do next_actual_bin = 0

			// Scale dB values to plot coordinates
			db1 := app.db_array[actual_bin]
			db2 := app.db_array[next_actual_bin]

			y1_norm := (max_db - db1) / db_range
			y2_norm := (max_db - db2) / db_range

			y1 := plot_y + i32(y1_norm * f32(plot_height))
			y2 := plot_y + i32(y2_norm * f32(plot_height))

			// Clamp to plot area
			y1 = math.clamp(y1, plot_y, plot_y + plot_height)
			y2 = math.clamp(y2, plot_y, plot_y + plot_height)

			DrawLine(i32(x), y1, i32(x + 1), y2, spectrum_color)
		}
	}

	// Draw peak spectrum if enabled
	if app.settings.spectrum_show_peaks && len(app.peak_spectrum) >= 2 {
		half_n := app.fft_n / 2

		for x in plot_x ..< waterfall_width - 1 {
			// Map x position to frequency bin (with fftshift)
			bin_f := f32(x) * f32(app.fft_n) / f32(waterfall_width)
			bin := int(bin_f)

			// Apply fftshift mapping (same as spectrum)
			actual_bin: int
			if bin < half_n {
				actual_bin = bin + half_n
			} else {
				actual_bin = bin - half_n
			}

			// Clamp to valid range
			if actual_bin >= app.fft_n do actual_bin = app.fft_n - 1
			if actual_bin < 0 do actual_bin = 0

			// Get next bin for line drawing
			next_bin_f := f32(x + 1) * f32(app.fft_n) / f32(waterfall_width)
			next_bin := int(next_bin_f)
			next_actual_bin: int
			if next_bin < half_n {
				next_actual_bin = next_bin + half_n
			} else {
				next_actual_bin = next_bin - half_n
			}
			if next_actual_bin >= app.fft_n do next_actual_bin = app.fft_n - 1
			if next_actual_bin < 0 do next_actual_bin = 0

			// Scale peak dB values to plot coordinates
			peak_db1 := app.peak_spectrum[actual_bin]
			peak_db2 := app.peak_spectrum[next_actual_bin]

			y1_norm := (max_db - peak_db1) / db_range
			y2_norm := (max_db - peak_db2) / db_range

			y1 := plot_y + i32(y1_norm * f32(plot_height))
			y2 := plot_y + i32(y2_norm * f32(plot_height))

			// Clamp to plot area
			y1 = math.clamp(y1, plot_y, plot_y + plot_height)
			y2 = math.clamp(y2, plot_y, plot_y + plot_height)

			DrawLine(i32(x), y1, i32(x + 1), y2, peak_color)
		}
	}

	// DrawRectangle(0, plot_y, 80, plot_height, rl.Color{40, 40, 40, 200})
	// panel, panel_pad := RLNextPanel(-1, 1)
	

	// Draw grid lines
	if app.settings.spectrum_show_grid {
		grid_lines := 5
		for i in 0 ..= grid_lines {
			y := plot_y + i32(f32(i) * f32(plot_height) / f32(grid_lines))
			DrawLine(plot_x, y, waterfall_width, y, grid_color)

			// Grid labels - skip first and last
			if i > 0 && i < grid_lines {
				db_val := max_db - (f32(i) / f32(grid_lines)) * db_range
				label := fmt.tprintf("%.0fdB", db_val)
				DrawText(cstring(raw_data(label)), plot_x + 10, y - 4, app.settings.ui_text_size_small, rl.Color{200, 200, 200, 255})
			}
		}
	}



	// Draw plot border
	// DrawRectangleLines(0, plot_y, waterfall_width, plot_height, rl.Color{100, 100, 120, 255})

	// Label
	// DrawText("Spectrum", waterfall_width - 100, plot_y + 5, 20, rl.Color{200, 200, 200, 255})

	// DebugButton(-1, -1)
}

draw_digit_input :: proc(pending_retune: ^bool, digit: ^u8, y: i32, x: i32) {
	using rl

	// Draw a rectangle for the digit input
	digit_width: i32 = 30
	digit_height: i32 = 42
	margin: i32 = 10
	backing_color := rl.Color{255, 255, 255, 20}
	backing_color_hover := rl.Color{255, 255, 255, 60}
	text_color := COLOR_HIGHLIGHT

	changed := false

	rect := Rectangle{f32(x), f32(y), f32(digit_width), f32(digit_height)}
	if CheckCollisionPointRec(GetMousePosition(), rect) {
		backing_color = backing_color_hover
		// Handle Mouse Scroll to change digit
		move := GetMouseWheelMove()
		if move != 0.0 {
			// Adjust digit with scroll
			if move > 0.0 {
				// Scroll up: increment digit
				if digit^ < 9 {
					digit^ += 1
				} else {
					digit^ = 0 // wrap around
				}
			} else if move < 0.0 {
				// Scroll down: decrement digit
				if digit^ > 0 {
					digit^ -= 1
				} else {
					digit^ = 9 // wrap around
				}
			}

			pending_retune^ = true // Mark as changed
		}

		// Handle Keyboard input (check if keys 0-9 are pressed)
		GetKeyPressed := GetKeyPressed()
		if GetKeyPressed >= KeyboardKey.ZERO && GetKeyPressed <= KeyboardKey.NINE {
			// Convert key to digit
			key_digit := u8(GetKeyPressed - KeyboardKey.ZERO)
			if key_digit != digit^ {
				digit^ = key_digit // Set the digit to the pressed key
				pending_retune^ = true // Mark as changed	

			}
		}

	}

	DrawRectangle(x, y, digit_width, digit_height, backing_color)
	DrawText(fmt.ctprintf("%d", digit^), x + 5, y + 3, 40, text_color)

}
draw_freq_control :: proc(fcs: ^FrequencyControlState) {
	using rl

	// Position for the frequency input controls
	ui_x := i32((GetScreenWidth() - app.sidebar_width) / 2 - 168)
	ui_y := i32(10) // Below the main frequency input

	// Box dimensions
	digit_width: i32 = 30
	digit_height: i32 = 42
	margin: i32 = 10
	backing_color := rl.Color{255, 255, 255, 20}
	backing_color_hover := rl.Color{255, 255, 255, 40}
	text_color := COLOR_HIGHLIGHT

	pending_retune := &fcs.pending_retune

	// Draw Rect back the digit
	draw_digit_input(pending_retune, &fcs.thou_left, ui_y, ui_x)

	ui_x += digit_width + margin
	draw_digit_input(pending_retune, &fcs.hund_left, ui_y, ui_x)

	ui_x += digit_width + margin
	draw_digit_input(pending_retune, &fcs.tens_left, ui_y, ui_x)

	ui_x += digit_width + margin
	draw_digit_input(pending_retune, &fcs.ones_left, ui_y, ui_x)

	// DOT
	ui_x += digit_width + margin
	DrawText(".", ui_x + 5, ui_y + 3, 40, COLOR_HIGHLIGHT)

	ui_x += digit_width + margin
	draw_digit_input(pending_retune, &fcs.ones_right, ui_y, ui_x)

	ui_x += digit_width + margin
	draw_digit_input(pending_retune, &fcs.tens_right, ui_y, ui_x)

	ui_x += digit_width + margin
	draw_digit_input(pending_retune, &fcs.hund_right, ui_y, ui_x)

	ui_x += digit_width + margin
	DrawText("MHz", ui_x, ui_y + 4, 40, backing_color_hover)

	if pending_retune^ {
		if GuiButton(
			Rectangle{f32(ui_x + digit_width + margin + 50), f32(ui_y + 10), 60, 24},
			"Tune",
		) {
			new_freq := fcs_get_frequency_hz(fcs)
			retune_device(app.dev, &app.center_freq, new_freq)
			fcs.pending_retune = false // Reset pending flag after processing
		}

	}

}
