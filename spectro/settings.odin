package spectro

Settings :: struct {
	// FFT Settings
	fft_size_options:          cstring,
	fft_size_index:            i32,
	fft_size:                  i32,

	// Spectrum Settings
	spectrum_height:           f32,
	spectrum_min_db:           f32,
	spectrum_max_db:           f32,
	spectrum_show_grid:        bool,
	spectrum_show_peaks:       bool,
	spectrum_peak_hold_time:   f32,
	spectrum_smooth:           bool,
	spectrum_smooth_frames:    f32,
	spectrum_frame_skip:       f32,
	spectrum_show_panel:       bool,

	// Radio Settings
	radio_sample_rate:         [32]u8,
	radio_sample_rate_editing: bool,
	radio_agc:                 bool,
	radio_gain:                f32,
	radio_tune_delay_ms:       f32,
	radio_sample_rate_options: cstring,
	radio_sample_rate_index:   i32,
	radio_device_options:      cstring,
	radio_device_index:        i32,

	// Waterfall Settings
	waterfall_min_db:          f32,
	waterfall_max_db:          f32,
	waterfall_gamma:           f32,
	waterfall_50pct_overlap:   bool,
	waterfall_autolevel:       bool,
	waterfall_show_panel:      bool,

	// UI Settings
	ui_settings_width:         f32,
	ui_settings_tab:           SettingsTab,
	ui_text_size_small:        i32,
	ui_text_size_med:          i32,
	ui_text_size_large:        i32,

	// General
	show_tuning_cursor:        bool,
	click_to_tune:             bool,
}

SettingsTab :: enum {
	RADIO,
	SPECTRUM,
	WATERFALL,
	UI,
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
	s.spectrum_show_panel = true

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
	s.waterfall_show_panel = true

	s.ui_settings_width = 300
	s.ui_settings_tab = .SPECTRUM
	s.ui_text_size_small = 10
	s.ui_text_size_med = 20
	s.ui_text_size_large = 40

	s.show_tuning_cursor = true
	s.click_to_tune = true
	return s
}
