package spectro
import "core:math"

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
