package utils
// FILE SUMMARY: Frequency unit conversions, rounding helper, and u8 normalization
// utilities used by radio setup & display logic.

// NOTE: Utility module untouched by FFT algorithm choices; potential future:
// add fused scaling/window helpers to reduce passes.

import "core:math"

mhz_to_hz :: proc "contextless" (freq: f64) -> u32 {
    return u32(math.round(freq * 1_000_000))
}

hz_to_mhz :: proc "contextless" (freq: u32) -> f64 {
    return f64(freq) / 1_000_000 
}

mhz_to_khz :: proc "contextless" (freq: f64) -> f64 {
    return freq * 1_000.0
}

khz_to_mhz :: proc "contextless" (freq: u32) -> f64 {
    return f64(freq) / 1_000.0
}

// Helper to round to specific decimal places
round_to_decimals :: proc "contextless" (value: f64, decimals: int) -> f64 {
    multiplier := math.pow(10.0, f64(decimals))
    return math.round(value * multiplier) / multiplier
}

u8_to_f32 :: proc (input: ^u8, len: u32) -> []f32 {
    output := make([]f32, len)
    data := ([^]u8)(input)
    for i in 0..<len {
        output[i] = f32(data[i]) / 255.0 // Normalize to [0, 1]
    }
    return output
}