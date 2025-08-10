package spectro

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