package fft
// FILE SUMMARY: In-place radix-2 DIT FFT with per-stage twiddles, first two stages
// unrolled, plus windowing & spectrum utilities; used for SDR IQ sample processing.
//
// --- Algorithm Notes ---------------------------------------------------------
// Current implementation: iterative in-place radix-2 Cooley–Tukey DIT.
// Optimizations present: per-stage twiddle tables, stage 0/1 unrolling,
// optional #no_bounds_check, complex multiply inlined for later stages.
// Not the theoretical minimum op-count: split-radix FFT (mixing radix-2/4)
// reduces real mults/adds by ~20% vs plain radix-2 for power-of-two sizes.
//
// Further speed avenues (not yet implemented here):
// 1. Split-Radix: implement recursive split of even/odd into even + (odd1, odd2)
//    with tailored butterflies → fewer flops.
// 2. Mixed Radix / Radix-4 / Radix-8: when size divisible by 4 or 8, larger butterflies
//    cut twiddle multiplications & improve vectorization.
// 3. Real-input FFT path: if input purely real, compute N/2 complex FFT + post shuffle.
// 4. SIMD (platform-specific): pack multiple butterflies (SoA layout or interleaved
//    re/im arrays) to exploit vector widths (e.g. AVX2/NEON).
// 5. Fused window + bit-reversal: apply Hann during bit reversal to save a pass.
// 6. Precomputed bit-reversal index table (for reused sizes) to remove loop logic.
// 7. Cache blocking / multi-threading for large N: split outer loops across threads.
// 8. Twiddle recurrence (cos/sin angle addition) to shrink memory footprint if L1
//    pressure higher than recompute cost.
// 9. Loop reordering for better locality: iterate contiguous j segments first
//    (Stockham variants avoid explicit bit reversal and can be more cache friendly).
// 10. Bluestein / Rader: for prime / non power-of-two lengths (not needed yet).
//
// Roadmap stubs (placeholders):
// TODO(split_radix): add split_radix_fft_inplace(plan, buf)
// TODO(radix4): detect if log2_n even and apply radix-4 stages early
// TODO(fuse_window): optional parameter to fft_inplace to apply window in bit_reverse

import "core:math"
import "core:mem"

M_PI :: 3.14159265358979323846

// Twiddles for one stage: w^k for k in [0, half_len)
StageTwiddles :: struct {
	half_len: int,
	w:        []Complex32,
}

FFTPlan :: struct {
	n:           int, // FFT size (power of 2)
	log2_n:      int,
	twiddles:    []StageTwiddles, // len == log2_n
	// Convenience: Hann window you can reuse (len n)
	window:      []f32,
	// Optional: keep sample_rate if you like helpers that know Fs
	sample_rate: f32,
}

Complex32 :: struct {
    re: f32,
    im: f32,
}

complex_add :: #force_inline proc "contextless" (a, b: Complex32) -> Complex32 {
    return Complex32{a.re + b.re, a.im + b.im};
}
complex_sub :: #force_inline proc "contextless" (a, b: Complex32) -> Complex32 {
    return Complex32{a.re - b.re, a.im - b.im};
}
complex_mul :: #force_inline proc "contextless" (a, b: Complex32) -> Complex32 {
    return Complex32{a.re*b.re - a.im*b.im, a.re*b.im + a.im*b.re};
}
complex_scale :: #force_inline proc "contextless" (a: Complex32, s: f32) -> Complex32 {
    return Complex32{a.re*s, a.im*s};
}
complex_conj :: #force_inline proc "contextless" (a: Complex32) -> Complex32 {
    return Complex32{a.re, -a.im};
}
complex_exp :: #force_inline proc "contextless" (theta: f32) -> Complex32 {
    // e^(i*theta) = cos(theta) + i sin(theta)
    return Complex32{math.cos(theta), math.sin(theta)};
}


is_power_of_two :: #force_inline proc "contextless" (n: int) -> bool {
    return n > 0 && ((n & (n-1)) == 0);
}

ilog2 :: #force_inline proc "contextless" (n: int) -> int {
    // assumes n is power of two
    nx: int = n;
    i: int = 0;
    for (nx > 1) {
        nx >>= 1;
        i += 1;
    }
    return i;
}

fft_make_plan :: proc(n: int, sample_rate: f32 = 0.0) -> FFTPlan {
    assert(is_power_of_two(n), "FFT size must be a power of two");
    log2_n := ilog2(n);

    tw := make([]StageTwiddles, log2_n);
    // Build per-stage twiddles for radix-2 iterative decimation-in-time
    // Stage s has size m = 2^(s+1); half_len = m/2
    for s in 0..<log2_n {
        m        := 1 << u32(s+1);
        half_len := m >> 1;
        arr      := make([]Complex32, half_len);

        // Forward FFT uses negative sign in exponent
        // w_m = exp(-2πi / m); then w_k = w_m^k
        // Precompute all k = [0, half_len)
        base := -2.0 * M_PI / f32(m);
        for k in 0..<half_len {
            theta   := base * f32(k);
            arr[k]  = complex_exp(theta);
        }
        tw[s] = StageTwiddles{half_len = half_len, w = arr};
    }

    // Precompute Hann window
    win := make([]f32, n);
    if n > 1 {
        denom := f32(n - 1);
        for i in 0..<n {
            win[i] = 0.5 * (1.0 - math.cos_f32(2.0*M_PI * f32(i) / denom));
        }
    } else if n == 1 {
        win[0] = 1.0;
    }

    return FFTPlan{
        n = n,
        log2_n = log2_n,
        twiddles = tw,
        window = win,
        sample_rate = sample_rate,
    };
}

fft_destroy_plan :: proc(plan: ^FFTPlan) {
    // free nested twiddle arrays
    for &stage in plan.twiddles {
        delete(stage.w)
    }
    delete(plan.twiddles)
    delete(plan.window)
    plan^ = FFTPlan{} // zero it
}

// -------- Bit-reversal permutation --------

bit_reverse_inplace :: #force_inline proc "contextless"(buf: []Complex32) {
    n := len(buf);
    j := 0;
    for i in 0..<(n-1) {
        if i < j {
            tmp := buf[i];
            buf[i] = buf[j];
            buf[j] = tmp;
        }
        m := n >> 1;
        for (m <= j) {
            j -= m;
            m >>= 1;
        }
        j += m;
    }
}

// -------- Core FFT / IFFT --------

fft_inplace_generic :: proc(plan: ^FFTPlan, buf: []Complex32) {
    // assert(len(buf) == plan.n, "buffer length != plan size");
    // 1) bit-reverse
    bit_reverse_inplace(buf);
    n := plan.n;
    for s in 0..<plan.log2_n {
        tw  := plan.twiddles[s];
        m   := tw.half_len << 1;
        for k in 0..<tw.half_len {
            wk := tw.w[k];
            for j := k; j < n; j += m {
                t := complex_mul(wk, buf[j + tw.half_len]);
                u := buf[j];
                buf[j]               = complex_add(u, t);
                buf[j + tw.half_len] = complex_sub(u, t);
            }
        }
    }
}

// New faster version with early-stage unrolling
fft_inplace :: proc(plan: ^FFTPlan, buf: []Complex32) {
    n := plan.n;
    if plan.log2_n < 2 {
        // n == 1 or 2 → generic is fine
        fft_inplace_generic(plan, buf);
        return;
    }

    // 1) bit-reverse
    bit_reverse_inplace(buf);

    // 2) Unrolled stage 0 (butterfly size 2, twiddle always 1)
    // (a, b) -> (a+b, a-b)
    #no_bounds_check for i := 0; i < n; i += 2 {
        a := buf[i];
        b := buf[i+1];
        buf[i]   = Complex32{a.re + b.re, a.im + b.im};
        buf[i+1] = Complex32{a.re - b.re, a.im - b.im};
    }

    if plan.log2_n == 1 {
        return;
    }

    // 3) Unrolled stage 1 (butterfly size 4)
    // k=0 twiddle 1; k=1 twiddle exp(-i*pi/2) = (0,-1) so multiply = (r,i)* (0,-1) = (i, -r)
    #no_bounds_check for i := 0; i < n; i += 4 {
        a0 := buf[i+0];
        a1 := buf[i+1];
        a2 := buf[i+2];
        a3 := buf[i+3];

        // k = 0
        t0 := a2;
        buf[i+0] = Complex32{a0.re + t0.re, a0.im + t0.im};
        buf[i+2] = Complex32{a0.re - t0.re, a0.im - t0.im};

        // k = 1 (twiddle (0,-1))
        t1 := Complex32{a3.im, -a3.re};
        buf[i+1] = Complex32{a1.re + t1.re, a1.im + t1.im};
        buf[i+3] = Complex32{a1.re - t1.re, a1.im - t1.im};
    }

    // 4) Remaining stages (s >= 2)
    #no_bounds_check for s in 2..<plan.log2_n {
        tw := plan.twiddles[s];
        half := tw.half_len;
        m := half << 1;

        // Loop order retained (k outer) for twiddle reuse
        for k in 0..<half {
            wk := tw.w[k];
            for j := k; j < n; j += m {
                b := buf[j + half];
                // Inline complex_mul(wk, b)
                t_re := wk.re*b.re - wk.im*b.im;
                t_im := wk.re*b.im + wk.im*b.re;
                a := buf[j];
                buf[j]       = Complex32{a.re + t_re, a.im + t_im};
                buf[j+half]  = Complex32{a.re - t_re, a.im - t_im};
            }
        }
    }
}

// -------- Windows / Utilities --------

fft_apply_hann :: proc(window: []f32, buf: []Complex32) {
    for i in 0..<len(buf) {
        w := window[i];
        buf[i].re *= w;
        buf[i].im *= w;
    }
}

fft_magnitude :: proc(buf_freq: []Complex32) -> []f32 {
    out := make([]f32, len(buf_freq));
    for i in 0..<len(buf_freq) {
        x := buf_freq[i];
        out[i] = math.sqrt(x.re*x.re + x.im*x.im); // |X[k]|
    }
    return out;
}

// dBFS relative to ref (1.0 == full-scale)
fft_power_dbfs :: proc(buf_freq: []Complex32, full_scale_ref: f32 = 1.0) -> []f32 {
    mags := fft_magnitude(buf_freq)
    eps: f32 = 1e-20; // avoid log(0)
    out  := make([]f32, len(mags))
    for i in 0..<len(mags) {
        // Power proportional to |X|; 20*log10 for amplitude, 10*log10 for power.
        out[i] = 20.0 * math.log10_f32((mags[i] / full_scale_ref) + eps);
    }
    return out;
}

// Return (bin_index, frequency_hz) for max bin in [0..N/2)
fft_peak_bin :: proc(power_db: []f32, sample_rate: f32) -> (int, f32) {
    n := len(power_db);
    half := n/2;
    max_k := 0;
    max_v := power_db[0];
    for k in 1..<half {
        v := power_db[k];
        if v > max_v {
            max_v = v;
            max_k = k;
        }
    }
    hz := (sample_rate * f32(max_k)) / f32(n);
    return max_k, hz;
}

// Shift zero-frequency to center (useful for spectrum display)
fft_shift_inplace :: proc(buf_freq: []Complex32) {
    n := len(buf_freq);
    half := n/2;
    for i in 0..<half {
        j := i + half;
        tmp := buf_freq[i];
        buf_freq[i] = buf_freq[j];
        buf_freq[j] = tmp;
    }
}

// -------- SDR helpers --------


// Convert interleaved CU8 IQ (u8,u8,...) in a byte slice to Complex32 in [-1,1)
cu8_interleaved_to_cf32 :: proc(iq_bytes: []u8) -> []Complex32 {
    assert((len(iq_bytes) % 2) == 0, "IQ bytes must be even (I,Q pairs)");
    n := len(iq_bytes) / 2;
    out := make([]Complex32, n);
    for i in 0..<n {
        I := iq_bytes[2*i+0];
        Q := iq_bytes[2*i+1];
        // Convert u8 [0,255] to f32 [-1,1) by: (x - 127.5) / 127.5
        out[i] = Complex32{
            re = (f32(I) - 127.5) / 127.5, 
            im = (f32(Q) - 127.5) / 127.5
        };
    }
    return out;
}

// Map bin index to frequency (0..N-1) → (-Fs/2 .. +Fs/2] after shift
bin_to_hz_centered :: proc(k: int, n: int, sample_rate: f32) -> f32 {
    // After fft_shift, index 0 is -Fs/2
    return (f32(k) - f32(n)/2.0) * (sample_rate / f32(n));
}