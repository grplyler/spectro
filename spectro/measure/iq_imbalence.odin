package measure

import "core:fmt"
import "core:math"

iq_imbalance_metrics :: struct {
	rho:  f32, // |E[z^2]| / E[|z|^2]  (0 = perfect)
	irr:  f32, // ~ 20*log10(2/rho) for small imbalance on proper signals
	gain: f32, // 10*log10(var(I)/var(Q))  (0 dB = ideal)
}

// iq: interleaved float32 buffer [I0,Q0,I1,Q1,...]
// returns: rho, IRR estimate (dB), gain mismatch (dB)
estimate_iq_imbalance :: proc(iq: []f32) -> iq_imbalance_metrics {
	n := len(iq) / 2
	if n == 0 {
		return iq_imbalance_metrics{}
	}

	// 1) Block DC removal (mean subtract)
	mean_i: f64 = 0.0
	mean_q: f64 = 0.0
	for k in 0 ..< n {
		mean_i += f64(iq[2 * k + 0])
		mean_q += f64(iq[2 * k + 1])
	}
	mean_i /= f64(n)
	mean_q /= f64(n)

	// 2) Accumulate stats
	e_abs2: f64 = 0.0 // E{|z|^2}
	e_z2_re: f64 = 0.0 // Re{E[z^2]}
	e_z2_im: f64 = 0.0 // Im{E[z^2]}
	ssi: f64 = 0.0 // sum squares I
	ssq: f64 = 0.0 // sum squares Q

	for k in 0 ..< n {
		I := f64(iq[2 * k + 0]) - mean_i
		Q := f64(iq[2 * k + 1]) - mean_q

		ssi += I * I
		ssq += Q * Q

		e_abs2 += I * I + Q * Q
		e_z2_re += (I * I - Q * Q) // Re{(I + jQ)^2}
		e_z2_im += (2.0 * I * Q) // Im{(I + jQ)^2}
	}

	eps := 1e-30
	p := e_abs2 / f64(n) // E{|z|^2}
	zr := e_z2_re / f64(n) // Re{E[z^2]}
	zi := e_z2_im / f64(n) // Im{E[z^2]}
	rho := math.sqrt(zr * zr + zi * zi) / (p + eps)

	// IRR estimate from rho (good for small imbalance on proper/circular signals)
	irr_db := 20.0 * math.log10(2.0 / (rho + 1e-20))

	varI := ssi / f64(n)
	varQ := ssq / f64(n)
	gain_db := 10.0 * math.log10((varI + eps) / (varQ + eps))

	return iq_imbalance_metrics{rho = f32(rho), irr = f32(irr_db), gain = f32(gain_db)}
}

// Optional: image-canceller coefficient c = E[z^2]/E[|z|^2]
// Apply with: y = z - c * conj(z)
estimate_image_canceller_c :: proc(iq: []f32) -> (c_re: f32, c_im: f32) {
	n := len(iq) / 2
	if n == 0 {
		return 0.0, 0.0
	}

	// DC-block
	mean_i: f64 = 0.0
	mean_q: f64 = 0.0
	for k in 0 ..< n {
		mean_i += f64(iq[2 * k + 0])
		mean_q += f64(iq[2 * k + 1])
	}
	mean_i /= f64(n)
	mean_q /= f64(n)

	e_abs2: f64 = 0.0
	e_z2_re: f64 = 0.0
	e_z2_im: f64 = 0.0

	for k in 0 ..< n {
		I := f64(iq[2 * k + 0]) - mean_i
		Q := f64(iq[2 * k + 1]) - mean_q

		e_abs2 += I * I + Q * Q
		e_z2_re += (I * I - Q * Q)
		e_z2_im += (2.0 * I * Q)
	}

	eps := 1e-30
	p := e_abs2 / f64(n)
	return f32((e_z2_re / f64(n)) / (p + eps)), f32((e_z2_im / f64(n)) / (p + eps))
}

// Example usage
main :: proc() {
	// Fake IQ: mostly proper noise; tweak to simulate imbalance if you like
	iq := []f32{0.2, -0.1, 0.05, 0.07, -0.3, 0.25, 0.1, -0.05}

	m := estimate_iq_imbalance(iq)
	fmt.printf("rho=%.6f, IRR~%.2f dB, gain_mismatch=%.2f dB\n", m.rho, m.irr, m.gain)

	c_re, c_im := estimate_image_canceller_c(iq)
	fmt.printf("image-canceller c = %.6f + j%.6f (apply y = z - c*conj(z))\n", c_re, c_im)
}
