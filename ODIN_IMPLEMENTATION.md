## Odin Spectral Waterfall Implementation

This Odin implementation provides a simplified version of the C spectral waterfall display, focusing on core functionality:

### Key Features Implemented:
1. **RTL-SDR Integration**: Continuous sample acquisition from RTL-SDR device
2. **Ring Buffer**: Thread-safe circular buffer for IQ sample storage between threads
3. **FFT Processing**: Real-time FFT computation using the existing fft module
4. **Power Spectrum**: Magnitude squared calculation and dB conversion
5. **Visual Waterfall**: Scrolling spectrogram display using Raylib
6. **Color Mapping**: Viridis colormap for dB-to-color conversion
7. **GPU Shader**: Efficient scrolling effect using fragment shader

### Configuration:
- **FFT Size**: 2048 samples
- **Sample Rate**: 2.048 MSPS  
- **History**: 1024 waterfall rows
- **Default Frequency**: 101.1 MHz

### Simplified Design:
- No overlapping windows (as requested)
- No audio DSP features
- Fixed FFT size (no dynamic sizing)
- Basic AGC mode
- 50% overlap for smooth display

### Key Differences from C Version:
- Simplified UI (no frequency controls, sliders)
- Fixed parameters (no runtime adjustment)
- Pure visual focus (no complex gain controls)
- Uses existing Odin FFT module instead of Kiss FFT

The implementation provides a clean, focused waterfall display that shows real-time frequency domain visualization of RTL-SDR data.
