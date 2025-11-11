"""Example of a high-pass filter applied to a simulated streaming signal with drift."""

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi

# Parameters
fs = 200.0  # Sampling frequency in Hz
fast_frq = 2.0  # Frequency of the sine wave in Hz
cutoff = 1.0  # High-pass cutoff frequency in Hz
order = 4  # Filter order

# Design filter (SOS form recommended)
sos = butter(order, cutoff, btype="highpass", fs=fs, output="sos")

# Initial filter state
zi = sosfilt_zi(sos)

# Simulated streaming signal: drift + sine wave + noise
t = np.linspace(0, 10, int(10 * fs))
drift = 0.1 * t  # slow drift
signal = drift + np.sin(2 * np.pi * fast_frq * t) + 0.05 * np.random.randn(len(t))

filtered = np.zeros_like(signal)

# Process sample-by-sample (or in small blocks)
for i, x in enumerate(signal):
    filtered[i], zi = sosfilt(sos, [x], zi=zi)

# Plot original vs. filtered
plt.figure(figsize=(10, 4))
plt.plot(t, signal, label="Original (with drift)")
plt.plot(t, filtered, label="High-pass filtered")
plt.xlabel("Time (s)")
plt.legend()
plt.tight_layout()
plt.show()
