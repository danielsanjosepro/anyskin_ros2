"""High pass filter for removing high-frequency drift from a signal."""

import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi


class HighPassFilter:
    def __init__(
        self,
        n_channels: int,
        cutoff_frequency: float,
        sampling_rate: float,
        order: int = 4,
    ):
        """Initialize the high-pass filter.

        Args:
            n_channels (int): Number of channels in the input signal.
            cutoff_frequency (float): The cutoff frequency for the high-pass filter.
            sampling_rate (float): The sampling rate of the input signal.
            order (int): The order of the filter. Default is 4.
        """

        self.n_channels = n_channels
        self.cutoff_frequency = cutoff_frequency
        self.sampling_rate = sampling_rate
        self.order = order

        self.sos = butter(
            self.order,
            self.cutoff_frequency,
            btype="highpass",
            fs=self.sampling_rate,
            output="sos",
        )

        # self.zi = sosfilt_zi(self.sos)
        # Initialize the zero-input state for multiple signals
        self.zi = np.tile(sosfilt_zi(self.sos), (n_channels, 1, 1))

    def apply(self, sample: np.ndarray) -> np.ndarray:
        """
        Apply the high-pass filter to the input signal.

        Args:
            sample (np.ndarray): The input signal to be filtered.
        """
        assert sample.shape[0] == self.n_channels, (
            f"Input sample must have {self.n_channels} channels, "
            f"but got {sample.shape[0]} channels."
        )

        filtered_sample = np.zeros_like(sample)
        for channel in range(self.n_channels):
            filtered_sample_ch, zi_ch = sosfilt(self.sos, sample[channel], zi=self.zi[channel])
            print(
                f"Filtered channel {channel} with {filtered_sample_ch} and zi shape {zi_ch}"
            )

            filtered_sample[channel] = filtered_sample_ch
            self.zi[channel] = zi_ch

        return filtered_sample

    def reset(self):
        """Reset the filter state."""
        self.zi = np.tile(sosfilt_zi(self.sos), (self.n_channels, 1, 1))
