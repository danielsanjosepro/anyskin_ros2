"""Tests for high_pass_filter.py"""

import numpy as np
import pytest
from anyskin_filtering.high_pass_filter import HighPassFilter


class TestHighPassFilter:
    def test_init(self):
        """Test filter initialization."""
        filter = HighPassFilter(
            n_channels=3, cutoff_frequency=10.0, sampling_rate=1000.0, order=4
        )

        assert filter.n_channels == 3
        assert filter.cutoff_frequency == 10.0
        assert filter.sampling_rate == 1000.0
        assert filter.order == 4
        assert filter.zi.shape == (3, 2, 2)  # (n_channels, n_sections, 2)

    def test_apply_single_sample(self):
        """Test filtering a single sample."""
        filter = HighPassFilter(
            n_channels=2, cutoff_frequency=10.0, sampling_rate=1000.0
        )

        # Create a simple test signal
        sample = np.array([1.0, 2.0])

        filtered_sample = filter.apply(sample)

        assert filtered_sample.shape == sample.shape
        assert isinstance(filtered_sample, np.ndarray)

    def test_apply_wrong_channels(self):
        """Test that wrong number of channels raises assertion error."""
        filter = HighPassFilter(
            n_channels=2, cutoff_frequency=10.0, sampling_rate=1000.0
        )

        # Wrong number of channels
        sample = np.array([1.0, 2.0, 3.0])

        with pytest.raises(AssertionError):
            filter.apply(sample)

    def test_reset(self):
        """Test filter reset functionality."""
        filter = HighPassFilter(
            n_channels=2, cutoff_frequency=10.0, sampling_rate=1000.0
        )

        # Apply some samples to change internal state
        sample = np.array([1.0, 2.0])
        filter.apply(sample)

        # Store current state
        zi_before_reset = filter.zi.copy()

        # Reset filter
        filter.reset()

        # Verify state was reset
        assert not np.array_equal(zi_before_reset, filter.zi)

    def test_dc_removal(self):
        """Test that DC component is removed."""
        filter = HighPassFilter(n_channels=1, cutoff_frequency=1.0, sampling_rate=100.0)

        # Create signal with DC offset
        dc_offset = 5.0
        samples = []

        # Apply multiple samples to settle the filter
        for _ in range(100):
            sample = np.array([dc_offset])
            filtered = filter.apply(sample)
            samples.append(filtered[0])

        # After settling, DC should be mostly removed
        final_samples = samples[-10:]  # Last 10 samples
        mean_output = np.mean(final_samples)

        # Output should be close to zero (DC removed)
        assert abs(mean_output) < 0.1
