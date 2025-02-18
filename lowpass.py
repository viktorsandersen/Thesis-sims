
import numpy as np

class LowPassFilter:
    def __init__(self, alpha, start_signal=None):
        self.alpha = alpha
        self.old_signal = start_signal

    def filter(self, signal):
        if self.old_signal is None:
            self.old_signal = signal
            return signal
        else:
            filtered_signal = self.alpha * signal + (1 - self.alpha) * self.old_signal
            self.old_signal = filtered_signal
            return filtered_signal