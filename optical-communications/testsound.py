import numpy as np
import sounddevice as sd
import time

# Parameters
fs = 44100  # Sampling rate
duration = 0.4  # Duration of each tone (100 ms)
tones = [
    329.63, 329.63, 329.63,   # E E E
    329.63, 329.63, 329.63,   # E E E
    329.63, 392.00, 261.63,   # E G C
    293.66, 329.63, 349.23,   # D E F
    349.23, 349.23, 349.23,   # F F F
    349.23, 329.63, 329.63,   # F E E
    329.63, 329.63, 293.66,   # E E D
    293.66, 329.63, 293.66,   # D D E D
    392.00                     # G
]


try:
    while True:
        for freq in tones:
            t = np.linspace(0, duration, int(fs * duration), endpoint=False)
            waveform = 1 * np.sin(2 * np.pi * freq * t)  # sine wave at freq
            sd.play(waveform, fs)
            sd.wait()  # wait for playback to finish
except KeyboardInterrupt:
    print("Stopped")
