import pyaudio
import socket
import wave
import os
from datetime import datetime

"""
Listen on udp port 6055 for audio data and stream directly to output speaker.
"""
FORMAT = pyaudio.paInt16  # Format of sampling
CHANNELS = 1              # Number of audio channels (1 for mono, 2 for stereo)
RATE = 16000              # Sampling rate (samples per second)
CHUNK = 1024              # Number of audio frames per buffer
RECORD_SECONDS = 10       # Duration of recording
PORT = 6055

"""
Store records under test_runs/CURRENT_DATETIME
"""
ROOT_DIR = os.path.join( os.path.dirname(__file__), "../../")
TEST_RUN_ROOT = os.path.join(ROOT_DIR, "testdata", "mic_streaming" )
TEST_RUN_DIR = os.path.join(TEST_RUN_ROOT, datetime.now().isoformat() )
os.makedirs( TEST_RUN_DIR )


# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))

# Create an audio object
p = pyaudio.PyAudio()

# Open stream
stream = p.open(format=pyaudio.paInt16, channels=CHANNELS, rate=RATE, output=True)

file_idx = 0
chunks = []
while True:
    try:
        data, addr = sock.recvfrom(CHUNK)
        chunks.append(data)
        if len(chunks) == int(RATE / CHUNK * RECORD_SECONDS) :
            with wave.open( os.path.join( TEST_RUN_DIR, f"rec_{file_idx:03d}.wav"), 'wb') as wf:
                wf.setnchannels(CHANNELS)
                wf.setsampwidth(p.get_sample_size(FORMAT))
                wf.setframerate(RATE)
                wf.writeframes(b''.join(chunks))
                chunks = []
                file_idx += 1
                
        #print("received message: %s" % data)
        stream.write(data)
    except KeyboardInterrupt:
        break

stream.stop_stream()
stream.close()
p.terminate()

sock.close()