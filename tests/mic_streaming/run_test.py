import pyaudio
import socket
import wave
import os
from datetime import datetime

"""
Listen on udp port 6055 for mic stream
"""
FORMAT = pyaudio.paInt16  # Format of sampling
CHANNELS = 1              # Number of audio channels (1 for mono, 2 for stereo)
RATE = 16000              # Sampling rate (samples per second)
CHUNK = 1024              # Number of audio frames per buffer
PORT = 6055

"""
Play and record wake-word-benchmark files
"""
ROOT_DIR = os.path.join( os.path.dirname(__file__), "../../")
BENCHMARK_DIR = os.path.join(ROOT_DIR, "testdata", "wake-word-benchmark" )
TEST_FILES = [
    "audio/jarvis/0e165e17-134f-4cee-9ec6-b43d1412d7d7.wav",
    "audio/jarvis/1c9fda58-050b-4b8f-8289-dd08c3107c8c.wav",
    "audio/jarvis/6a8a0d5f-514c-4e7e-bc98-961ecef12f1f.wav"
]

"""
Store records under test_runs/CURRENT_DATETIME
"""
TEST_RUN_ROOT = os.path.join(ROOT_DIR, "testdata", "mic_streaming" )
TEST_RUN_DIR = os.path.join(TEST_RUN_ROOT, datetime.now().isoformat() )
os.makedirs( TEST_RUN_DIR )


# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))

# Create an audio object
p = pyaudio.PyAudio()


def play_wav_chunk(file_path):
    chunk = 1024
   
    # Open the WAV file
    with wave.open(file_path, 'rb') as wf:
        stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                    channels=wf.getnchannels(),
                    rate=wf.getframerate(),
                    output=True)
        print( f"WAV: rate:{wf.getframerate()} channels:{wf.getnchannels()}" )
        # Read data in chunks
        data = wf.readframes(chunk)

        # yield audio chunk
        while data:
            stream.write(data)
            yield data
            data = wf.readframes(chunk)
        
        # Close and terminate the stream
        stream.close()


def write_wav_file(file_path, chunks):
    with wave.open( file_path, 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(chunks))


test_files = [
    os.path.join(BENCHMARK_DIR, test_file) for test_file in TEST_FILES
]

for file_idx, test_file in enumerate(test_files):
    chunks = []
    for chunk in play_wav_chunk(test_file) :
        data, addr = sock.recvfrom(CHUNK)
        chunks.append(data)
    for i in range(40):
        data, addr = sock.recvfrom(CHUNK)
        chunks.append(data)
    write_wav_file( os.path.join( TEST_RUN_DIR, f"rec_{file_idx:03d}.wav" ) , chunks)        


sock.close()
p.terminate()