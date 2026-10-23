"""Offline speech recognition with Vosk on the USB microphone."""
import json
import logging
import queue
import sys
import threading
from typing import Callable

from .commands import grammar, parse_command

log = logging.getLogger(__name__)


class VoiceListener(threading.Thread):
    """Puts a Command on `out_queue` for every recognised command phrase."""

    def __init__(self, out_queue: "queue.Queue", model_dir: str, sample_rate: int = 16000,
                 device=None, is_muted: Callable[[], bool] = lambda: False):
        super().__init__(name="voice", daemon=True)
        self.out_queue = out_queue
        self.model_dir = model_dir
        self.sample_rate = sample_rate
        self.device = device
        self.is_muted = is_muted  # ignore the mic while our own speaker is talking
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        import sounddevice as sd
        from vosk import KaldiRecognizer, Model

        rec = KaldiRecognizer(Model(self.model_dir), self.sample_rate, json.dumps(grammar()))
        audio: "queue.Queue[bytes]" = queue.Queue()

        def on_audio(indata, frames, time_info, status):
            if not self.is_muted():
                audio.put(bytes(indata))

        with sd.RawInputStream(samplerate=self.sample_rate, blocksize=8000, device=self.device,
                               dtype="int16", channels=1, callback=on_audio):
            log.info("Listening for voice commands")
            while not self._stop_event.is_set():
                try:
                    data = audio.get(timeout=0.5)
                except queue.Empty:
                    continue
                if rec.AcceptWaveform(data):
                    self._handle_text(json.loads(rec.Result()).get("text", ""))

    def _handle_text(self, text: str):
        if not text or text == "[unk]":
            return
        cmd = parse_command(text)
        log.info("Heard %r -> %s", text, cmd)
        if cmd:
            self.out_queue.put(cmd)


class TextCommandSource(threading.Thread):
    """Type commands in the terminal instead of speaking (python -m lab_assistant.main --text)."""

    def __init__(self, out_queue: "queue.Queue"):
        super().__init__(name="text-commands", daemon=True)
        self.out_queue = out_queue

    def stop(self):
        pass  # daemon thread, exits with the program

    def run(self):
        for line in sys.stdin:
            cmd = parse_command(line)
            if cmd:
                self.out_queue.put(cmd)
            else:
                print(f"Not a command: {line.strip()!r}")
