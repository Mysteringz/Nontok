"""Text-to-speech on the USB speaker (espeak-ng, offline). Utterances play in order, never overlap."""
import logging
import queue
import subprocess
import threading
from typing import Sequence

log = logging.getLogger(__name__)


class Speaker:
    def __init__(self, command: Sequence[str] = ("espeak-ng", "-s", "150"), enabled: bool = True):
        self.command = list(command)
        self.enabled = enabled
        self._queue: "queue.Queue[str]" = queue.Queue()
        self._speaking = threading.Event()
        threading.Thread(target=self._worker, name="speaker", daemon=True).start()

    @property
    def speaking(self) -> bool:
        return self._speaking.is_set() or not self._queue.empty()

    def say(self, text: str):
        log.info("SAY: %s", text)
        if self.enabled:
            self._queue.put(text)

    def _worker(self):
        while True:
            text = self._queue.get()
            self._speaking.set()
            try:
                subprocess.run(self.command + [text], check=False, timeout=15)
            except (OSError, subprocess.TimeoutExpired) as e:
                log.error("Speaker failed: %s", e)
            finally:
                self._speaking.clear()
