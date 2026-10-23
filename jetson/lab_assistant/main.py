"""Entry point on the Jetson:  cd jetson && python -m lab_assistant.main [--text]"""
import argparse
import logging
import queue
import time

from .config import load_yaml
from .hardware.camera import Camera
from .hardware.serial_link import RobotLink
from .navigation.lab_map import LabMap
from .navigation.localizer import Localizer
from .perception.detector import EquipmentDetector
from .perception.markers import MarkerDetector
from .state_machine import Robot, StateMachine
from .voice.recognizer import TextCommandSource, VoiceListener
from .voice.speaker import Speaker

log = logging.getLogger("lab_assistant")


def main(argv=None):
    parser = argparse.ArgumentParser(description="Self-driving lab assistant")
    parser.add_argument("--config", default="config/robot.yaml")
    parser.add_argument("--map", default="config/lab_map.yaml")
    parser.add_argument("--text", action="store_true", help="type commands instead of speaking them")
    parser.add_argument("--mute", action="store_true", help="log speech instead of playing it")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    cfg = load_yaml(args.config)
    lab_map = LabMap.load(args.map)

    link = RobotLink(**cfg["serial"])
    link.open()
    camera = Camera(**cfg["camera"]).start()
    speaker = Speaker(**cfg["speaker"], enabled=not args.mute)
    log.info("Loading YOLO model %s", cfg["detector"]["weights"])
    detector = EquipmentDetector(**cfg["detector"])

    commands: "queue.Queue" = queue.Queue()
    if args.text:
        source = TextCommandSource(commands)
    else:
        source = VoiceListener(commands, is_muted=lambda: speaker.speaking, **cfg["voice"])
    source.start()

    robot = Robot(
        link=link,
        camera=camera,
        detector=detector,
        markers=MarkerDetector(**cfg["markers"]),
        speaker=speaker,
        localizer=Localizer(cfg["motion"]["speed_scale"]),
        lab_map=lab_map,
        cfg=cfg,
    )
    sm = StateMachine(robot)
    speaker.say("Ready")

    period = 1.0 / cfg["loop_hz"]
    last = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            dt, last = now - last, now
            try:
                cmd = commands.get_nowait()
            except queue.Empty:
                cmd = None
            sm.tick(dt, cmd)
            time.sleep(max(0.0, period - (time.monotonic() - now)))
    except KeyboardInterrupt:
        log.info("Shutting down")
    finally:
        link.stop()
        source.stop()
        camera.stop()
        link.close()


if __name__ == "__main__":
    main()
