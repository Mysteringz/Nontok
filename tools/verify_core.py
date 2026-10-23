"""Run the real Mega controller/parser against a host Arduino adapter."""
import shutil
import subprocess
import tempfile
from pathlib import Path

root = Path(__file__).resolve().parents[1]
compiler = shutil.which("g++") or shutil.which("clang++")
if not compiler:
    raise SystemExit("A C++ compiler is required")
with tempfile.TemporaryDirectory(prefix="nontok-mega-") as directory:
    for motors, legacy, sensors, bluetooth in ((0,0,0,0),(1,0,0,0),(1,1,1,0),(1,0,1,2),(1,0,1,3)):
        binary = str(Path(directory) / "tests")
        subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror",
            "-I", str(root / "tests/support"), "-I", str(root / "firmware/core_robot_mega/include"),
            "-DMOTOR_WIRING_CONFIRMED="+str(motors), "-DMOEBIUS_LEGACY_M4_PINS="+str(legacy),
            "-DSENSOR_WIRING_CONFIRMED="+str(sensors), "-DBLUETOOTH_UART="+str(bluetooth),
            str(root / "tests/firmware_core_test.cpp"),
            *[str(root / "firmware/core_robot_mega/src" / name) for name in
              ("control.cpp", "localization.cpp", "calibration.cpp", "storage.cpp", "survey.cpp")], "-o", binary], check=True)
        subprocess.run([binary], check=True)
    subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror",
        "-I", str(root / "firmware/core_robot_mega/include"),
        str(root / "tests/frequency_color_test.cpp"), "-o", binary], check=True)
    subprocess.run([binary], check=True)
    subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror",
        "-I", str(root / "firmware/core_robot_mega/include"),
        *[str(root / p) for p in ("tests/survey_test.cpp", "firmware/core_robot_mega/src/survey.cpp",
                                  "firmware/core_robot_mega/src/localization.cpp")], "-o", binary], check=True)
    subprocess.run([binary], check=True)
print("All five Mega-only test configurations passed")
