"""Run the real Mega controller/parser against a host Arduino adapter."""
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

root = Path(__file__).resolve().parents[1]
compiler = shutil.which("g++") or shutil.which("clang++")
if not compiler:
    raise SystemExit("A C++ compiler is required")
with tempfile.TemporaryDirectory(prefix="nontok-mega-") as directory:
    # Wiring updates must fail closed when an enable flag would share outputs.
    pin_probe='#include "config.h"\nint main() { return 0; }\n'
    for flags,error in (
        ([],None),
        (["-DMOTOR_WIRING_CONFIRMED=1"],None),
        (["-DSENSOR_WIRING_CONFIRMED=1"],None),
        (["-DSENSOR_WIRING_CONFIRMED=1","-DRANGE_LEFT_ECHO_PIN=32"],"Colour OUT cannot share"),
        (["-DSENSOR_WIRING_CONFIRMED=1","-DCOLOR_DRIVER=0"],None),
        (["-DMOTOR_WIRING_CONFIRMED=1","-DMOEBIUS_LEGACY_M4_PINS=1",
          "-DSENSOR_WIRING_CONFIRMED=1"],None),
        (["-DSENSOR_WIRING_CONFIRMED=1",
          "-DCOLOR_SCALE_PINS_EXPOSED=1"],None),
        (["-DSENSOR_WIRING_CONFIRMED=1","-DCOLOR_SCALE_PINS_EXPOSED=1",
          "-DOLED_RESET_ON_D28=1"],"Colour S0 D28 cannot also drive")):
        check=subprocess.run([compiler,"-std=c++11","-fsyntax-only","-x","c++",
            "-I",str(root/"tests/support"),"-I",str(root/"firmware/core_robot_mega/include"),
            *flags,"-"],input=pin_probe,text=True,capture_output=True)
        if error:
            assert check.returncode!=0 and error in check.stderr,check.stderr
        else:
            assert check.returncode==0,check.stderr
    print("Pin allocation: reported conflicts rejected; disabled and explicitly rewired fixtures compile",flush=True)
    for motors, legacy, sensors, bluetooth in ((0,0,0,0),(1,0,0,0),(1,1,1,0),(1,0,1,2),(1,0,1,3)):
        binary = str(Path(directory) / "tests")
        subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror",
            "-I", str(root / "tests/support"), "-I", str(root / "firmware/core_robot_mega/include"),
            "-DMOTOR_WIRING_CONFIRMED="+str(motors), "-DMOEBIUS_LEGACY_M4_PINS="+str(legacy),
            "-DSENSOR_WIRING_CONFIRMED="+str(sensors), "-DBLUETOOTH_UART="+str(bluetooth),
            # Mock sensors bypass acquisition; none of these fixtures authorizes a real motor run.
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
subprocess.run([sys.executable,str(root/"tools/verify_color_sampler.py")],check=True)
print("All five Mega-only test configurations passed")
