# PlatformIO post-script for the "ota" environment:
# reads OTA_PASS from src/secrets.h (git-ignored) and passes it to espota,
# so the password never has to be typed or stored anywhere else.
Import("env")
import os
import re

path = os.path.join(env.subst("$PROJECT_DIR"), "src", "secrets.h")
try:
    with open(path) as f:
        m = re.search(r'^\s*#define\s+OTA_PASS\s+"([^"]*)"', f.read(), re.M)
    if m:
        env.Append(UPLOADERFLAGS=["--auth=" + m.group(1)])
        print("OTA: using password from secrets.h")
    else:
        print("OTA: no OTA_PASS in secrets.h, uploading without a password")
except OSError:
    print("OTA: secrets.h not found")
