# The current ESP32CAN library includes a deprecated header file. This script patches that.
# in src/CAN.c the header esp_intr.h is replaced with esp_intr_alloc.h

from os.path import join, isfile

Import("env")

LIBRARY_DIR = join(env['PROJECT_LIBDEPS_DIR'], env['PIOENV'], "ESP32CAN")
SRC_PATH = join(LIBRARY_DIR, "src", "CAN.c")

# only patch if we haven't already done it
if not isfile(SRC_PATH + ".bak"):
    assert isfile(SRC_PATH)
    env.Execute("sed -I .bak 's/esp_intr.h/esp_intr_alloc.h/g' '%s'" % (SRC_PATH))
