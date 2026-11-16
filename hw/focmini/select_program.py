"""Choose one Arduino application source without adding a PlatformIO env."""

import os

Import("env")

program = os.environ.get("FOCMINI_PROGRAM", "main").strip().lower()
source_filters = {
    "main": ["-<*>", "+<main.cpp>"],
    "motor_test": ["-<*>", "+<motor_test.cpp>"],
    "encoder_test": ["-<*>", "+<encoder_test.cpp>"],
}

if program not in source_filters:
    print("FOCMINI_PROGRAM must be 'main', 'motor_test', or 'encoder_test'")
    env.Exit(2)

env.Replace(SRC_FILTER=source_filters[program])
print("FOCMINI_PROGRAM=%s; selected source: %s.cpp" % (program, program))
