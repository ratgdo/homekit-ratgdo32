#!/usr/bin/env python3
#
# This script runs patch command on specified files.
#
# Copyright (c) 2024-25 David Kerr, https://github.com/dkerr64
#
import os

Import("env")
#print(env['PROJECT_PACKAGES_DIR']);
#print(env['PROJECT_LIBDEPS_DIR']);

if os.name == "nt":
    pass
else:
    os.system("patch -N " + env['PROJECT_PACKAGES_DIR'] + "/framework-arduinoespressif32/libraries/WebServer/src/WebServer.cpp url_not_found_log.patch")
