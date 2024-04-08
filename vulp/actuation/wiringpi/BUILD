# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_import(
    name = "wiringPi",
    hdrs= ["wiringPi.h", "wiringSerial.h"],
    shared_library = "libwiringPi.so.3.2",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "wiringpi",
    hdrs = glob(["*.h"]),
    srcs = glob(["*.cpp"]),
    # linkopts = ["-lwiringPi"],
    # linkopts = ["-lbcm2835"],
    deps = [
            "@spdlog",
            "//vulp/actuation/moteus",
            "//vulp/actuation:imu_data",
            "//vulp/actuation:servo_layout",
            ] + select({
                "//:pi64_config": [
                    "//vulp/actuation/wiringpi/atk_imu",
                    "//vulp/actuation/wiringpi/hall",
                    ":wiringPi",
                ],
                "//conditions:default": []
            }),
)


add_lint_tests()
