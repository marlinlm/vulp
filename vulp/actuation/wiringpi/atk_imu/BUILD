load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "atk_imu",
    hdrs = ["atk_ms901m.h", "wiring_uart.h"],
    srcs = ["atk_ms901m.cpp"],
    # copts = ["-IlibwiringPi.so.3.11"],
    # linkopts = ["-L"],
    deps = [
            "@spdlog",
            ]+ select({
                "//:pi64_config": [            
                    "//vulp/actuation:imu_data",
                    "//vulp/actuation/moteus",
                    "//vulp/actuation/wiringpi:wiringPi",
                    "@org_llvm_libcxx//:libcxx"
                ],
                "//conditions:default": [],
    }),
)


add_lint_tests()
