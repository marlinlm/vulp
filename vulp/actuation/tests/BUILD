# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "test_common",
    hdrs = [
        "coffee_machine_layout.h",
    ],
)

cc_test(
    name = "interface_test",
    srcs = [
        "InterfaceTest.cpp",
    ],
    deps = [
        "//vulp/actuation:interface",
        ":test_common",
        "@googletest//:main",
    ],
)

cc_test(
    name = "mock_interface_test",
    srcs = [
        "MockInterfaceTest.cpp",
    ],
    deps = [
        "//vulp/actuation:mock_interface",
        ":test_common",
        "@googletest//:main",
    ],
)

cc_test(
    name = "bullet_utils_test",
    srcs = [
        "bullet_utils_test.cpp",
    ],
    data = [
        "@upkie_description",
    ],
    deps = [
        "//vulp/actuation:bullet_utils",
        "@bazel_tools//tools/cpp/runfiles",
        "@googletest//:main",
    ],
)

cc_test(
    name = "bullet_interface_test",
    srcs = [
        "BulletInterfaceTest.cpp",
    ],
    data = [
        "@upkie_description",
    ],
    deps = [
        "//vulp/actuation:bullet_interface",
        ":test_common",
        "@bazel_tools//tools/cpp/runfiles",
        "@googletest//:main",
    ],
)

add_lint_tests()