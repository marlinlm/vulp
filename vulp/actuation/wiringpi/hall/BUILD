load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hall",
    hdrs = glob(["*.h"]),
    srcs = glob(["*.cpp"]),
    include_prefix = "vulp/actuation",
    deps = [
            "@spdlog",
            ] + select({
                "//:pi64_config": [
                "//vulp/actuation/wiringpi:wiringPi",
                "@org_llvm_libcxx//:libcxx"
                ],
                "//conditions:default": [],
    }),
)

add_lint_tests()
