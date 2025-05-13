How to set up and Compile using Bazel

Create symlink to link with kernel_platform for building and running bazel

mkdir -p kernel_platform/external_modules
cd kernel_platform/external_modules
ln -s ../../vendor/qcom/opens/fingerprint


Command to check from kernel_platform dir to check if setup correct
./build_with_bazel.py -t pinapple gki --lto=thin


Command to run bazel build
./tools/bazel build --lto=thin //external_modules/fingerprint:all

Command example to run bazel run to copy to dist dir
./tools/bazel run --lto=thin //external_modules/fingerprint:pinapple_gki_qbt_handler_dist

Note: A few misspells on purpose to avoid banned keywords.
