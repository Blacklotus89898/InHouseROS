# cmake/

Shared CMake helpers for the InHouseROS superproject.

Current convention: compiler warning profiles live in the root CMakeLists.txt as
INTERFACE targets (`ihrs_warnings` strict, `ihrs_warnings_legacy` for code being
modernized). Add reusable find-modules, toolchain files, or functions here.
