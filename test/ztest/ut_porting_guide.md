# SOF Unit-Test Porting Guide

## Purpose

This document defines the requirements for porting SOF unit tests from CMocka to Zephyr
ztest. It records decisions and reviewer feedback from completed ports so that subsequent
work remains equivalent, portable, and reviewable.

The migration plan and per-test status are maintained in [ztest_plan.md](ztest_plan.md).

## Scope and Test Architecture

The initial target for all ports is Zephyr's `native_sim` platform:

```sh
west twister --testsuite-root sof/test/ztest/unit/ --platform native_sim \
	--verbose --inline-logs
```

Basic and advanced unit tests belong below `test/ztest/unit/`. Tests that exercise
component interactions, pipelines, or platform-specific behavior belong below
`test/ztest/integration/` and may be enabled on additional platforms.

## Porting Requirements

### Preserve test intent

- Port the existing CMocka test cases, vectors, edge cases, expected values, and tolerances
  before extending test coverage.
- Preserve SOF fixed-point semantics. Do not replace SOF reference vectors with generic
  `libm` calculations unless the reference behavior is proven equivalent.
- Document intentional changes to legacy tests, including known defects found in source test
  data. Do not silently "correct" them during a mechanical port.
- Each setup operation must establish a condition required by the test. Do not add redundant
  initialization or generated scaffolding without understanding the API being tested.

### Test data and bounds

- Derive array sizes from the data using `ARRAY_SIZE()` and `sizeof()`; never duplicate
  vector lengths as numeric literals.
- For coupled input, expected-output, and reference arrays, add a compile-time size check:

  ```c
  BUILD_ASSERT(ARRAY_SIZE(input) == ARRAY_SIZE(reference));
  ```

- Iterate using the size of the array indexed in the loop, or use a single shared size
  constant for both generation and iteration.
- Use the exact fixed-point and integer types required by the API. Do not rely on implicit
  narrowing conversions in test inputs or expected results.
- Declare immutable test vectors `const` where the tested API permits it.

### Assertions and helpers

- Check the return value of fallible helpers such as `memcpy_s()` and assert success before
  inspecting test results.
- Include actionable assertion messages for data-driven tests: identify the input/vector
  index, expected value, actual value, and tolerance where applicable.
- Use named constants for repeated fixed-point scales, tolerances, and conversion factors.
- Extract truly shared conversion or rounding logic into a local test helper instead of
  copying it between test cases.

### Source and build integration

- Add the test source and only its required portable SOF sources to the test
  `CMakeLists.txt`.
- Gate architecture-specific implementations, such as Xtensa HiFi sources, so they are not
  compiled for `native_sim`.
- Add required host libraries explicitly when the test itself uses them. Verify the selected
  native toolchain links them successfully.
- Do not modify production firmware sources solely to accommodate a ztest build. If a source
  change is necessary, justify it independently and validate both firmware and `native_sim`
  configurations.
- Keep includes minimal. Remove unused headers, especially headers that define large static
  reference tables.

### Memory allocation and sanitizers

- Do not add per-suite `malloc()`-backed copies of `rmalloc()`/`rzalloc()`/`rfree()`/
  `sof_heap_alloc()`. Link the real `zephyr/lib/alloc.c` and add
  `CONFIG_SYS_HEAP_BIG_ONLY=y` to `prj.conf` (its native_sim heap is 256 KiB). Suites that
  need the module allocator but not `module/generic.c` link
  `test/ztest/unit/common/mod_alloc.c`, which forwards `mod_alloc_ext()`/
  `mod_balloc_align()`/`mod_free()` to the SOF heap. `common/alloc.c` remains for suites
  that replace the allocator through `-Wl,--wrap`.
- `zephyr/lib/alloc.c` provides two native_sim back ends: the Zephyr `sys_heap`
  (default, same allocator as firmware) and host `malloc()`/`free()` through
  `nsi_host_malloc()` (`CONFIG_SOF_NATIVE_SIM_HOST_HEAP=y`). ASan and LSan cannot see blocks
  carved out of the static `sys_heap` arena, so add a second `<scenario>.host_heap`
  scenario with `extra_configs: [CONFIG_SOF_NATIVE_SIM_HOST_HEAP=y]` (see
  `math/fft/testcase.yaml`).
- Every allocation made by a test case must be freed by that test case. Check with:

  ```sh
  DEBUGINFOD_URLS= west twister --platform native_sim -T <suite dir> \
  	--enable-asan --enable-lsan --inline-logs
  ```

  Clear `DEBUGINFOD_URLS` (or set `ASAN_OPTIONS=symbolize=0`): otherwise
  `llvm-symbolizer` queries debuginfod while LSan prints the leak report, which takes
  longer than twister's post-verdict grace period. Twister then kills the binary,
  ignores its exit code and reports the leaking suite as passed.

### ztest metadata and style

- Add or update `CMakeLists.txt`, `prj.conf`, and `testcase.yaml` with the test source.
- Use the established `sof.unit.*` testcase naming convention and tags that describe the
  functions covered.
- Ensure comments, testcase descriptions, test names, and tags accurately describe the
  mathematical operation and fixed-point format.
- Prefer symbolic standard limits such as `INT32_MAX` over opaque numeric literals.
- Apply SOF C formatting, including preprocessor directives at column zero.

## Lessons from Completed Ports

| Pull request | Lessons to retain |
|---|---|
| [#10066](https://github.com/thesofproject/sof/pull/10066) | Test setup must reflect API requirements; unnecessary initialization obscures intent and suggests the test was not understood. |
| [#10136](https://github.com/thesofproject/sof/pull/10136) | Inspect suspicious legacy test data, such as skipped values, but distinguish real defects from false-positive automated findings by checking the actual C layout and semantics. |
| [#10138](https://github.com/thesofproject/sof/pull/10138) | Use `ARRAY_SIZE()`, const-correct vectors, exact-width types, and explicit shared suite/test infrastructure rather than implicit cross-source dependencies. |
| [#10233](https://github.com/thesofproject/sof/pull/10233) | Keep metadata accurate; avoid repeated conversion literals and duplicated rounding logic; consistently use numeric types and math functions matching the reference data. |
| [#10364](https://github.com/thesofproject/sof/pull/10364) | Do not change production behavior for tests without proving build-configuration semantics. Name fixed-point scales and make assertion failures diagnosable. |
| [#10590](https://github.com/thesofproject/sof/pull/10590) | Keep host builds portable by gating HiFi/Xtensa sources, coupling sample counts, verifying vector lengths, and checking copy-helper status. |
| [#10764](https://github.com/thesofproject/sof/pull/10764) | Match API input types, remove unused headers, replace hard-coded data lengths, and retain SOF-specific references instead of generic floating-point oracles. |

## Pre-Pull-Request Checklist

1. The ztest preserves the legacy test's behavior, data, fixed-point format, and tolerance.
2. No hard-coded vector lengths, uncoupled loop bounds, or unchecked array relationships remain.
3. All input and output types match the tested API without implicit truncation.
4. Every fallible helper result is checked and assertions identify failing data.
5. Metadata, test names, comments, tags, and source dependencies accurately match the test.
6. `native_sim` compiles only portable sources and links all required host libraries.
7. Production-code changes, if any, are independently necessary and safe for firmware builds.
8. The target Twister suite passes locally and its CI result is checked before final review.
