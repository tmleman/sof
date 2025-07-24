# Feature Request

## **Problem Description**

The current SOF unit test infrastructure uses CMock/CMocka framework, which creates problems:

1. **Framework Mismatch**: SOF is transitioning to Zephyr RTOS, but unit tests still use CMock instead of Zephyr's native testing framework
2. **Integration Challenges**: CMock tests don't integrate well with Zephyr's development workflow and Twister test runner
3. **Platform Fragmentation**: 15 different platform configurations with varying test counts (48-56 tests)
4. **Developer Experience**: Context-switching between different testing frameworks

## **Proposed Solution**

Migrate all 56 existing CMock-based unit tests to Zephyr's native ztest framework:

**Goals:**
- Complete framework migration from CMock to ztest
- Unified testing experience aligned with Zephyr ecosystem
- Enhanced CI/CD integration with Twister test runner
- Maintain functional equivalence of all existing tests

**Key Features:**
- All 56 tests migrated (18 audio components, 10 library functions, 28 mathematical functions)
- Platform consistency across all 15 configurations
- Split architecture: basic unit tests (native_sim only) vs integration tests (multi-platform)

## **Current State**

- **56 tests** across 15 platform configurations using CMock
- **Proof of Concept**: Working ztest implementation in development branch (not yet integrated)

### Platform Configurations

| Platform Configuration | Test Count | Primary Target |
|------------------------|------------|----------------|
| `unit_test_defconfig` | 56 tests | Generic UT framework |
| `acp_6_3_defconfig` | 50 tests | AMD ACP 6.3 |
| `acp_7_0_defconfig` | 50 tests | AMD ACP 7.0 |
| `imx8_defconfig` | 48 tests | NXP i.MX8 |
| `imx8m_defconfig` | 48 tests | NXP i.MX8M |
| `imx8ulp_defconfig` | 48 tests | NXP i.MX8ULP |
| `imx8x_defconfig` | 48 tests | NXP i.MX8X |
| `mt8186_defconfig` | 48 tests | MediaTek MT8186 |
| `mt8188_defconfig` | 48 tests | MediaTek MT8188 |
| `mt8195_defconfig` | 48 tests | MediaTek MT8195 |
| `mt8196_defconfig` | 48 tests | MediaTek MT8196 |
| `mt8365_defconfig` | 48 tests | MediaTek MT8365 |
| `rembrandt_defconfig` | 48 tests | AMD Rembrandt |
| `renoir_defconfig` | 48 tests | AMD Renoir |
| `vangogh_defconfig` | 48 tests | AMD Van Gogh |

**Note**: All platform configurations are built in CI using native host compilers (not cross-compilation). The tests executed across different platform configurations are largely identical, with minimal platform-specific differences. This results in significant redundancy in CI builds, where the same test logic is compiled and executed multiple times with only minor configuration variations. The proposed ztest architecture aims to eliminate this redundancy by distinguishing between basic unit tests (native_sim only) and platform-specific integration tests.

### Complete Test Migration Status

**Audio Components (18 tests)**

| Test Name | Status | PR Link | Notes |
|-----------|--------|---------|-------|
| `buffer_copy` | ⏳ Pending | - | Buffer management |
| `buffer_new` | ⏳ Pending | - | Buffer creation |
| `buffer_wrap` | ⏳ Pending | - | Buffer wrapping |
| `buffer_write` | ⏳ Pending | - | Buffer write operations |
| `comp_set_state` | ⏳ Pending | - | Component state management |
| `pcm_float_generic` | ⏳ Pending | - | PCM format conversion |
| `mixer` | ⏳ Pending | - | Audio mixing |
| `pipeline_new` | ⏳ Pending | - | Pipeline creation |
| `pipeline_connect_upstream` | ⏳ Pending | - | Pipeline connection |
| `pipeline_free` | ⏳ Pending | - | Pipeline cleanup |
| `volume_process` | ⏳ Pending | - | Volume processing |
| `mux_get_processing_function` | ⏳ Pending | - | MUX function selection |
| `mux_copy` | ⏳ Pending | - | MUX copy operations |
| `demux_copy` | ⏳ Pending | - | DEMUX copy operations |
| `selector_test` | ⏳ Pending | - | Channel selector |
| `eq_iir_process` | ⏳ Pending | - | IIR equalizer |
| `eq_fir_process` | ⏳ Pending | - | FIR equalizer |
| `drc_math_test` | ⏳ Pending | - | Dynamic range compression |

**Library Functions (10 tests)**

| Test Name | Status | PR Link | Notes |
|-----------|--------|---------|-------|
| `rstrcmp` | ❌ Blocked | [Comment](https://github.com/thesofproject/sof/issues/10110#issuecomment-3113547909) | String comparison |
| `rstrlen` | ❌ Blocked | [Comment](https://github.com/thesofproject/sof/issues/10110#issuecomment-3113547909) | String length |
| `fast-get-tests` | ✅ Complete | [#10136](https://github.com/thesofproject/sof/pull/10136) | Fast getter functions |
| `list_init` | ✅ Complete | [#10066](https://github.com/thesofproject/sof/pull/10066) | List initialization (PoC exists) |
| `list_is_empty` | ✅ Complete | [#10066](https://github.com/thesofproject/sof/pull/10066) | List empty check (PoC exists) |
| `list_item_append` | ✅ Complete | [#10066](https://github.com/thesofproject/sof/pull/10066) | List append (PoC exists) |
| `list_item_del` | ✅ Complete | [#10066](https://github.com/thesofproject/sof/pull/10066) | List deletion (PoC exists) |
| `list_item_is_last` | ✅ Complete | [#10066](https://github.com/thesofproject/sof/pull/10066) | List last item check (PoC exists) |
| `list_item_prepend` | ✅ Complete | [#10066](https://github.com/thesofproject/sof/pull/10066) | List prepend (PoC exists) |
| `list_item` | ✅ Complete | [#10066](https://github.com/thesofproject/sof/pull/10066) | List item operations (PoC exists) |

**Mathematical Functions (28 tests)**

*Arithmetic Functions (6 tests)*

| Test Name | Status | PR Link | Notes |
|-----------|--------|---------|-------|
| `gcd` | ✅ Complete | [#10138](https://github.com/thesofproject/sof/pull/10138) | Greatest common divisor |
| `ceil_divide` | ✅ Complete | [#10138](https://github.com/thesofproject/sof/pull/10138) | Ceiling division |
| `find_equal_int16` | ✅ Complete | [#10138](https://github.com/thesofproject/sof/pull/10138) | Find equal int16 values |
| `find_min_int16` | ✅ Complete | [#10138](https://github.com/thesofproject/sof/pull/10138) | Find minimum int16 |
| `find_max_abs_int32` | ✅ Complete | [#10138](https://github.com/thesofproject/sof/pull/10138) | Find max absolute int32 |
| `norm_int32` | ✅ Complete | [#10138](https://github.com/thesofproject/sof/pull/10138) | Normalize int32 |

*Trigonometry Functions (9 tests)*

| Test Name | Status | PR Link | Notes |
|-----------|--------|---------|-------|
| `sin_32b_fixed` | ✅ Complete | [#10233](https://github.com/thesofproject/sof/pull/10233) | 32-bit fixed sine |
| `cos_32b_fixed` | ✅ Complete | [#10233](https://github.com/thesofproject/sof/pull/10233) | 32-bit fixed cosine |
| `sin_16b_fixed` | ✅ Complete | [#10233](https://github.com/thesofproject/sof/pull/10233) | 16-bit fixed sine |
| `cos_16b_fixed` | ✅ Complete | [#10233](https://github.com/thesofproject/sof/pull/10233) | 16-bit fixed cosine |
| `asin_32b_fixed` | ✅ Complete | [#10233](https://github.com/thesofproject/sof/pull/10233) | 32-bit fixed arcsine |
| `acos_32b_fixed` | ✅ Complete | [#10233](https://github.com/thesofproject/sof/pull/10233) | 32-bit fixed arccosine |
| `asin_16b_fixed` | ✅ Complete | [#10233](https://github.com/thesofproject/sof/pull/10233) | 16-bit fixed arcsine |
| `acos_16b_fixed` | ✅ Complete | [#10233](https://github.com/thesofproject/sof/pull/10233) | 16-bit fixed arccosine |
| `lut_sin_16b_fixed` | ✅ Complete | [#10233](https://github.com/thesofproject/sof/pull/10233) | 16-bit sine lookup table |

*Advanced Math Functions (6 tests)*

| Test Name | Status | PR Link | Notes |
|-----------|--------|---------|-------|
| `scalar_power` | ✅ Complete | [#10364](https://github.com/thesofproject/sof/pull/10364) | Scalar power operations - test implementation complete |
| `base2_logarithm` | ✅ Complete | [#10590](https://github.com/thesofproject/sof/pull/10590) | Base-2 logarithm |
| `exponential` | ✅ Complete | [#10590](https://github.com/thesofproject/sof/pull/10590) | Exponential function |
| `square_root` | 🔄 In Progress | [#10764](https://github.com/thesofproject/sof/pull/10764) | Square root |
| `base_10_logarithm` | 🔄 In Progress | [#10764](https://github.com/thesofproject/sof/pull/10764) | Base-10 logarithm |
| `base_e_logarithm` | 🔄 In Progress | [#10764](https://github.com/thesofproject/sof/pull/10764) | Natural logarithm |

*Audio Processing Math (7 tests)*

| Test Name | Status | PR Link | Notes |
|-----------|--------|---------|-------|
| `a_law_codec` | ⏳ Pending | - | A-law codec |
| `mu_law_codec` | ⏳ Pending | - | Mu-law codec |
| `fft` | ⏳ Pending | - | Fast Fourier Transform |
| `window` | ⏳ Pending | - | Window functions |
| `matrix` | ⏳ Pending | - | Matrix operations |
| `auditory` | ⏳ Pending | - | Auditory processing |
| `dct` | ⏳ Pending | - | Discrete Cosine Transform |

**Status Legend:**
- ⏳ **Pending**: Not yet started
- 🟡 **PoC Ready**: Proof of concept exists, ready for integration
- 🔄 **In Progress**: Currently being migrated
- ✅ **Complete**: Migration completed and merged
- ❌ **Blocked**: Migration blocked by dependencies

# Implementation Plan

## Math Tests Migration Strategy

### Current CMock Structure Analysis

The math tests are organized into 8 subcategories with clear separation:

```
sof/test/cmocka/src/math/
├── arithmetic/     # Codecs & Advanced Math (6 tests)
├── numbers/        # Basic Arithmetic (6 tests)
├── trig/           # Trigonometry (9 tests)
├── fft/            # FFT (1 test)
├── window/         # Window Functions (1 test)
├── matrix/         # Matrix Operations (1 test)
├── auditory/       # Auditory Processing (1 test)
└── dct/            # DCT (1 test)
```

### Proposed Ztest Directory Structure

Based on the logical grouping and complexity analysis, the following structure is proposed:

```
sof/test/ztest/unit/math/
├── basic/                          # Simple, fast unit tests
│   ├── arithmetic/                 # Basic number operations (6 tests)
│   │   ├── test_gcd_ztest.c
│   │   ├── test_ceil_divide_ztest.c
│   │   ├── test_find_equal_int16_ztest.c
│   │   ├── test_find_min_int16_ztest.c
│   │   ├── test_find_max_abs_int32_ztest.c
│   │   ├── test_norm_int32_ztest.c
│   │   ├── CMakeLists.txt
│   │   ├── testcase.yaml
│   │   └── prj.conf
│   └── trigonometry/               # Trig functions (9 tests)
│       ├── test_sin_32b_fixed_ztest.c
│       ├── test_cos_32b_fixed_ztest.c
│       ├── test_sin_16b_fixed_ztest.c
│       ├── test_cos_16b_fixed_ztest.c
│       ├── test_asin_32b_fixed_ztest.c
│       ├── test_acos_32b_fixed_ztest.c
│       ├── test_asin_16b_fixed_ztest.c
│       ├── test_acos_16b_fixed_ztest.c
│       ├── test_lut_sin_16b_fixed_ztest.c
│       ├── CMakeLists.txt
│       ├── testcase.yaml
│       └── prj.conf
├── advanced/                       # Complex mathematical operations
│   ├── functions/                  # Advanced math functions (6 tests)
│   │   ├── test_scalar_power_ztest.c
│   │   ├── test_base2_logarithm_ztest.c
│   │   ├── test_exponential_ztest.c
│   │   ├── test_square_root_ztest.c
│   │   ├── test_base_10_logarithm_ztest.c
│   │   ├── test_base_e_logarithm_ztest.c
│   │   ├── CMakeLists.txt
│   │   ├── testcase.yaml
│   │   └── prj.conf
│   └── codecs/                     # Audio codec functions (2 tests)
│       ├── test_a_law_codec_ztest.c
│       ├── test_mu_law_codec_ztest.c
│       ├── CMakeLists.txt
│       ├── testcase.yaml
│       └── prj.conf
└── dsp/                           # Digital Signal Processing (5 tests)
    ├── transforms/                # FFT, DCT (2 tests)
    │   ├── test_fft_ztest.c
    │   ├── test_dct_ztest.c
    │   ├── CMakeLists.txt
    │   ├── testcase.yaml
    │   └── prj.conf
    ├── processing/                # Window, Matrix, Auditory (3 tests)
    │   ├── test_window_ztest.c
    │   ├── test_matrix_ztest.c
    │   ├── test_auditory_ztest.c
    │   ├── CMakeLists.txt
    │   ├── testcase.yaml
    │   └── prj.conf
    └── shared/                    # Common DSP test utilities
        ├── math_test_data.h       # Shared test vectors
        └── dsp_test_utils.h       # Common DSP test functions
```

### Migration Phases

**Phase 2A: Basic Math Functions (12 tests)**
- Priority: High (foundation for other math tests)
- Target Platform: `native_sim` only
- Complexity: Low - simple numerical operations
- Dependencies: Minimal SOF dependencies

```
math/basic/arithmetic/     (6 tests) - Week 1
math/basic/trigonometry/   (9 tests) - Week 2-3
```

**Phase 2B: Advanced Math Functions (8 tests)**
- Priority: Medium
- Target Platform: `native_sim` only
- Complexity: Medium - more complex algorithms
- Dependencies: SOF math libraries

```
math/advanced/functions/   (6 tests) - Week 4
math/advanced/codecs/      (2 tests) - Week 5
```

**Phase 2C: DSP Functions (8 tests)**
- Priority: Medium-High (used by audio components)
- Target Platform: `native_sim` primarily, may need additional platforms
- Complexity: High - complex algorithms with reference data
- Dependencies: SOF DSP libraries, test reference data

```
math/dsp/transforms/       (2 tests) - Week 6
math/dsp/processing/       (3 tests) - Week 7-8
```

### Test Complexity Analysis

| Category | Complexity | Reasoning | Migration Effort |
|----------|------------|-----------|------------------|
| **Basic Arithmetic** | Low | Simple numerical operations, minimal dependencies | 1-2 days per test |
| **Trigonometry** | Low-Medium | Mathematical functions with reference values | 2-3 days per test |
| **Advanced Functions** | Medium | Complex algorithms, precision requirements | 3-4 days per test |
| **Codecs** | Medium | Audio-specific algorithms, test vectors | 3-4 days per test |
| **DSP Transforms** | High | Complex algorithms, large reference datasets | 5-7 days per test |
| **DSP Processing** | High | Multi-dimensional data, complex validation | 5-7 days per test |

### Technical Considerations

**Reference Data Management:**
- Many math tests use MATLAB-generated reference data (.h files)
- Reference data will be preserved and moved to appropriate test directories
- Some tests may need reference data regeneration for validation

**Test Vector Strategy:**
- Large test vectors (>1KB) will be kept in separate header files
- Small test vectors will be embedded directly in test files
- Shared test vectors will be placed in `math/shared/` directory

**Platform Requirements:**
- All basic and advanced math tests: `native_sim` only
- DSP tests: `native_sim` + selected platforms if precision issues arise
- No cross-compilation requirements for math tests

**Dependencies:**
- Basic tests: Minimal SOF math library dependencies
- Advanced tests: SOF math libraries + potentially external math libraries
- DSP tests: Full SOF audio processing stack components

## Phase-Based Approach

## Phase-Based Approach

| Phase | Scope | Key Deliverables |
|-------|-------|------------------|
| ~~Phase 1~~ | ~~PoC Integration~~ | ~~Merge PoC to main, establish dual architecture~~ |
| **Phase 2** | Basic Unit Tests | Math, string, library functions (native_sim only) |
| **Phase 3** | Advanced Unit Tests | Complex math, audio processing math (native_sim only) |
| **Phase 4** | Integration Tests | Pipeline, modules, audio components (multi-platform) |

## Test Architecture

1. **Basic Unit Tests** (`sof/test/ztest/unit/`)
   - Platform: `native_sim` only
   - Scope: Mathematical functions, basic library functions
   - Execution: Fast, isolated testing

2. **Integration Tests** (`sof/test/ztest/integration/`)
   - Platform: `native_sim` in CI, with additional platforms enabled as needed (subset of current 15 configurations)
   - Scope: Pipeline management, audio modules, component interaction
   - Execution: Complex scenarios, real-world use cases

**Migration Approach:**
- **All tests** will initially be ported to run on `native_sim` platform and executed in CI
- **Complex tests** identified during migration will additionally be enabled on other platforms, requiring collaboration with specific platform developers
- **CI Impact**: No change to current CI behavior - tests will continue to run natively on the host, maintaining the same execution model as today's CMock tests
- **Platform-specific work**: Only complex integration tests will require platform maintainer involvement for multi-platform validation

## Success Criteria

- [ ] 100% test migration completion (56/56 tests)
- [ ] Zero regression in test coverage or functionality

## Benefits

- Unified testing framework aligned with SOF's Zephyr transition
- Improved developer productivity and reduced context switching
- Enhanced CI/CD integration through Twister test runner
- Future-proof testing infrastructure

## ToDo

- What about `sof/zephyr/test`?
- What about `sof/test/cmocka/src/lib/alloc`?
