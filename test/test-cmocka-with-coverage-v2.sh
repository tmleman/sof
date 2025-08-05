#!/bin/sh

set -e

# Absolute
SOFTOP=$(cd "$(dirname "$0")"/.. && pwd)

# Relative
BUILDTOP=build_ut_defs_coverage

print_usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Build and run SOF cmocka unit tests with coverage collection.

OPTIONS:
    --help, -h          Show this help message
    --config CONFIG     Build only specific config (e.g., unit_test_defconfig)
    --clean             Clean build directories before building

EXAMPLES:
    $0                                    # Build and test all configs with coverage
    $0 --config unit_test_defconfig       # Test only unit_test_defconfig
    $0 --clean                           # Clean and rebuild everything

The script will:
1. Build cmocka unit tests with coverage flags enabled
2. Run all tests
3. Collect coverage data using gcov and lcov
4. Generate HTML coverage reports

Coverage reports will be saved in: coverage_reports/
EOF
}

rebuild_config_with_coverage()
{
    local conf="$1"
    mkdir -p "$BUILDTOP"
    printf '\n    =========  Building native cmocka tests with coverage for %s ======\n\n' "$conf"

    # First configure the build with coverage flags
    ( set -x
     cmake  -DINIT_CONFIG="$conf" -S "$SOFTOP" -B "$BUILDTOP"/"$conf" \
            -DBUILD_UNIT_TESTS=ON           \
            -DBUILD_UNIT_TESTS_HOST=ON
    )

    # Now apply coverage flags by modifying the build files
    printf '\n    Adding coverage flags to CMake build...\n'

    # Add coverage flags to all C compile commands
    ( cd "$BUILDTOP"/"$conf" && set -x

      # Find all CMake cache and build files and modify compile flags
      if [ -f "CMakeCache.txt" ]; then
          # Add coverage flags to CMAKE_C_FLAGS
          sed -i 's/CMAKE_C_FLAGS:STRING=/CMAKE_C_FLAGS:STRING=--coverage -fprofile-arcs -ftest-coverage -O0 -g /' CMakeCache.txt || true
          sed -i 's/CMAKE_EXE_LINKER_FLAGS:STRING=/CMAKE_EXE_LINKER_FLAGS:STRING=--coverage -lgcov /' CMakeCache.txt || true
      fi

      # Regenerate build files with new flags
      cmake .

      # Build with coverage
      cmake --build . -- -j"$(nproc)"
    )
}

run_tests_with_coverage()
{
    local build_dir="$1"
    printf '\n\n    =========  Running native cmocka tests with coverage in %s ======\n\n' "$build_dir"

    # Change to build directory to ensure .gcda files are created in the right place
    ( cd "$build_dir" && set -x

     # Set environment variables for gcov
     export GCOV_PREFIX_STRIP=4
     export GCOV_PREFIX=./gcov_data
     mkdir -p gcov_data

     # Run tests
     cmake --build . -- test ARGS="-V"

     # List generated coverage files
     printf '\n    Coverage files generated:\n'
     find . -name "*.gcda" -o -name "*.gcno" | head -10
    )
}

collect_coverage_data()
{
    local config_name="$1"
    local build_dir="$2"

    printf '\n\n    =========  Collecting coverage data for %s ======\n\n' "$config_name"

    local coverage_output_dir="coverage_reports/$config_name"
    mkdir -p "$coverage_output_dir"

    # Change to build directory to search for coverage files
    cd "$build_dir"

    # Check if any .gcda files exist
    local gcda_files=$(find . -name "*.gcda" 2>/dev/null | wc -l)
    local gcno_files=$(find . -name "*.gcno" 2>/dev/null | wc -l)

    printf "Found %d .gcda files and %d .gcno files\n" "$gcda_files" "$gcno_files"

    if [ "$gcda_files" -eq 0 ]; then
        printf '\n    Warning: No .gcda coverage data files found.\n'
        printf '    Trying to manually generate coverage data...\n'

        # Try to run gcov manually on test executables
        printf '    Looking for test executables...\n'
        find . -name "*test*" -type f -executable | head -5

        cd - >/dev/null
        return 1
    fi

    # Find all .gcda files and generate coverage info
    printf '\n    Generating coverage info files...\n'
    ( set -x
      # Capture coverage data
      lcov --capture --directory . --output-file "../$coverage_output_dir/coverage.info" --rc lcov_branch_coverage=1

      # Filter out system files and keep only SOF source files
      if [ -f "../$coverage_output_dir/coverage.info" ]; then
          lcov --remove "../$coverage_output_dir/coverage.info" \
               '/usr/*' \
               '*/test/cmocka/*' \
               '*/cmocka_git/*' \
               '*/build*' \
               --output-file "../$coverage_output_dir/filtered.info" \
               --rc lcov_branch_coverage=1
      fi
    )

    cd - >/dev/null

    # Generate coverage reports
    if [ -f "$coverage_output_dir/filtered.info" ]; then
        printf '\n    Generating HTML coverage report...\n'
        ( set -x
          genhtml "$coverage_output_dir/filtered.info" \
                  --output-directory "$coverage_output_dir/html" \
                  --title "SOF cmocka Unit Tests Coverage - $config_name" \
                  --show-details --legend --branch-coverage
        )
        printf '\n    HTML Coverage report: %s\n' "$(pwd)/$coverage_output_dir/html/index.html"

        # Show coverage summary
        printf '\n    Coverage Summary for %s:\n' "$config_name"
        lcov --summary "$coverage_output_dir/filtered.info" 2>/dev/null | grep -E "(lines|functions|branches)" || true
    else
        printf '\n    Warning: No filtered coverage data found for %s\n' "$config_name"
    fi
}

clean_build_dirs()
{
    printf '\n    =========  Cleaning build directories ======\n\n'
    ( set -x
      rm -rf "$BUILDTOP"
      rm -rf "coverage_reports"
    )
}

main()
{
    local specific_config=""
    local clean_build=false

    # Parse command line arguments
    while [ $# -gt 0 ]; do
        case "$1" in
            --help|-h)
                print_usage
                exit 0
                ;;
            --config)
                specific_config="$2"
                shift 2
                ;;
            --clean)
                clean_build=true
                shift
                ;;
            *)
                printf "Error: Unknown option '%s'\n" "$1" >&2
                print_usage >&2
                exit 1
                ;;
        esac
    done

    # Check for required tools
    if ! command -v lcov >/dev/null 2>&1; then
        printf "Error: lcov is required for coverage collection. Please install it:\n"
        printf "  Ubuntu/Debian: sudo apt-get install lcov\n"
        printf "  Fedora/RHEL: sudo dnf install lcov\n"
        exit 1
    fi

    if ! command -v genhtml >/dev/null 2>&1; then
        printf "Error: genhtml is required for HTML coverage reports. Please install lcov package.\n"
        exit 1
    fi

    # Clean if requested
    if [ "$clean_build" = true ]; then
        clean_build_dirs
    fi

    local defconfig

    if [ -n "$specific_config" ]; then
        # Build and test specific config
        if [ ! -f "$SOFTOP/src/arch/xtensa/configs/$specific_config" ]; then
            printf "Error: Configuration file '%s' not found in %s/src/arch/xtensa/configs/\n" \
                   "$specific_config" "$SOFTOP" >&2
            exit 1
        fi

        rebuild_config_with_coverage "$specific_config"
        run_tests_with_coverage "$BUILDTOP/$specific_config"
        collect_coverage_data "$specific_config" "$BUILDTOP/$specific_config"
    else
        # Build and test only unit_test_defconfig for now
        rebuild_config_with_coverage "unit_test_defconfig"
        run_tests_with_coverage "$BUILDTOP/unit_test_defconfig"
        collect_coverage_data "unit_test_defconfig" "$BUILDTOP/unit_test_defconfig"
    fi

    printf '\n\n    =========  Coverage collection complete! ======\n\n'

    printf 'Open the coverage reports in your browser:\n'
    if [ -n "$specific_config" ]; then
        printf '  %s\n' "$(pwd)/coverage_reports/$specific_config/html/index.html"
    else
        printf '  %s\n' "$(pwd)/coverage_reports/unit_test_defconfig/html/index.html"
    fi
}

main "$@"
