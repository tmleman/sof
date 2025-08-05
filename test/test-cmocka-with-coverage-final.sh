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
    $0                                    # Build and test unit_test_defconfig with coverage
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

    # Build with coverage flags directly in CMAKE_C_FLAGS
    ( set -x
     cmake  -DINIT_CONFIG="$conf" -S "$SOFTOP" -B "$BUILDTOP"/"$conf" \
            -DBUILD_UNIT_TESTS=ON           \
            -DBUILD_UNIT_TESTS_HOST=ON      \
            -DCMAKE_C_FLAGS="--coverage -O0 -g" \
            -DCMAKE_EXE_LINKER_FLAGS="--coverage"

     cmake --build "$BUILDTOP"/"$conf" -- -j"$(nproc)"
    )
}

run_tests_with_coverage()
{
    local build_dir="$1"
    printf '\n\n    =========  Running native cmocka tests with coverage in %s ======\n\n' "$build_dir"

    # Run tests in build directory
    ( cd "$build_dir" && set -x
     # Run tests
     cmake --build . -- test ARGS="-V"
    )
}

collect_coverage_data()
{
    local config_name="$1"
    local build_dir="$2"

    printf '\n\n    =========  Collecting coverage data for %s ======\n\n' "$config_name"

    local coverage_output_dir="coverage_reports/$config_name"
    mkdir -p "$coverage_output_dir"

    # Check for .gcda files in different locations
    cd "$build_dir"

    # Count coverage files
    local gcda_files=$(find . -name "*.gcda" 2>/dev/null | wc -l)
    local gcno_files=$(find . -name "*.gcno" 2>/dev/null | wc -l)

    printf "Found %d .gcda files and %d .gcno files\n" "$gcda_files" "$gcno_files"

    if [ "$gcda_files" -eq 0 ]; then
        printf '\n    Warning: No .gcda coverage data files found.\n'
        cd - >/dev/null
        return 1
    fi

    # Create initial coverage data file
    printf '\n    Generating coverage info files...\n'

    # Use absolute path for output and ensure directory exists
    local abs_coverage_dir="$(cd ..; pwd)/coverage_reports/$config_name"
    mkdir -p "$abs_coverage_dir"

    ( set -x
      # Capture coverage data
      lcov --capture --directory . --output-file "$abs_coverage_dir/coverage.info" --rc lcov_branch_coverage=1

      # Filter out system files and keep only SOF source files
      if [ -f "$abs_coverage_dir/coverage.info" ]; then
          lcov --remove "$abs_coverage_dir/coverage.info" \
               '/usr/*' \
               '*/test/cmocka/src/*' \
               '*/cmocka_git/*' \
               '*/build*' \
               '*/gcov_data/*' \
               --output-file "$abs_coverage_dir/filtered.info" \
               --rc lcov_branch_coverage=1
      fi
    )

    cd - >/dev/null

    # Generate coverage reports
    if [ -f "$abs_coverage_dir/filtered.info" ]; then
        printf '\n    Generating HTML coverage report...\n'
        ( set -x
          genhtml "$abs_coverage_dir/filtered.info" \
                  --output-directory "$abs_coverage_dir/html" \
                  --title "SOF cmocka Unit Tests Coverage - $config_name" \
                  --show-details --legend --branch-coverage
        )
        printf '\n    HTML Coverage report: %s\n' "file://$abs_coverage_dir/html/index.html"

        # Show coverage summary
        printf '\n    Coverage Summary for %s:\n' "$config_name"
        lcov --summary "$abs_coverage_dir/filtered.info" 2>/dev/null | grep -E "(lines|functions|branches)" || true

        # Show detailed coverage info
        printf '\n    Detailed Coverage Files:\n'
        lcov --list "$abs_coverage_dir/filtered.info" 2>/dev/null | head -20 || true

    else
        printf '\n    Warning: No filtered coverage data found for %s\n' "$config_name"
        # Show what files we actually captured
        if [ -f "$coverage_output_dir/coverage.info" ]; then
            printf '\n    Raw coverage files captured:\n'
            lcov --list "$coverage_output_dir/coverage.info" 2>/dev/null | head -10 || true
        fi
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
    local specific_config="unit_test_defconfig"  # Default to unit_test_defconfig
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

    # Build and test specific config
    if [ ! -f "$SOFTOP/src/arch/xtensa/configs/$specific_config" ]; then
        printf "Error: Configuration file '%s' not found in %s/src/arch/xtensa/configs/\n" \
               "$specific_config" "$SOFTOP" >&2
        exit 1
    fi

    rebuild_config_with_coverage "$specific_config"
    run_tests_with_coverage "$BUILDTOP/$specific_config"
    collect_coverage_data "$specific_config" "$BUILDTOP/$specific_config"

    printf '\n\n    =========  Coverage collection complete! ======\n\n'

    printf 'Open the coverage report in your browser:\n'
    printf '  file://%s\n' "$(pwd)/build_ut_defs_coverage/coverage_reports/$specific_config/html/index.html"

    # Also show a simple way to view coverage in terminal
    printf '\nOr view coverage summary in terminal:\n'
    printf '  lcov --summary coverage_reports/%s/filtered.info\n' "$specific_config"
}

main "$@"
