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
    --coverage-only     Only generate coverage reports (skip building/testing)
    --coverage-format   Coverage report format: html|lcov|both (default: html)

EXAMPLES:
    $0                                    # Build and test all configs with coverage
    $0 --config unit_test_defconfig       # Test only unit_test_defconfig
    $0 --clean                           # Clean and rebuild everything
    $0 --coverage-only                   # Just generate coverage reports
    $0 --coverage-format both           # Generate both HTML and LCOV reports

The script will:
1. Build cmocka unit tests with coverage flags enabled
2. Run all tests
3. Collect coverage data using gcov
4. Generate coverage reports using lcov

Coverage reports will be saved in: coverage_reports/
EOF
}

rebuild_config_with_coverage()
{
    local conf="$1"
    mkdir -p "$BUILDTOP"
    printf '\n    =========  Building native cmocka tests with coverage for %s ======\n\n' "$conf"

    ( set -x
     cmake  -DINIT_CONFIG="$conf" -S "$SOFTOP" -B "$BUILDTOP"/"$conf" \
            -DBUILD_UNIT_TESTS=ON           \
            -DBUILD_UNIT_TESTS_HOST=ON      \
            -DCMAKE_C_FLAGS="--coverage -fprofile-arcs -ftest-coverage -O0 -g" \
            -DCMAKE_CXX_FLAGS="--coverage -fprofile-arcs -ftest-coverage -O0 -g" \
            -DCMAKE_EXE_LINKER_FLAGS="--coverage -lgcov"

     cmake --build "$BUILDTOP"/"$conf" -- -j"$(nproc)"
    )
}

run_tests_with_coverage()
{
    local build_dir="$1"
    printf '\n\n    =========  Running native cmocka tests with coverage in %s ======\n\n' "$build_dir"

    # Change to build directory to ensure .gcda files are created in the right place
    ( cd "$build_dir" && set -x
     cmake --build . -- test ARGS="-V"
    )
}

collect_coverage_data()
{
    local config_name="$1"
    local build_dir="$2"
    local coverage_format="$3"

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
        printf '\n    Warning: No .gcda coverage data files found. Tests may not have run or coverage may not be enabled.\n'
        printf '    Checking if coverage flags were applied correctly...\n'

        # Check if any executables exist and show their compilation flags
        local test_executables=$(find . -name "*.dir" -type d | head -3)
        if [ -n "$test_executables" ]; then
            printf '    Sample build directories found:\n'
            echo "$test_executables"
        fi

        cd - >/dev/null
        return 1
    fi

    # Find all .gcda files and generate coverage info
    ( set -x
      # Initialize lcov
      lcov --capture --initial --directory . --output-file "../$coverage_output_dir/baseline.info" 2>/dev/null || true

      # Capture coverage data after test execution
      lcov --capture --directory . --output-file "../$coverage_output_dir/coverage.info" 2>/dev/null || true

      # Combine baseline and test coverage data
      if [ -f "../$coverage_output_dir/baseline.info" ] && [ -f "../$coverage_output_dir/coverage.info" ]; then
          lcov --add-tracefile "../$coverage_output_dir/baseline.info" \
               --add-tracefile "../$coverage_output_dir/coverage.info" \
               --output-file "../$coverage_output_dir/total.info"
      elif [ -f "../$coverage_output_dir/coverage.info" ]; then
          cp "../$coverage_output_dir/coverage.info" "../$coverage_output_dir/total.info"
      fi

      # Filter out system files and test files (keep only SOF source files)
      if [ -f "../$coverage_output_dir/total.info" ]; then
          lcov --remove "../$coverage_output_dir/total.info" \
               '/usr/*' \
               '*/test/*' \
               '*/cmocka/*' \
               '*/build*' \
               --output-file "../$coverage_output_dir/filtered.info"
      fi
    )

    cd - >/dev/null

    # Generate coverage reports
    if [ -f "$coverage_output_dir/filtered.info" ]; then
        case "$coverage_format" in
            html|both)
                printf '\n    Generating HTML coverage report...\n'
                ( set -x
                  genhtml "$coverage_output_dir/filtered.info" \
                          --output-directory "$coverage_output_dir/html" \
                          --title "SOF cmocka Unit Tests Coverage - $config_name" \
                          --show-details --legend
                )
                printf '\n    HTML Coverage report: %s\n' "$(pwd)/$coverage_output_dir/html/index.html"
                ;;
        esac

        case "$coverage_format" in
            lcov|both)
                printf '\n    LCOV coverage report: %s\n' "$(pwd)/$coverage_output_dir/filtered.info"
                ;;
        esac

        # Show coverage summary
        printf '\n    Coverage Summary for %s:\n' "$config_name"
        lcov --summary "$coverage_output_dir/filtered.info" 2>/dev/null | grep -E "(lines|functions|branches)" || true
    else
        printf '\n    Warning: No coverage data found for %s\n' "$config_name"
    fi
}

generate_combined_coverage()
{
    local coverage_format="$1"

    printf '\n\n    =========  Generating combined coverage report ======\n\n'

    mkdir -p "coverage_reports/combined"

    # Combine all filtered coverage files
    local lcov_add_args=""
    for info_file in coverage_reports/*/filtered.info; do
        if [ -f "$info_file" ]; then
            lcov_add_args="$lcov_add_args --add-tracefile $info_file"
        fi
    done

    if [ -n "$lcov_add_args" ]; then
        ( set -x
          lcov $lcov_add_args --output-file "coverage_reports/combined/total.info"
        )

        case "$coverage_format" in
            html|both)
                printf '\n    Generating combined HTML coverage report...\n'
                ( set -x
                  genhtml "coverage_reports/combined/total.info" \
                          --output-directory "coverage_reports/combined/html" \
                          --title "SOF cmocka Unit Tests - Combined Coverage" \
                          --show-details --legend
                )
                printf '\n    Combined HTML Coverage report: %s\n' "$(pwd)/coverage_reports/combined/html/index.html"
                ;;
        esac

        case "$coverage_format" in
            lcov|both)
                printf '\n    Combined LCOV coverage report: %s\n' "$(pwd)/coverage_reports/combined/total.info"
                ;;
        esac

        # Show combined coverage summary
        printf '\n    Combined Coverage Summary:\n'
        lcov --summary "coverage_reports/combined/total.info" 2>/dev/null | grep -E "(lines|functions|branches)" || true
    else
        printf '\n    Warning: No coverage data files found to combine\n'
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
    local coverage_only=false
    local coverage_format="html"

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
            --coverage-only)
                coverage_only=true
                shift
                ;;
            --coverage-format)
                coverage_format="$2"
                case "$coverage_format" in
                    html|lcov|both)
                        ;;
                    *)
                        printf "Error: Invalid coverage format '%s'. Use html, lcov, or both.\n" "$coverage_format" >&2
                        exit 1
                        ;;
                esac
                shift 2
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

    # Skip building/testing if coverage-only mode
    if [ "$coverage_only" = false ]; then
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
        else
            # Build and test all configurations
            for d in "$SOFTOP"/src/arch/xtensa/configs/*_defconfig; do
                defconfig=$(basename "$d")
                rebuild_config_with_coverage "$defconfig"
            done

            # Run all the tests
            for d in "$BUILDTOP"/*_defconfig; do
                run_tests_with_coverage "$d"
            done
        fi
    fi

    # Collect coverage data
    if [ -n "$specific_config" ]; then
        collect_coverage_data "$specific_config" "$BUILDTOP/$specific_config" "$coverage_format"
    else
        for d in "$BUILDTOP"/*_defconfig; do
            config_name=$(basename "$d")
            collect_coverage_data "$config_name" "$d" "$coverage_format"
        done

        # Generate combined coverage report
        generate_combined_coverage "$coverage_format"
    fi

    printf '\n\n    =========  Coverage collection complete! ======\n\n'

    if [ "$coverage_format" = "html" ] || [ "$coverage_format" = "both" ]; then
        printf 'Open the coverage reports in your browser:\n'
        if [ -n "$specific_config" ]; then
            printf '  %s\n' "$(pwd)/coverage_reports/$specific_config/html/index.html"
        else
            printf '  Combined: %s\n' "$(pwd)/coverage_reports/combined/html/index.html"
            printf '  Individual configs in: %s\n' "$(pwd)/coverage_reports/"
        fi
    fi
}

main "$@"
