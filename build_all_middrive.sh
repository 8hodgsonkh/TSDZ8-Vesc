#!/bin/bash
# Multi-target build script for TSDZ8 mid-drive VESC firmware
# Builds all supported hardware targets with mid-drive support

set -e

# Directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
OUTPUT_DIR="${SCRIPT_DIR}/release_bins"

# Hardware targets with mid-drive support
TARGETS=(
    "go_foc_s100_d100s"     # Makerx S100 - reference platform
    "75_100"                # Flipsky 75/100
    "75_100_V2"             # Flipsky 75/100 V2
    "mksesc_75_100_old"     # Makerbase 75/100 V1
    "mksesc_75_100_v2"      # Makerbase 75/100 V2
)

echo "==========================================="
echo "TSDZ8 Mid-Drive VESC Firmware Multi-Builder"
echo "==========================================="
echo ""

# Create output directory
mkdir -p "${OUTPUT_DIR}"

# Track results
PASSED=()
FAILED=()

for target in "${TARGETS[@]}"; do
    echo ""
    echo ">>> Building: ${target}"
    echo "-------------------------------------------"
    
    # Clean previous build
    rm -rf "${BUILD_DIR}/${target}"
    
    # Build with parallel jobs
    if make -j16 "${target}"; then
        echo "✓ ${target} built successfully"
        
        # Copy binary to release folder
        if [ -f "${BUILD_DIR}/${target}/${target}.bin" ]; then
            cp "${BUILD_DIR}/${target}/${target}.bin" "${OUTPUT_DIR}/"
            echo "  → Copied to release_bins/${target}.bin"
        fi
        
        PASSED+=("${target}")
    else
        echo "✗ ${target} FAILED"
        FAILED+=("${target}")
    fi
done

echo ""
echo "==========================================="
echo "Build Summary"
echo "==========================================="
echo "Passed: ${#PASSED[@]}/${#TARGETS[@]}"
for t in "${PASSED[@]}"; do
    echo "  ✓ ${t}"
done

if [ ${#FAILED[@]} -gt 0 ]; then
    echo ""
    echo "Failed: ${#FAILED[@]}/${#TARGETS[@]}"
    for t in "${FAILED[@]}"; do
        echo "  ✗ ${t}"
    done
    exit 1
fi

echo ""
echo "All binaries saved to: ${OUTPUT_DIR}/"
ls -la "${OUTPUT_DIR}/"
