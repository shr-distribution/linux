#!/bin/bash
#
# build-tenderloin.sh - Build kernel for HP TouchPad
#
# This script handles the full build process for the TouchPad kernel:
# - Uses tenderloin_defconfig by default (--defconfig to override)
# - Uses out-of-tree build to keep source tree clean
# - Builds kernel, modules, and device trees
# - Packages for moboot bootloader
# - Optionally deploys to device
#
# Usage:
#   ./scripts/build-tenderloin.sh [options]
#
# Options:
#   -c, --clean          Clean build directory before building
#   -d, --deploy         Deploy to device after successful build
#   -r, --reboot         Reboot device after deploy (implies --deploy)
#   -j, --jobs N         Number of parallel jobs (default: nproc)
#   -v, --variant V      Device variant: topaz (default) or opal
#   --defconfig NAME     Defconfig to use (default: tenderloin_defconfig)
#                        Shorthand: debug, fast, or full name
#   -h, --help           Show this help message
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KERNEL_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${KERNEL_DIR}/../build-output"

# Default options
CLEAN=0
DEPLOY=0
REBOOT=0
JOBS=$(nproc)
VARIANT="topaz"

# Default defconfig
DEFCONFIG="tenderloin_defconfig"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

usage() {
    sed -n '2,/^$/p' "$0" | grep '^#' | sed 's/^# \?//'
    exit 0
}

info() {
    echo -e "${GREEN}==>${NC} $*"
}

warn() {
    echo -e "${YELLOW}WARNING:${NC} $*"
}

error() {
    echo -e "${RED}ERROR:${NC} $*" >&2
    exit 1
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--clean)
            CLEAN=1
            shift
            ;;
        -d|--deploy)
            DEPLOY=1
            shift
            ;;
        -r|--reboot)
            DEPLOY=1
            REBOOT=1
            shift
            ;;
        -j|--jobs)
            JOBS="$2"
            shift 2
            ;;
        -v|--variant)
            VARIANT="$2"
            shift 2
            ;;
        --defconfig)
            DEFCONFIG="$2"
            shift 2
            ;;
        -h|--help)
            usage
            ;;
        *)
            error "Unknown option: $1"
            ;;
    esac
done

# Resolve defconfig shorthand names
case "$DEFCONFIG" in
    debug)    DEFCONFIG="tenderloin_debug_defconfig" ;;
    fast)     DEFCONFIG="tenderloin_fast_defconfig" ;;
    normal)   DEFCONFIG="tenderloin_defconfig" ;;
esac

# Validate defconfig exists
if [[ ! -f "${KERNEL_DIR}/arch/arm/configs/${DEFCONFIG}" ]]; then
    error "Defconfig not found: arch/arm/configs/${DEFCONFIG}"
fi

# Validate variant
if [[ "$VARIANT" != "topaz" && "$VARIANT" != "opal" ]]; then
    error "Invalid variant: $VARIANT (must be 'topaz' or 'opal')"
fi

# Set up environment
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-

# Check for cross-compiler
if ! command -v ${CROSS_COMPILE}gcc &> /dev/null; then
    # Try alternative prefix
    export CROSS_COMPILE=arm-linux-gnueabi-
    if ! command -v ${CROSS_COMPILE}gcc &> /dev/null; then
        error "Cross-compiler not found. Install arm-linux-gnueabihf-gcc or arm-linux-gnueabi-gcc"
    fi
fi

info "Building HP TouchPad kernel (${VARIANT})"
info "  Defconfig: ${DEFCONFIG}"
info "  Build dir: ${BUILD_DIR}"
info "  Jobs: ${JOBS}"

# Create build directory
mkdir -p "${BUILD_DIR}"

# Clean if requested
if [[ $CLEAN -eq 1 ]]; then
    info "Cleaning build directory..."
    rm -rf "${BUILD_DIR:?}"/*
fi

# Configure
if [[ ! -f "${BUILD_DIR}/.config" ]] || [[ $CLEAN -eq 1 ]]; then
    info "Configuring with ${DEFCONFIG}..."
    make -C "${KERNEL_DIR}" O="${BUILD_DIR}" "${DEFCONFIG}"
fi

# Build
info "Building zImage, modules, and dtbs..."
make -C "${KERNEL_DIR}" O="${BUILD_DIR}" -j"${JOBS}" zImage modules dtbs

# Check build success
if [[ ! -f "${BUILD_DIR}/arch/arm/boot/zImage" ]]; then
    error "Build failed: zImage not found"
fi

info "Build successful!"

# Package
info "Packaging for moboot..."
"${SCRIPT_DIR}/pack-uimage.sh" "${VARIANT}"

# Check package success
if [[ ! -f "${BUILD_DIR}/uImage.LuneOS" ]]; then
    error "Packaging failed: uImage.LuneOS not found"
fi

info "Package created: ${BUILD_DIR}/uImage.LuneOS"

# Deploy if requested
if [[ $DEPLOY -eq 1 ]]; then
    DEPLOY_OPTS=""
    if [[ $REBOOT -eq 1 ]]; then
        DEPLOY_OPTS="--reboot"
    fi
    info "Deploying to device..."
    "${SCRIPT_DIR}/deploy-to-touchpad.sh" ${DEPLOY_OPTS}
fi

info "Done!"
