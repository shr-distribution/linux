#!/bin/bash
#
# QCE Hardware Crypto Verification Script
# Tests whether QCE hardware crypto is being utilized by OpenSSH
#
# Usage: ./scripts/test-qce-crypto.sh

set -e

DEVICE_IP="172.16.42.2"
SSH_PORT="22"
TEST_SIZE_MB=50  # Smaller for faster iteration

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_step() {
    echo -e "${GREEN}[STEP]${NC} $1"
}

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_result() {
    echo -e "${GREEN}[RESULT]${NC} $1"
}

run_on_device() {
    ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -p $SSH_PORT root@$DEVICE_IP "$1" 2>/dev/null
}

echo "==============================================="
echo "  QCE Hardware Crypto Verification Test"
echo "==============================================="
echo ""

# Step 0: Clean up /tmp on device
log_step "Cleaning up test files from device /tmp..."
run_on_device "rm -f /tmp/*.test /tmp/test-* /tmp/qce-* /tmp/*-payload 2>/dev/null; df -h /tmp" | sed 's/^/  /'
echo ""

# Step 1: Check QCE driver status
log_step "Checking QCE driver status..."
if run_on_device "dmesg | grep -i qce | grep -i 'Crypto Engine'" > /tmp/qce-probe.log; then
    log_info "QCE driver probe status:"
    cat /tmp/qce-probe.log | sed 's/^/  /'
else
    log_warn "No QCE probe messages found in dmesg"
fi
echo ""

# Step 2: Check available crypto algorithms
log_step "Checking registered crypto algorithms..."
run_on_device "cat /proc/crypto | grep -A 10 'name.*:.*aes' | head -50" > /tmp/crypto-aes.log
log_info "AES algorithms in /proc/crypto:"
if grep -q "qce" /tmp/crypto-aes.log; then
    log_result "✓ QCE algorithms found!"
    grep -E "name|driver|module|priority" /tmp/crypto-aes.log | sed 's/^/  /'
else
    log_warn "✗ No QCE algorithms found - only software implementations"
    grep -E "name|driver|priority" /tmp/crypto-aes.log | head -20 | sed 's/^/  /'
fi
echo ""

# Step 3: Check OpenSSL engine support
log_step "Checking OpenSSL AF_ALG engine..."
if run_on_device "openssl engine -t -c 2>&1" > /tmp/openssl-engines.log; then
    log_info "OpenSSL engines:"
    cat /tmp/openssl-engines.log | sed 's/^/  /'

    if grep -q "afalg" /tmp/openssl-engines.log; then
        log_result "✓ AF_ALG engine is available"
    else
        log_warn "✗ AF_ALG engine NOT found - OpenSSL won't use QCE"
        log_info "AF_ALG engine is needed to bridge OpenSSL to kernel crypto"
    fi
else
    log_error "Failed to query OpenSSL engines"
fi
echo ""

# Step 4: Create test payload
log_step "Creating ${TEST_SIZE_MB}MB test payload..."
dd if=/dev/urandom of=/tmp/qce-test-payload bs=1M count=$TEST_SIZE_MB 2>&1 | tail -1
echo ""

# Step 5: Clear dmesg buffer and start monitoring
log_step "Preparing to monitor kernel crypto activity..."
run_on_device "dmesg -c > /dev/null"  # Clear dmesg
echo ""

# Step 6: Test AES256-GCM with monitoring
log_step "Testing AES256-GCM cipher with kernel monitoring..."

# Start background dmesg monitor
run_on_device "while true; do dmesg -c 2>/dev/null; sleep 0.5; done" > /tmp/qce-dmesg-monitor.log 2>&1 &
MONITOR_PID=$!
sleep 1

# Run the transfer
log_info "Starting SCP transfer with aes256-gcm..."
START_TIME=$(date +%s.%N)
scp -P $SSH_PORT -c aes256-gcm@openssh.com /tmp/qce-test-payload root@$DEVICE_IP:/var/tmp/qce-test 2>&1 | tail -1
END_TIME=$(date +%s.%N)
ELAPSED=$(echo "$END_TIME - $START_TIME" | bc)
THROUGHPUT=$(echo "scale=2; $TEST_SIZE_MB / $ELAPSED" | bc)

# Stop monitor
sleep 1
kill $MONITOR_PID 2>/dev/null || true
wait $MONITOR_PID 2>/dev/null || true

log_result "Transfer completed: ${ELAPSED}s = ${THROUGHPUT} MB/s"
echo ""

# Step 7: Analyze dmesg output
log_step "Analyzing kernel activity during transfer..."
if grep -iE "qce|adm.*dma|crypto" /tmp/qce-dmesg-monitor.log > /tmp/qce-activity.log 2>/dev/null; then
    log_result "✓ Kernel crypto activity detected:"
    cat /tmp/qce-activity.log | sed 's/^/  /'

    if grep -q "qce" /tmp/qce-activity.log; then
        log_result "✓✓ QCE hardware was actively used!"
    else
        log_warn "Only generic crypto activity - QCE may not be used"
    fi
else
    log_warn "✗ No kernel crypto activity detected during transfer"
    log_info "This suggests SSH is using userspace crypto, not kernel AF_ALG"
fi
echo ""

# Step 8: Compare software vs hardware paths
log_step "Testing pure software ChaCha20 for comparison..."
run_on_device "dmesg -c > /dev/null"  # Clear dmesg

START_TIME=$(date +%s.%N)
scp -P $SSH_PORT -c chacha20-poly1305@openssh.com /tmp/qce-test-payload root@$DEVICE_IP:/var/tmp/qce-test 2>&1 | tail -1
END_TIME=$(date +%s.%N)
ELAPSED_CHACHA=$(echo "$END_TIME - $START_TIME" | bc)
THROUGHPUT_CHACHA=$(echo "scale=2; $TEST_SIZE_MB / $ELAPSED_CHACHA" | bc)

log_result "ChaCha20 (software): ${ELAPSED_CHACHA}s = ${THROUGHPUT_CHACHA} MB/s"

# Check for any crypto activity
sleep 1
run_on_device "dmesg" > /tmp/chacha-dmesg.log 2>/dev/null
if grep -iE "qce|crypto" /tmp/chacha-dmesg.log | grep -v "random" > /dev/null 2>&1; then
    log_warn "Unexpected: kernel crypto activity for ChaCha20 (should be pure software)"
else
    log_info "✓ ChaCha20 used software path as expected (no kernel activity)"
fi
echo ""

# Step 9: Direct QCE test via kernel crypto API
log_step "Testing QCE directly via kernel crypto..."

run_on_device 'cat > /tmp/test-qce-direct.sh' <<'SCRIPT_EOF'
#!/bin/sh
# Direct kernel crypto test using cryptsetup or dm-crypt

# Check if we can use AF_ALG socket directly
if [ -e /proc/sys/crypto ]; then
    echo "Crypto API available"
fi

# Try to benchmark kernel AES via dmsetup if available
if command -v cryptsetup >/dev/null 2>&1; then
    echo "cryptsetup available - can test dm-crypt"
    cryptsetup benchmark 2>&1 | grep -i aes | head -5
else
    echo "cryptsetup not available - skipping dm-crypt test"
fi

# Show which algorithms have non-zero refcnt (actively used)
echo ""
echo "Currently active crypto algorithms:"
cat /proc/crypto | awk '/^name/ {name=$3} /^refcnt/ {if ($3 > 0) print name, "refcnt="$3}'
SCRIPT_EOF

run_on_device "chmod +x /tmp/test-qce-direct.sh && /tmp/test-qce-direct.sh" | sed 's/^/  /'
echo ""

# Step 10: Summary and recommendations
echo "==============================================="
echo "  Summary and Recommendations"
echo "==============================================="
echo ""

log_step "Test Results Summary:"
echo "  AES256-GCM throughput: ${THROUGHPUT} MB/s"
echo "  ChaCha20 throughput: ${THROUGHPUT_CHACHA} MB/s"
echo ""

if [ -s /tmp/qce-activity.log ]; then
    log_result "✓ QCE kernel activity was detected"
    VERDICT="QCE hardware crypto appears to be working"
else
    log_warn "✗ No QCE activity detected during SSH transfer"
    VERDICT="OpenSSH is NOT using QCE hardware"
fi

echo ""
log_result "Verdict: $VERDICT"
echo ""

if ! grep -q "afalg" /tmp/openssl-engines.log 2>/dev/null; then
    log_step "To enable AF_ALG engine (required for OpenSSH to use QCE):"
    echo ""
    echo "1. Add to /etc/ssl/openssl.cnf:"
    echo ""
    cat <<'OPENSSL_CONF'
openssl_conf = openssl_init

[openssl_init]
engines = engine_section

[engine_section]
afalg = afalg_section

[afalg_section]
engine_id = afalg
dynamic_path = /usr/lib/engines-3/afalg.so
default_algorithms = ALL
init = 1
OPENSSL_CONF
    echo ""
    echo "2. Verify with: openssl engine -t -c"
    echo "3. Re-run this test script"
    echo ""
fi

# Cleanup
rm -f /tmp/qce-test-payload /tmp/qce-*.log /tmp/chacha-dmesg.log /tmp/crypto-aes.log /tmp/openssl-engines.log
run_on_device "rm -f /var/tmp/qce-test /tmp/test-qce-direct.sh"

echo "Test complete."
echo ""
