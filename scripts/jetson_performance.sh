#!/bin/bash

# Jetson Performance Control Script
# Manages power modes and clock frequencies for optimal ROVIO performance

show_usage() {
    cat << EOF
Usage: $0 [COMMAND]

Commands:
    max         Enable maximum performance (MAXN + jetson_clocks)
    balanced    Set balanced power mode
    efficient   Set power-efficient mode
    status      Show current power mode and frequencies
    help        Show this help message

Examples:
    sudo $0 max       # Enable max performance for ROVIO
    sudo $0 status    # Check current settings
    sudo $0 balanced  # Return to balanced mode

Note: Most commands require sudo privileges
EOF
}

check_root() {
    if [ "$EUID" -ne 0 ]; then
        echo "❌ ERROR: This command requires sudo privileges"
        echo "   Run: sudo $0 $1"
        exit 1
    fi
}

enable_max_performance() {
    check_root "max"
    
    echo "════════════════════════════════════════════════════════════════"
    echo "   Enabling MAXIMUM PERFORMANCE for Jetson Orin"
    echo "════════════════════════════════════════════════════════════════"
    echo ""
    
    # Set MAXN mode
    echo "📊 Setting power mode to MAXN..."
    nvpmodel -m 0
    if [ $? -eq 0 ]; then
        echo "   ✅ Power mode set to MAXN"
    else
        echo "   ❌ Failed to set power mode"
        exit 1
    fi
    
    # Lock clocks
    echo ""
    echo "🔒 Locking CPU/GPU frequencies to maximum..."
    jetson_clocks
    if [ $? -eq 0 ]; then
        echo "   ✅ Frequencies locked"
    else
        echo "   ❌ Failed to lock frequencies"
        exit 1
    fi
    
    # Show results
    echo ""
    echo "════════════════════════════════════════════════════════════════"
    echo "   ✅ MAXIMUM PERFORMANCE ENABLED"
    echo "════════════════════════════════════════════════════════════════"
    show_status_internal
}

set_balanced_mode() {
    check_root "balanced"
    
    echo "Setting balanced power mode..."
    nvpmodel -m 2
    echo "✅ Balanced mode enabled"
}

set_efficient_mode() {
    check_root "efficient"
    
    echo "Setting power-efficient mode..."
    nvpmodel -m 4
    echo "✅ Efficient mode enabled"
}

show_status_internal() {
    echo ""
    echo "📊 Current Status:"
    echo ""
    
    # Power mode
    echo "Power Mode:"
    nvpmodel -q 2>/dev/null | grep "NV Power Mode" || echo "  Unable to read"
    
    echo ""
    echo "CPU Frequencies:"
    for i in 0 1 2 3; do
        if [ -f /sys/devices/system/cpu/cpu$i/cpufreq/scaling_cur_freq ]; then
            freq=$(cat /sys/devices/system/cpu/cpu$i/cpufreq/scaling_cur_freq)
            freq_ghz=$(awk "BEGIN {printf \"%.2f\", $freq/1000000}")
            echo "  CPU $i: ${freq_ghz} GHz"
        fi
    done
    
    echo ""
}

show_status() {
    echo "════════════════════════════════════════════════════════════════"
    echo "   Jetson Orin Performance Status"
    echo "════════════════════════════════════════════════════════════════"
    show_status_internal
}

# Main
case "${1:-help}" in
    max)
        enable_max_performance
        ;;
    balanced)
        set_balanced_mode
        show_status
        ;;
    efficient)
        set_efficient_mode
        show_status
        ;;
    status)
        show_status
        ;;
    help|--help|-h)
        show_usage
        ;;
    *)
        echo "❌ Unknown command: $1"
        echo ""
        show_usage
        exit 1
        ;;
esac
