#!/bin/bash
# Test script for maintenance monitor node
# Publishes fake battery data to test the maintenance monitor

echo "Starting battery data publisher for maintenance monitor test..."
echo "This will publish battery data that gradually drains"
echo "Press Ctrl+C to stop"
echo ""

# Start with healthy battery and drain it
voltage=12.4
percentage=1.0

while true; do
    # Publish battery state
    ros2 topic pub --once /battery_state sensor_msgs/BatteryState \
        "{voltage: $voltage, percentage: $percentage, power_supply_status: 1, present: true}"
    
    echo "Published: ${voltage}V (${percentage}%)"
    
    # Drain battery slightly
    voltage=$(echo "$voltage - 0.05" | bc)
    percentage=$(echo "$percentage - 0.02" | bc)
    
    # Reset if too low
    if (( $(echo "$voltage < 10.5" | bc -l) )); then
        echo "Battery critically low - resetting to full"
        voltage=12.4
        percentage=1.0
    fi
    
    sleep 5
done
