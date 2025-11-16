#!/bin/bash
echo ""
echo "=== FTC DEPLOY ==="
echo ""

PATH="/Users/${whoami}/Library/Android/sdk/platform-tools:$PATH"

# Auto-connect + show connection
adb connect 192.168.43.1:5555 2>/dev/null
adb devices | grep 192.168.43.1 | grep device