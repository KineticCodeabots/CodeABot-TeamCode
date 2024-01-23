@echo off
netsh wlan connect name="23389-RC" ssid="23389-RC" interface="Wi-Fi"
timeout /t 5
adb connect 192.168.43.1:5555