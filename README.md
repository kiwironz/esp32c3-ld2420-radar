<document filename="README.md">
# ESP32-C3 LD2420 Radar Sensor
<p>This project is an Arduino-based firmware for an ESP32-C3 microcontroller connected to an LD2420 radar module. It provides presence detection and distance measurement via serial data and a presence pin, with integration for Wi-Fi, MQTT for remote monitoring and configuration, and a web server for status viewing and OTA updates.</p>
<h2>Features</h2>
<ul>
<li><strong>Radar Detection</strong>: Detects presence and distance using the LD2420 module's serial output and OT2 presence pin.</li>
<li><strong>16 Gate Sensitivities</strong>: Configurable sensitivities for 16 detection gates (0-15), allowing fine-tuned detection ranges.</li>
<li><strong>MQTT Integration</strong>: Publishes presence and distance updates; subscribes to configuration and reset commands.</li>
<li><strong>Web Interface</strong>: Real-time status display with presence indicators, distance, system info, and links to OTA update and JSON status.</li>
<li><strong>OTA Updates</strong>: Over-the-air firmware updates via ElegantOTA web page.</li>
<li><strong>Watchdog Timer</strong>: Prevents system hangs using ESP32's task watchdog.</li>
<li><strong>Configurable Reset Time and Distance Publish Interval</strong>: Adjustable via MQTT.</li>
<li><strong>Error Reporting</strong>: Publishes errors via MQTT for debugging (e.g., invalid thresholds, command failures).</li>
</ul>
<h2>Hardware Requirements</h2>
<ul>
<li><strong>Microcontroller</strong>: ESP32-C3 (e.g., ESP32-C3-SuperMini or ESP32-C3-DevKitM-1).</li>
<li><strong>Radar Module</strong>: LD2420 with firmware v2.1 or later (supports 16 gates).</li>
<li><strong>Connections</strong>:
<ul>
<li>ESP32 RX (GPIO8) to LD2420 TX (OT1).</li>
<li>ESP32 TX (GPIO9) to LD2420 RX.</li>
<li>ESP32 GPIO10 to LD2420 OT2 (presence pin).</li>
</ul>
</li>
<li><strong>Power</strong>: Stable 3.3V supply for LD2420 (50mA typical).</li>
</ul>
<h2>Setup</h2>
<h3>Prerequisites</h3>
<ul>
<li><strong>PlatformIO</strong>: Recommended for building and uploading (VSCode extension or CLI).</li>
<li><strong>Libraries</strong>: Automatically installed via <code>platformio.ini</code> (AsyncTCP, AsyncMqttClient, ElegantOTA, ArduinoJson).</li>
<li><strong>MQTT Broker</strong>: A running MQTT server (e.g., Mosquitto) at <code>192.168.5.160:1883</code> with credentials <code>username</code> / <code>password</code> (update in code if different).</li>
</ul>
<h3>Installation</h3>
<ol>
<li><strong>Clone the Repository</strong>:
<pre><code>git clone &#x3C;repository_url>
cd &#x3C;repository_directory>
</code></pre>
</li>
<li><strong>Configure Wi-Fi and MQTT</strong>:
<ul>
<li>Edit <code>main.cpp</code> to update <code>ssid</code>, <code>password</code>, <code>mqttServer</code>, <code>mqttPort</code>, <code>mqttUser</code>, and <code>mqttPassword</code> as needed.</li>
</ul>
</li>
<li><strong>Build and Upload</strong>:
<ul>
<li>Open the project in VSCode with PlatformIO.</li>
<li>Run <code>PlatformIO: Build</code> to compile.</li>
<li>Run <code>PlatformIO: Upload</code> to flash to the ESP32-C3 (connect via USB, select correct port in <code>platformio.ini</code>).</li>
</ul>
</li>
<li><strong>Serial Monitor</strong>:
<ul>
<li>Use <code>PlatformIO: Serial Monitor</code> or an external tool at 115200 baud to view logs.</li>
</ul>
</li>
</ol>
<h2>Usage</h2>
<h3>Boot and Initialization</h3>
<ul>
<li>On boot, the ESP32 connects to Wi-Fi, initializes the LD2420 radar, applies default sensitivities (gates 0-5: 10, gates 6-15: 100), and starts the web server.</li>
<li>Serial output shows initialization steps, including threshold settings and presence pin state.</li>
</ul>
<h3>Presence Detection</h3>
<ul>
<li><strong>Serial Data</strong>: Parses "ON" / "OFF" and "Range X" from LD2420 serial output.</li>
<li><strong>Presence Pin</strong>: Reads GPIO10 (LD2420 OT2) for hardware presence signal.</li>
<li>Combined presence (serial or pin) triggers MQTT updates.</li>
</ul>
<h3>MQTT Topics</h3>
<ul>
<li><strong>Subscribe to State Updates</strong>:
<pre><code>mosquitto_sub -h 192.168.5.160 -p 1883 -u username -P password -t "home/radar/ld2420/state"
</code></pre>
<ul>
<li>Example output on presence detection:
<pre><code>{"presence":true,"serial_presence":true,"pin_presence":true,"distance_cm":130}
</code></pre>
</li>
<li>Example output on no presence:
<pre><code>{"presence":false,"serial_presence":false,"pin_presence":false}
</code></pre>
</li>
</ul>
</li>
<li><strong>Publish Configuration</strong>:
<ul>
<li>Set new sensitivities and reset time (16 values, 0-100):
<pre><code>mosquitto_pub -h 192.168.5.160 -p 1883 -u username -P password -t "home/radar/ld2420/config" -m '{"sensitivities":[10,10,10,10,10,10,100,100,100,100,100,100,100,100,100,100],"reset_time":5}'
</code></pre>
</li>
<li>The code verifies each gate and publishes errors if mismatches occur.</li>
</ul>
</li>
<li><strong>Request Current Configuration</strong>:
<pre><code>mosquitto_pub -h 192.168.5.160 -p 1883 -u username -P password -t "home/radar/ld2420/config/get" -m ""
</code></pre>
<ul>
<li>Sub to the response:
<pre><code>mosquitto_sub -h 192.168.5.160 -p 1883 -u username -P password -t "home/radar/ld2420/config/state"
</code></pre>
</li>
<li>Example output:
<pre><code>{"sensitivities":[10,10,10,10,10,10,100,100,100,100,100,100,100,100,100,100],"reset_time":5}
</code></pre>
</li>
</ul>
</li>
<li><strong>Reboot Device</strong>:
<pre><code>mosquitto_pub -h 192.168.5.160 -p 1883 -u username -P password -t "home/radar/ld2420/reset" -m ""
</code></pre>
</li>
<li><strong>Factory Reset Radar</strong>:
<pre><code>mosquitto_pub -h 192.168.5.160 -p 1883 -u username -P password -t "home/radar/ld2420/reset_factory" -m ""
</code></pre>
<ul>
<li>Resets the LD2420 and reboots the ESP32.</li>
</ul>
</li>
<li><strong>Error Messages</strong>:
<pre><code>mosquitto_sub -h 192.168.5.160 -p 1883 -u username -P password -t "home/radar/ld2420/error"
</code></pre>
<ul>
<li>Examples:
<ul>
<li>"Verification failed for gate 0: expected sensitivity=10, got=58 (high=1000000000, low=1000000000)"</li>
<li>"Invalid thresholds for gate 15: high=1000000000, low=1000000000"</li>
</ul>
</li>
</ul>
</li>
</ul>
<h3>Web Interface</h3>
<ul>
<li><strong>Status Page</strong>: Access at <code>http://&#x3C;ESP_IP_ADDRESS>/</code> (auto-refreshes every 5s).
<ul>
<li>Displays presence status, distance, detection sources (serial/pin), system info (IP, RSSI, uptime, heap, reset time).</li>
<li>Links to OTA update and JSON status.</li>
</ul>
</li>
<li><strong>JSON Status</strong>: <code>http://&#x3C;ESP_IP_ADDRESS>/status</code>
<ul>
<li>Example output:
<pre><code>{"presence":true,"serial_presence":true,"pin_presence":true,"distance_cm":130,"device":"home/radar/ld2420","firmware":"2.1","wifi_rssi":-60,"uptime":3600,"free_heap":120000,"reset_time":5,"distance_interval":10,"sensitivities":[10,10,10,10,10,10,100,100,100,100,100,100,100,100,100,100]}
</code></pre>
</li>
</ul>
</li>
<li><strong>Current Config</strong>: <code>http://&#x3C;ESP_IP_ADDRESS>/config</code>
<ul>
<li>JSON output with sensitivities (as "zones" for compatibility), reset time, distance interval.</li>
</ul>
</li>
<li><strong>OTA Update</strong>: <code>http://&#x3C;ESP_IP_ADDRESS>/update</code>
<ul>
<li>Upload new firmware via web form.</li>
</ul>
</li>
</ul>
<h2>Troubleshooting</h2>
<ul>
<li><strong>No Web Page</strong>:
<ul>
<li>Check serial for "WiFi connected! IP address: X.X.X.X" and "HTTP server started".</li>
<li>Ensure device is on the same network as the browser.</li>
<li>Test root URL first.</li>
<li>Clear browser cache or try incognito mode.</li>
<li>Verify <code>platformio.ini</code> partition scheme supports OTA.</li>
</ul>
</li>
<li><strong>Corrupt Frame Errors</strong>:
<ul>
<li>Indicates UART data mixing. Check connections and baud rate (115200).</li>
<li>Use factory reset if persistent.</li>
</ul>
</li>
<li><strong>Invalid Thresholds</strong>:
<ul>
<li>Run factory reset to clear bad config.</li>
<li>Verify sensitivities after boot.</li>
</ul>
</li>
<li><strong>MQTT Errors</strong>:
<ul>
<li>Check broker credentials and connectivity.</li>
<li>Sub to error topic for diagnostics.</li>
</ul>
</li>
<li><strong>Memory Issues</strong>:
<ul>
<li>Monitor heap via status page. If low, reduce buffer sizes.</li>
</ul>
</li>
</ul>
<h2>License</h2>
<p>MIT License. See LICENSE file for details.
</p></document>
