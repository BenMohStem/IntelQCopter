from pathlib import Path
import json
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
p=r/'lessons/xbox-test-menu/main.cpp'
s=p.read_text()
marker='static bool write_register(uint8_t reg, uint8_t value)'
helper='''// DevKitC-1 addressable RGB LED. The separate power LED cannot be controlled.
// Override at build time for boards whose RGB LED is wired to another GPIO.
#ifndef XBOX_LED_GPIO
#define XBOX_LED_GPIO 48
#endif

static void connection_led(uint32_t now, bool connected, bool fresh)
{
    static int previous = -1;
    // Search: 80 ms per edge. Connected but waiting/stale: 500 ms per edge.
    bool on = fresh || ((now / (connected ? 500 : 80)) % 2 == 0);
    if ((int)on != previous)
    {
        rgbLedWrite(XBOX_LED_GPIO, 0, on ? 24 : 0, 0);
        previous = on;
    }
}

'''
assert marker in s and 'static void connection_led' not in s
s=s.replace(marker,helper+marker)
s=s.replace('    xbox.begin();\n    for (;;)', '''    connection_led(millis(), false, false);
    xbox.begin();
    // Request scan-response data as well as advertisements for discovery.
    NimBLEDevice::getScan()->setActiveScan(true);
    Serial.printf("XBOX_LED,GPIO=%u,SEARCH_EDGE_MS=80,WAIT_EDGE_MS=500,READY=STEADY_GREEN\\n", XBOX_LED_GPIO);
    for (;;)''')
needle='''        uint8_t buttons = 0;'''
s=s.replace(needle,'''        connection_led(now, connected, fresh);
        uint8_t buttons = 0;''')
s=s.replace('''            Serial.printf("XBOX,CONNECTED=%u,FRESH=%u,CORE=%d,HEAP=%u,MIN_HEAP=%u\\n",
                connected, fresh, xPortGetCoreID(), ESP.getFreeHeap(), ESP.getMinFreeHeap());''','''            Serial.printf("XBOX,CONNECTED=%u,FRESH=%u,CORE=%d,HEAP=%u,MIN_HEAP=%u,SCANNING=%u,FAILED=%lu\\n",
                connected, fresh, xPortGetCoreID(), ESP.getFreeHeap(), ESP.getMinFreeHeap(),
                xbox.isScanning(), (unsigned long)xbox.countFailedConnection);''')
p.write_text(s,encoding='utf-8',newline='\n')
out=r/'measurements/software/2026-10-30-xbox-led'
out.mkdir(exist_ok=True)
print('Updated menu discovery and RGB status indicator; no motor/calibration changes.')
