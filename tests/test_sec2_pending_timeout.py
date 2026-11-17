"""Exercise the Sec+2.0 pending-command timers with a fake clock and queue."""
from pathlib import Path
import subprocess
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[1] / "src/comms.cpp"


class Sec2PendingTimeoutTest(unittest.TestCase):
    def test_pending_timeouts(self):
        source = SOURCE.read_text()
        start = source.index("static _millis_t pendingLightOn = 0;")
        end = source.index("#define SEC1_CMD", start)
        helpers = source[start:end]
        loop_start = source.index("void comms_loop_sec2()")
        loop_end = source.index("void comms_loop_drycontact()", loop_start)
        loop = source[loop_start:loop_end]
        self.assertIn("check_pending_sec2();\n        process_send_queue();", loop)
        self.assertNotIn("pending_timestamp", source)
        self.assertNotIn("pending_sec2_dequeued", source)
        self.assertIn("pendingLightOn = value ? _millis() : 0;", source)
        self.assertIn("pendingDoorCommand = _millis();", source)
        self.assertIn("elapsed > SECPLUS2_PENDING_TIMEOUT && elapsed < 0x80000000", helpers)
        self.assertNotIn("txQueueCount", helpers)
        harness = r"""
#include <cassert>
#include <cstdint>
#ifdef ESP32
using _millis_t = int64_t;
#else
using _millis_t = uint32_t;
#endif
uint32_t now = 0;
_millis_t _millis() { return now; }
const char *TAG = "test";
#define ESP_LOGW(...) ((void)0)
""" + helpers + r"""
int main() {
    pendingLightOn = pendingLockOff = pendingDoorCommand = 100;
    now = 1099; check_pending_sec2();
    assert(pendingLightOn && pendingLockOff && pendingDoorCommand);
    now = 1100; check_pending_sec2();
    assert(pendingLightOn && pendingLockOff && pendingDoorCommand);
    now = 1101; check_pending_sec2();
    assert(!pendingLightOn && !pendingLockOff && !pendingDoorCommand);

    // Opposite requests have independent timeout periods.
    now = 2000;
    pendingLightOn = pendingLockOn = 2000;
    pendingLightOff = pendingLockOff = pendingDoorCommand = 2500;
    now = 3001; check_pending_sec2();
    assert(!pendingLightOn && !pendingLockOn);
    assert(pendingLightOff == 2500 && pendingLockOff == 2500 && pendingDoorCommand == 2500);
    now = 3501; check_pending_sec2();
    assert(!pendingLightOff && !pendingLockOff && !pendingDoorCommand);

    // A concurrent request newer than the sampled time must not underflow into expiry.
    now = 4000; pendingLockOn = 4100; check_pending_sec2();
    assert(pendingLockOn == 4100);
    now = 5100; check_pending_sec2(); assert(pendingLockOn == 4100);
    now = 5101; check_pending_sec2(); assert(!pendingLockOn);

    // Queue contents do not suppress a timeout while awaiting a reply.
    pendingLightOff = 6000; now = 7001;
    check_pending_sec2(); assert(!pendingLightOff);

#ifdef ESP8266
    // The ESP8266 millisecond clock wraps at 32 bits.
    pendingLightOn = UINT32_MAX - 500; now = 498;
    check_pending_sec2(); assert(pendingLightOn);
    now = 499; check_pending_sec2(); assert(pendingLightOn);
    now = 500; check_pending_sec2(); assert(!pendingLightOn);
#endif
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            cpp = Path(tmp) / "pending.cpp"
            cpp.write_text(harness)
            for target in ("ESP8266", "ESP32"):
                with self.subTest(target=target):
                    binary = Path(tmp) / target
                    subprocess.run(["c++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                                    "-D" + target, str(cpp), "-o", str(binary)], check=True)
                    subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
