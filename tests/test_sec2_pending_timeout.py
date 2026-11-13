"""Exercise Sec+2.0 pending-command generations and timeouts with a fake clock."""
from pathlib import Path
import subprocess
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[1] / "src/comms.cpp"


class Sec2PendingTimeoutTest(unittest.TestCase):
    def test_pending_timeouts_follow_queued_generation(self):
        source = SOURCE.read_text()
        start = source.index("#define SECPLUS2_PENDING_TIMEOUT")
        end = source.index("#define SEC1_CMD", start)
        helpers = source[start:end]
        loop_start = source.index("void comms_loop_sec2()")
        loop_end = source.index("void comms_loop_drycontact()", loop_start)
        loop = source[loop_start:loop_end]
        self.assertIn("check_pending_sec2();\n        process_send_queue();", loop)
        self.assertIn("pending_sec2_sent(pkt_ac);", source)
        self.assertIn("pending_sec2_dropped(pkt_ac);", source)
        self.assertIn("pkt_ac.pending_generation = pendingGeneration;", source)
        self.assertIn("pendingSec2QueueMutex = xSemaphoreCreateMutex();", source)
        for signature in (
            "bool process_send_queue()\n{\n    PendingSec2QueueGuard queueGuard;",
            "bool door_command(DoorAction action)\n{\n    PendingSec2QueueGuard queueGuard;",
            "bool set_lock(bool value, bool verify)\n{\n    PendingSec2QueueGuard queueGuard;",
            "bool set_light(bool value, bool verify)\n{\n    PendingSec2QueueGuard queueGuard;",
        ):
            self.assertIn(signature, source)
        harness = r"""
#include <cassert>
#include <cstdint>
#ifdef ESP32
using _millis_t = int64_t;
#else
using _millis_t = uint32_t;
#endif
#define PENDING_SEC2_ENTER() ((void)0)
#define PENDING_SEC2_EXIT() ((void)0)
enum class PendingCommandType : uint8_t { None, Door, LightOn, LightOff, LockOn, LockOff };
struct PendingCommand { _millis_t since; uint32_t generation; bool sent; };
struct PacketAction {
    int pkt;
    bool inc_counter;
    uint32_t delay;
    uint32_t pending_generation;
    PendingCommandType pending_type;
};
PendingCommand pendingLightOn = {0, 0, false}, pendingLightOff = {0, 0, false};
PendingCommand pendingLockOn = {0, 0, false}, pendingLockOff = {0, 0, false};
PendingCommand pendingDoorCommand = {0, 0, false};
uint32_t pendingSec2Generation = 0;
int queueGuardCount = 0;
struct PendingSec2QueueGuard {
    PendingSec2QueueGuard() { ++queueGuardCount; }
    ~PendingSec2QueueGuard() { --queueGuardCount; }
};
uint32_t now = 0;
_millis_t _millis() { return now; }
const char *TAG = "test";
#define ESP_LOGW(...) ((void)0)
""" + helpers + r"""
int main() {
    uint32_t first = pending_sec2_reserve(pendingLightOn, &pendingLightOff);
    PacketAction oldAction = {0, true, 0, first, PendingCommandType::LightOn};
    now = 10000;
    check_pending_sec2();
    assert(pending_sec2_active(pendingLightOn));
    assert(!pendingLightOn.sent);
    assert(pending_sec2_reserve(pendingLightOn, nullptr, true) == 0);

    // A newer same-state action replaces the pending generation; an older queued packet
    // cannot start its timeout or clear the newer request.
    uint32_t second = pending_sec2_reserve(pendingLightOn, &pendingLightOff);
    assert(second != first);
    pending_sec2_sent(oldAction);
    assert(pendingLightOn.generation == second && !pendingLightOn.sent);
    pending_sec2_dropped(oldAction);
    assert(pending_sec2_active(pendingLightOn));
    PacketAction newAction = {0, true, 0, second, PendingCommandType::LightOn};
    pending_sec2_sent(newAction);
    assert(pendingLightOn.sent && pendingLightOn.since == now);
    now += 1000; check_pending_sec2(); assert(pending_sec2_active(pendingLightOn));
    now++; check_pending_sec2(); assert(!pending_sec2_active(pendingLightOn));

    // A failed queued request only clears the generation associated with that packet.
    uint32_t third = pending_sec2_reserve(pendingLockOn, &pendingLockOff);
    PacketAction failedAction = {0, true, 0, third, PendingCommandType::LockOn};
    pending_sec2_dropped(failedAction);
    assert(!pending_sec2_active(pendingLockOn));

    // Failed admission restores an older action that is still in the queue.
    uint32_t queuedGeneration = pending_sec2_reserve(pendingLightOff, &pendingLightOn);
    PacketAction queuedAction = {0, true, 0, queuedGeneration, PendingCommandType::LightOff};
    PendingCommand previousPending, previousOpposite;
    uint32_t rejectedGeneration = pending_sec2_reserve(pendingLightOff, &pendingLightOn, false,
                                                        &previousPending, &previousOpposite);
    pending_sec2_rollback(pendingLightOff, &pendingLightOn, rejectedGeneration,
                          previousPending, &previousOpposite);
    assert(pendingLightOff.generation == queuedGeneration && !pendingLightOff.sent);
    pending_sec2_sent(queuedAction);
    assert(pendingLightOff.sent && pendingLightOff.since == now);

    // Acknowledgements clear transmitted requests but preserve queued requests.
    uint32_t sent = pending_sec2_reserve(pendingLockOn, &pendingLockOff);
    PacketAction sentAction = {0, true, 0, sent, PendingCommandType::LockOn};
    pending_sec2_sent(sentAction);
    uint32_t queued = pending_sec2_reserve(pendingLockOff, &pendingLockOn);
    pending_sec2_acknowledge_pair(pendingLockOn, pendingLockOff);
    assert(!pending_sec2_active(pendingLockOn));
    assert(pendingLockOff.generation == queued && !pendingLockOff.sent);

    // Future timestamps are not treated as elapsed time after a stale clock snapshot.
    now = 4000;
    uint32_t future = pending_sec2_reserve(pendingDoorCommand);
    PacketAction futureAction = {0, true, 0, future, PendingCommandType::Door};
    now = 3900; check_pending_sec2();
    pending_sec2_sent(futureAction);
    assert(pending_sec2_active(pendingDoorCommand));
    pending_sec2_acknowledge(pendingDoorCommand);
    assert(!pending_sec2_active(pendingDoorCommand));
    pending_sec2_clear(pendingDoorCommand);

#ifdef ESP32
    // The 64-bit clock can safely expire a command after more than half the 32-bit range.
    pendingLockOff = {1, 123, true};
    now = 0x80000002;
    check_pending_sec2(); assert(!pending_sec2_active(pendingLockOff));
#endif

#ifdef ESP8266
    // Unsigned elapsed time still handles a legitimate 32-bit millis() rollover.
    pending_sec2_clear(pendingDoorCommand);
    now = UINT32_MAX - 500;
    uint32_t wrap = pending_sec2_reserve(pendingLightOff);
    PacketAction wrapAction = {0, true, 0, wrap, PendingCommandType::LightOff};
    pending_sec2_sent(wrapAction);
    now = 499; check_pending_sec2(); assert(pending_sec2_active(pendingLightOff));
    now = 500; check_pending_sec2(); assert(!pending_sec2_active(pendingLightOff));
#endif
    assert(queueGuardCount == 0);
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
