// SOURCE_LIFECYCLE_HELPERS

#include <cassert>
#include <functional>
#include <string>
#include <thread>
#include <vector>

int main()
{
    // A stale EventSource request must not replace a reused slot's client.
    std::string uuid = "new-uuid", current = "live-client";
    assert(!bindSSEClient(true, uuid, std::string("old-uuid"), current, std::string("stale-client")));
    assert(current == "live-client");
    assert(!bindSSEClient(false, uuid, uuid, current, std::string("inactive-client")));
    assert(current == "live-client");
    assert(bindSSEClient(true, uuid, uuid, current, std::string("replacement-client")));
    assert(current == "replacement-client");

    // A heartbeat disconnect logs; the logger broadcasts synchronously and
    // discovers the same disconnected slot. Another client must remain counted.
    std::atomic<bool> disconnected(true), survivor(true);
    std::atomic<uint32_t> count(2);
    unsigned cleaned = 0, notified = 0;
    bool connected = true;
    std::function<void()> remove;
    remove = [&] {
        retireSSESubscription(disconnected, count,
            [&] {
                connected = false;
                ++cleaned;
                remove(); // Even cleanup can trigger a logging callback.
            },
            [&](uint32_t remaining) {
                assert(!connected);
                assert(remaining == 1);
                ++notified;
                remove(); // Diagnostic -> logToBuffer -> SSEBroadcastState.
            });
    };
    remove();
    remove(); // Outer heartbeat resumes after nested broadcast removal.
    assert(count == 1 && survivor && cleaned == 1 && notified == 1);

    // Simultaneous removals must also have one winner.
    disconnected = true;
    count = 2;
    std::atomic<unsigned> concurrentCleanup(0), concurrentNotify(0);
    std::vector<std::thread> removers;
    for (unsigned i = 0; i < 8; ++i)
        removers.emplace_back([&] {
            retireSSESubscription(disconnected, count,
                [&] { ++concurrentCleanup; },
                [&](uint32_t remaining) { assert(remaining == 1); ++concurrentNotify; });
        });
    for (auto &thread : removers)
        thread.join();
    assert(count == 1 && survivor && concurrentCleanup == 1 && concurrentNotify == 1);
}
