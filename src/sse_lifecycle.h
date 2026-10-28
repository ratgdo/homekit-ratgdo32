#pragma once

#include <atomic>
#include <cstdint>

// Claim removal before cleanup or diagnostics, either of which may log and
// synchronously reenter the broadcaster. Only the winning caller owns cleanup.
template <typename Cleanup, typename Notify>
bool retireSSESubscription(std::atomic<bool> &active,
                           std::atomic<uint32_t> &count,
                           Cleanup cleanup, Notify notify)
{
    if (!active.exchange(false))
        return false;
    const uint32_t remaining = count.fetch_sub(1) - 1;
    cleanup();
    notify(remaining);
    return true;
}
