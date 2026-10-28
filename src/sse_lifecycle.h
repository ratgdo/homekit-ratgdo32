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

// Ticker callbacks never touch clients or wait for the lifecycle lock.
inline void requestSSEHeartbeat(std::atomic<bool> &pending) { pending.store(true); }
inline bool takeSSEHeartbeat(std::atomic<bool> &pending) { return pending.exchange(false); }

// The caller holds the lifecycle lock while checking and assigning the client.
template <typename UUID, typename Client>
bool bindSSEClient(bool active, const UUID &registeredUUID,
                   const UUID &requestedUUID, Client &current, const Client &incoming)
{
    if (!active || registeredUUID != requestedUUID)
        return false;
    current = incoming;
    return true;
}
