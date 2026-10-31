from pathlib import Path
import subprocess
import tempfile
import unittest

root = Path(__file__).resolve().parents[1]
source = (root/'src/web.cpp').read_text()
start = source.index('bool clientWrite(WiFiClient client, const char *data)')
end = source.index('// Helper functions for connection throttling', start)
production = source[start:end]
fixture = r'''
#include <cassert>
#include <cerrno>
#include <cstring>
#include <cstdlib>
#include <new>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
static unsigned allocations;
void *operator new(size_t size) { ++allocations; if (void *p = malloc(size)) return p; throw std::bad_alloc(); }
void operator delete(void *p) noexcept { free(p); }
void operator delete(void *p, std::size_t) noexcept { free(p); }
static char captured[20000];
static size_t capturedLength;
static unsigned calls, failAt, flushes, closed;
static long failure;
static bool useRealSocket;
static ssize_t testSend(int fd, const void *data, size_t length, int flags) {
#ifdef ESP32
    assert(flags == MSG_DONTWAIT);
#else
    (void)flags;
#endif
    ++calls;
    if (useRealSocket) return ::send(fd, data, length, MSG_DONTWAIT);
    if (calls == failAt) return failure;
    assert(capturedLength + length < sizeof(captured));
    memcpy(captured + capturedLength, data, length);
    capturedLength += length;
    captured[capturedLength] = 0;
    return length;
}
#ifdef ESP32
static ssize_t testSendmsg(int fd, const struct msghdr *message, int flags) {
    assert(flags == MSG_DONTWAIT);
    assert(message->msg_iovlen == 5);
    ++calls;
    if (useRealSocket) return ::sendmsg(fd, message, flags);
    if (calls == failAt) return failure;
    size_t total = 0;
    for (size_t i = 0; i < static_cast<size_t>(message->msg_iovlen); ++i) {
        const struct iovec &part = message->msg_iov[i];
        assert(capturedLength + part.iov_len < sizeof(captured));
        memcpy(captured + capturedLength, part.iov_base, part.iov_len);
        capturedLength += part.iov_len;
        total += part.iov_len;
    }
    captured[capturedLength] = 0;
    return total;
}
#endif
struct WiFiClient {
    int socket = 0;
    int fd() { return socket; }
    void stop() { ++closed; }
    void flush() { ++flushes; }
    size_t write(const char *data, size_t length) { return testSend(socket, data, length, 0); }
};
#define send testSend
#define sendmsg testSendmsg
// PRODUCTION
#undef send
#undef sendmsg
static void reset() { capturedLength = calls = failAt = flushes = closed = 0; }
int main() {
    WiFiClient client;
    const unsigned before = allocations;
    for (size_t length : {0U, 2U, 8192U}) {
        char payload[8193]; memset(payload, 'x', length); payload[length] = 0;
        reset();
        assert(sendSSEEvent(client, "message", payload));
#ifdef ESP32
        assert(calls == 1);
#else
        assert(calls == (length ? 5U : 4U));
#endif
        assert(closed == 0);
        assert(!strncmp(captured, "event: message\ndata: ", 21));
        assert(capturedLength == 23 + length);
        assert(!memcmp(captured + 21, payload, length));
        assert(!strcmp(captured + 21 + length, "\n\n"));
#ifdef ESP8266
        assert(flushes == 1);
#else
        assert(flushes == 0);
#endif
    }
#ifdef ESP32
    for (long result = -1; result < 25; ++result) {
        reset(); failAt = 1; failure = result;
        assert(!sendSSEEvent(client, "message", "{}"));
        assert(calls == 1 && closed == 1);
    }
#else
    for (unsigned part = 1; part <= 5; ++part) {
        for (long result : {-1L, 0L, 1L}) {
            reset(); failAt = part; failure = result;
            assert(!sendSSEEvent(client, "message", "{}"));
            assert(calls == part && closed == 1);
        }
    }
#endif
    reset();
    assert(clientWrite(client, ""));
    assert(calls == 0 && closed == 0);
    int sockets[2]; assert(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) == 0);
    assert(fcntl(sockets[0], F_SETFL, O_NONBLOCK) == 0);
    char fill[4096] = {};
    while (::send(sockets[0], fill, sizeof(fill), MSG_DONTWAIT) > 0) {}
    assert(errno == EAGAIN || errno == EWOULDBLOCK);
    reset(); useRealSocket = true; client.socket = sockets[0];
    assert(!sendSSEEvent(client, "message", "{}"));
    assert(calls == 1 && closed == 1);
    close(sockets[0]); close(sockets[1]);
    assert(allocations == before);
}
'''
fixture = '#include <initializer_list>\n' + fixture.replace('// PRODUCTION', production)


class SSETransport(unittest.TestCase):
    def test_actual_allocation_free_transport(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            cpp = directory/'sse.cpp'
            cpp.write_text(fixture)
            for platform in ['ESP32', 'ESP8266']:
                binary = directory/platform
                subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                                '-fsanitize=address,undefined', '-D'+platform,
                                str(cpp), '-o', str(binary)], check=True)
                subprocess.run([str(binary)], check=True)
                print(platform+': actual source passed empty/large frames, each short/error span, full socket, flush, and zero heap allocation checks')


if __name__ == '__main__':
    unittest.main()
