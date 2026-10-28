#include "../lib/ratgdo/ipv4_config.h"
#include <cassert>
#include <map>
#include <string>

struct Address
{
    std::string value;
    explicit operator uint32_t() const { return value == "0.0.0.0" ? 0 : 1; }
    std::string toString() const { return value; }
};

struct Config
{
    std::map<std::string, std::string> values;
    void set(const char *key, const char *value) { values[key] = value; }
};

int main()
{
    Config config;
    const Address none{"0.0.0.0"}, ip{"192.168.23.32"}, mask{"255.255.255.0"};
    const Address gateway{"192.168.23.1"}, ipv6dns{"fe80::1"};
    assert(update_ipv4_config(config, ip, mask, gateway, gateway, true));
    auto saved = config.values;

    // A new connection acquires IPv6 before IPv4. Retain the previous IPv4
    // settings until the separate IPv4 event supplies a valid address.
    assert(!update_ipv4_config(config, none, none, none, ipv6dns, false));
    assert(config.values == saved);
    const Address renewed{"192.168.23.33"};
    assert(update_ipv4_config(config, renewed, mask, gateway, ipv6dns, false));
    assert(config.values.at("localIP") == renewed.value);
    assert(config.values.at("subnetMask") == mask.value);
    assert(config.values.at("gatewayIP") == gateway.value);
    assert(config.values.at("nameserverIP") == gateway.value);
    assert(update_ipv4_config(config, ip, mask, gateway, none, true));
    assert(config.values.at("nameserverIP") == gateway.value);

    // A first boot with no saved network settings must not publish zero IPv4.
    Config fresh;
    assert(!update_ipv4_config(fresh, none, none, none, none, true));
    assert(fresh.values.empty());
    assert(update_ipv4_config(fresh, ip, mask, gateway, gateway, true));
    assert(fresh.values == saved);
}
