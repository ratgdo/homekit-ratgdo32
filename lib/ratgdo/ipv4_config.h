#pragma once
#include <cstdint>

// HomeSpan's connection callback runs on the first acquired address, which can
// be IPv6. Do not replace saved IPv4 settings with zeros while DHCP is pending.
template <class Config, class Address>
bool update_ipv4_config(Config &config, const Address &address,
                        const Address &mask, const Address &gateway,
                        const Address &dns, bool dnsIsIPv4)
{
    if (static_cast<uint32_t>(address) == 0)
        return false;
    config.set("localIP", address.toString().c_str());
    config.set("subnetMask", mask.toString().c_str());
    config.set("gatewayIP", gateway.toString().c_str());
    if (dnsIsIPv4 && static_cast<uint32_t>(dns) != 0)
        config.set("nameserverIP", dns.toString().c_str());
    return true;
}
