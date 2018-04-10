# kcsd
KCS Daemon for OpenBMC that bridges KCS requests to the DBUS interface for IPMI

This daemon is intended for use in the OpenBMC project at https://github.com/openbmc/openbmc.

## Dependencies
It is reliant on the patch series from Haiyue Wang that can be found at the following URLs. [1] is the driver itself, and [2] is the devicetree change to enable it on the g5 aspeed platform:

1. https://patchwork.kernel.org/patch/10196203/
2. https://patchwork.kernel.org/patch/10257559/
