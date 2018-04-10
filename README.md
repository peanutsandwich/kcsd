# kcsd
KCS Daemon for OpenBMC that bridges KCS requests to the DBUS interface for IPMI.

This daemon is intended for use in the OpenBMC project at https://github.com/openbmc/openbmc.

## Dependencies
It is reliant on the patch series from Haiyue Wang that can be found at the following URLs. [1] is the driver itself, and [2] is the devicetree change to enable it on the g5 aspeed platform:

1. https://patchwork.kernel.org/patch/10196203/
2. https://patchwork.kernel.org/patch/10257559/

## Limitations
Note that this is not designed to be a replacement to the block transfer daemon (btbridged) implemented at https://github.com/openbmc/btbridge, but rather supplements it for platforms that prefer the KCS interface.

## Inclusion in OpenBmc
This module can be included in openbmc using the bb file that was added in the following commit: https://github.com/peanutsandwich/openbmc/tree/9d7bf955938e99b700d2e4eef11691a601da2642.

This bb file installes a udev rules file that will start an instance of kcsd for each ipmi KCS interface that has been enabled in the device tree.
