/*
 * Copyright (c) 2012 INRIA, 2012 University of Washington
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 */

#ifndef TAP_FD_NET_DEVICE_HELPER_H
#define TAP_FD_NET_DEVICE_HELPER_H

#include "emu-fd-net-device-helper.h"

#include "ns3/attribute.h"
#include "ns3/fd-net-device.h"
#include "ns3/mac48-address.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/object-factory.h"

#include <string>

namespace ns3
{

/**
 * \ingroup fd-net-device
 * \brief build a set of FdNetDevice objects attached to a virtual TAP network
 * interface
 *
 */
class TapFdNetDeviceHelper : public EmuFdNetDeviceHelper
{
  public:
    /**
     * Construct a TapFdNetDeviceHelper.
     */
    TapFdNetDeviceHelper();

    ~TapFdNetDeviceHelper() override
    {
    }

    /**
     * Set flag IFF_NO_PI on the device.
     *
     * \param pi Set the IFF_NO_PI flag if pi is false.
     */
    void SetModePi(bool pi);

    /**
     * Set the device IPv4 address.
     *
     * \param address The IPv4 address for the TAP device.
     */
    void SetTapIpv4Address(Ipv4Address address);

    /**
     * Set the IPv4 network mask for the TAP device.
     *
     * \param mask The IPv4 network mask for the TAP device.
     */
    void SetTapIpv4Mask(Ipv4Mask mask);

    /**
     * Set the device IPv6 address.
     *
     * \param address The IPv6 address for the TAP device.
     */
    void SetTapIpv6Address(Ipv6Address address);

    /**
     * Set the IPv6 network mask for the TAP device.
     *
     * \param prefix The IPv6 network prefix for the TAP device.
     */
    void SetTapIpv6Prefix(int prefix);

    /**
     * Set the MAC address for the TAP device.
     *
     * \param mac The MAC address the TAP device.
     */
    void SetTapMacAddress(Mac48Address mac);

  protected:
    /**
     * This method creates an ns3::FdNetDevice attached to a virtual TAP network
     * interface
     *
     * \param node The node to install the device in
     * \returns A container holding the added net device.
     */
    Ptr<NetDevice> InstallPriv(Ptr<Node> node) const override;

    /**
     * Sets a file descriptor on the FileDescriptorNetDevice.
     * \param device the device to install the file descriptor in
     */
    void SetFileDescriptor(Ptr<FdNetDevice> device) const override;

    /**
     * Call out to a separate process running as suid root in order to create a
     * TAP device and obtain the file descriptor associated to it.
     * \returns The file descriptor associated with the TAP device.
     */
    int CreateFileDescriptor() const override;

    /**
     * The TAP device flag IFF_NO_PI.
     */
    bool m_modePi;

    /**
     * The IPv4 address for the TAP device.
     */
    Ipv4Address m_tapIp4;

    /**
     * The IPv6 address for the TAP device.
     */
    Ipv6Address m_tapIp6;

    /**
     * The network mask IPv4 for the TAP device.
     */
    Ipv4Mask m_tapMask4;

    /**
     * The network prefix IPv6 for the TAP device.
     */
    int m_tapPrefix6;

    /**
     * The TAP device MAC address.
     */
    Mac48Address m_tapMac;
};

} // namespace ns3

#endif /* TAP_FD_NET_DEVICE_HELPER_H */
