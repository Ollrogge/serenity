/*
 * Copyright (c) 2021, Luke Wilde <lukew@serenityos.org>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#define DWC2_DEBUG 1

#include <Kernel/Bus/USB/DWC2/DWC2Controller.h>
#include <Kernel/Bus/USB/DWC2/DWC2RootHub.h>
#include <Kernel/Bus/USB/USBClasses.h>
#include <Kernel/Bus/USB/USBConstants.h>
#include <Kernel/Bus/USB/USBEndpoint.h>
#include <Kernel/Bus/USB/USBHub.h>
#include <Kernel/Bus/USB/USBRequest.h>

// FIXME: Should we share this code with UHCIRootHub? Just copied it for now.

namespace Kernel::USB {

static USBDeviceDescriptor DWC2_root_hub_device_descriptor = {
    {
        sizeof(USBDeviceDescriptor), // 18 bytes long
        DESCRIPTOR_TYPE_DEVICE,
    },
    0x0110, // USB 1.1
    (u8)USB_CLASS_HUB,
    0,      // Hubs use subclass 0
    0,      // Full Speed Hub
    64,     // Max packet size
    0x0,    // Vendor ID
    0x0,    // Product ID
    0x0110, // Product version (can be anything, currently matching usb_spec_compliance_bcd)
    0,      // Index of manufacturer string. FIXME: There is currently no support for string descriptors.
    0,      // Index of product string. FIXME: There is currently no support for string descriptors.
    0,      // Index of serial string. FIXME: There is currently no support for string descriptors.
    1,      // One configuration descriptor
};

static USBConfigurationDescriptor DWC2_root_hub_configuration_descriptor = {
    {
        sizeof(USBConfigurationDescriptor), // 9 bytes long
        DESCRIPTOR_TYPE_CONFIGURATION,
    },
    sizeof(USBConfigurationDescriptor) + sizeof(USBInterfaceDescriptor) + sizeof(USBEndpointDescriptor), // Combined length of configuration, interface and endpoint and descriptors.
    1,                                                                                                   // One interface descriptor
    1,                                                                                                   // Configuration #1
    0,                                                                                                   // Index of configuration string. FIXME: There is currently no support for string descriptors.
    (1 << 7) | (1 << 6),                                                                                 // Bit 6 is set to indicate that the root hub is self powered. Bit 7 must always be 1.
    0,                                                                                                   // 0 mA required from the bus (self-powered)
};

static USBInterfaceDescriptor DWC2_root_hub_interface_descriptor = {
    {
        sizeof(USBInterfaceDescriptor), // 9 bytes long
        DESCRIPTOR_TYPE_INTERFACE,
    },
    0, // Interface #0
    0, // Alternate setting
    1, // One endpoint
    (u8)USB_CLASS_HUB,
    0, // Hubs use subclass 0
    0, // Full Speed Hub
    0, // Index of interface string. FIXME: There is currently no support for string descriptors
};

static USBEndpointDescriptor DWC2_root_hub_endpoint_descriptor = {
    {
        sizeof(USBEndpointDescriptor), // 7 bytes long
        DESCRIPTOR_TYPE_ENDPOINT,
    },
    USBEndpoint::ENDPOINT_ADDRESS_DIRECTION_IN | 1,           // IN Endpoint #1
    USBEndpoint::ENDPOINT_ATTRIBUTES_TRANSFER_TYPE_INTERRUPT, // Interrupt endpoint
    2,                                                        // Max Packet Size FIXME: I'm not sure what this is supposed to be as it is implementation defined. 2 is the number of bytes Get Port Status returns.
    0xFF,                                                     // Max possible interval
};

// NOTE: DWC2 does not provide us anything for the Root Hub's Hub Descriptor.
static USBHubDescriptor DWC2_root_hub_hub_descriptor = {
    {
        sizeof(USBHubDescriptor), // 7 bytes long. FIXME: Add the size of the VLAs at the end once they're supported.
        DESCRIPTOR_TYPE_HUB,
    },
    DWC2Controller::NUMBER_OF_ROOT_PORTS, // 1 port
    0x0,                                  // Ganged power switching, not a compound device, global over-current protection.
    0x0,                                  // DWC2 ports are always powered, so there's no time from power on to power good.
    0x0,                                  // Self-powered
};

ErrorOr<NonnullOwnPtr<DWC2RootHub>> DWC2RootHub::try_create(NonnullLockRefPtr<DWC2Controller> DWC2_controller)
{
    return adopt_nonnull_own_or_enomem(new (nothrow) DWC2RootHub(move(DWC2_controller)));
}

DWC2RootHub::DWC2RootHub(NonnullLockRefPtr<DWC2Controller> DWC2_controller)
    : m_dwc2_controller(move(DWC2_controller))
{
}

ErrorOr<void> DWC2RootHub::setup(Badge<DWC2Controller>)
{
    m_hub = TRY(Hub::try_create_root_hub(m_dwc2_controller, Device::DeviceSpeed::FullSpeed));

    // NOTE: The root hub will be on the default address at this point.
    // The root hub must be the first device to be created, otherwise the HCD will intercept all default address transfers as though they're targeted at the root hub.
    TRY(m_hub->enumerate_device());

    dbgln("here?");

    // NOTE: The root hub is no longer on the default address.
    TRY(m_hub->enumerate_and_power_on_hub());

    return {};
}

ErrorOr<size_t> DWC2RootHub::handle_control_transfer(Transfer& transfer)
{
    auto const& request = transfer.request();
    auto* request_data = transfer.buffer().as_ptr() + sizeof(USBRequestData);

    if constexpr (DWC2_DEBUG) {
        dbgln("DWC2RootHub: Received control transfer.");
        dbgln("DWC2RootHub: Request Type: 0x{:02x}", request.request_type);
        dbgln("DWC2RootHub: Request: 0x{:02x}", request.request);
        dbgln("DWC2RootHub: Value: 0x{:04x}", request.value);
        dbgln("DWC2RootHub: Index: 0x{:04x}", request.index);
        dbgln("DWC2RootHub: Length: 0x{:04x}", request.length);
    }

    size_t length = 0;

    switch (request.request) {
    case HubRequest::GET_STATUS: {
        if (request.index > DWC2Controller::NUMBER_OF_ROOT_PORTS)
            return EINVAL;

        length = min(transfer.transfer_data_size(), sizeof(HubStatus));
        VERIFY(length <= sizeof(HubStatus));
        HubStatus hub_status {};

        if (request.index == 0) {
            // If index == 0, the actual request is Get Hub Status
            // DWC2 does not provide "Local Power Source" or "Over-current" and their corresponding change flags.
            // The members of hub_status are initialized to 0, so we can memcpy it straight away.
            memcpy(request_data, (void*)&hub_status, length);
            break;
        }

        // If index != 0, the actual request is Get Port Status
        m_dwc2_controller->get_port_status({}, request.index - 1, hub_status);
        memcpy(request_data, (void*)&hub_status, length);
        break;
    }
    case HubRequest::GET_DESCRIPTOR: {
        u8 descriptor_type = request.value >> 8;
        switch (descriptor_type) {
        case DESCRIPTOR_TYPE_DEVICE:
            length = min(transfer.transfer_data_size(), sizeof(USBDeviceDescriptor));
            VERIFY(length <= sizeof(USBDeviceDescriptor));
            memcpy(request_data, (void*)&DWC2_root_hub_device_descriptor, length);
            break;
        case DESCRIPTOR_TYPE_CONFIGURATION: {
            auto index = 0u;

            // Send over the whole descriptor chain
            length = DWC2_root_hub_configuration_descriptor.total_length;
            VERIFY(length <= sizeof(USBConfigurationDescriptor) + sizeof(USBInterfaceDescriptor) + sizeof(USBEndpointDescriptor));
            memcpy(request_data, (void*)&DWC2_root_hub_configuration_descriptor, sizeof(USBConfigurationDescriptor));
            index += sizeof(DWC2_root_hub_configuration_descriptor);
            memcpy(request_data + index, (void*)&DWC2_root_hub_interface_descriptor, sizeof(USBInterfaceDescriptor));
            index += sizeof(DWC2_root_hub_interface_descriptor);
            memcpy(request_data + index, (void*)&DWC2_root_hub_endpoint_descriptor, sizeof(USBEndpointDescriptor));
            break;
        }
        case DESCRIPTOR_TYPE_INTERFACE:
            length = min(transfer.transfer_data_size(), sizeof(USBInterfaceDescriptor));
            VERIFY(length <= sizeof(USBInterfaceDescriptor));
            memcpy(request_data, (void*)&DWC2_root_hub_interface_descriptor, length);
            break;
        case DESCRIPTOR_TYPE_ENDPOINT:
            length = min(transfer.transfer_data_size(), sizeof(USBEndpointDescriptor));
            VERIFY(length <= sizeof(USBEndpointDescriptor));
            memcpy(request_data, (void*)&DWC2_root_hub_endpoint_descriptor, length);
            break;
        case DESCRIPTOR_TYPE_HUB:
            length = min(transfer.transfer_data_size(), sizeof(USBHubDescriptor));
            VERIFY(length <= sizeof(USBHubDescriptor));
            memcpy(request_data, (void*)&DWC2_root_hub_hub_descriptor, length);
            break;
        default:
            return EINVAL;
        }
        break;
    }
    case USB_REQUEST_SET_ADDRESS:
        dbgln_if(DWC2_DEBUG, "DWC2RootHub: Attempt to set address to {}, ignoring.", request.value);
        if (request.value > USB_MAX_ADDRESS)
            return EINVAL;
        // Ignore SET_ADDRESS requests. USBDevice sets its internal address to the new allocated address that it just sent to us.
        // The internal address is used to check if the request is directed at the root hub or not.
        break;
    case HubRequest::SET_FEATURE: {
        if (request.index == 0) {
            // If index == 0, the actual request is Set Hub Feature.
            // DWC2 does not provide "Local Power Source" or "Over-current" and their corresponding change flags.
            // Therefore, ignore the request, but return an error if the value is not "Local Power Source" or "Over-current"
            switch (request.value) {
            case HubFeatureSelector::C_HUB_LOCAL_POWER:
            case HubFeatureSelector::C_HUB_OVER_CURRENT:
                break;
            default:
                return EINVAL;
            }

            break;
        }

        // If index != 0, the actual request is Set Port Feature.
        u8 port = request.index & 0xFF;
        if (port > DWC2Controller::NUMBER_OF_ROOT_PORTS)
            return EINVAL;

        auto feature_selector = (HubFeatureSelector)request.value;
        TRY(m_dwc2_controller->set_port_feature({}, port - 1, feature_selector));
        break;
    }
    case HubRequest::CLEAR_FEATURE: {
        if (request.index == 0) {
            // If index == 0, the actual request is Clear Hub Feature.
            // DWC2 does not provide "Local Power Source" or "Over-current" and their corresponding change flags.
            // Therefore, ignore the request, but return an error if the value is not "Local Power Source" or "Over-current"
            switch (request.value) {
            case HubFeatureSelector::C_HUB_LOCAL_POWER:
            case HubFeatureSelector::C_HUB_OVER_CURRENT:
                break;
            default:
                return EINVAL;
            }

            break;
        }

        // If index != 0, the actual request is Clear Port Feature.
        u8 port = request.index & 0xFF;
        if (port > DWC2Controller::NUMBER_OF_ROOT_PORTS)
            return EINVAL;

        auto feature_selector = (HubFeatureSelector)request.value;
        TRY(m_dwc2_controller->clear_port_feature({}, port - 1, feature_selector));
        break;
    }
    default:
        return EINVAL;
    }

    transfer.set_complete();
    return length;
}

}
