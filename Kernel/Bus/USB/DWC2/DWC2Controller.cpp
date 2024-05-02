/*
 * Copyright (c) 2023, Timon Kruiper <timonkruiper@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#define DWC2_DEBUG 1

#include <Kernel/Arch/aarch64/ASM_wrapper.h>

#include <Kernel/Arch/Delay.h>
#include <Kernel/Bus/USB/DWC2/DWC2Controller.h>
#include <Kernel/Bus/USB/USBManagement.h>
#include <Kernel/Bus/USB/USBRequest.h>
#include <Kernel/Memory/TypedMapping.h>
#include <Kernel/Tasks/Process.h>

#if ARCH(AARCH64)
#    include <Kernel/Arch/aarch64/RPi/MMIO.h>
#endif

namespace Kernel::USB {

// Useful resources:
// - https://web.archive.org/web/20211125211715/http://www.capital-micro.com/PDF/CME-M7_Family_User_Guide_EN.pdf (Chapter 18, page 370)
// - https://static6.arrow.com/aropdfconversion/aa9a14376a75e7c5d6daa9f6aaed8411909d2021/rt3050_5x_v2.0_081408_0902.pdf (Chapter 3.19, page 130)
// - https://gitlab.com/qemu-project/qemu/-/blob/master/hw/usb/hcd-dwc2.c

// 18.5.2.2 Global CSR Map
struct CoreGlobalRegisters {
    u32 GOTGCTL;   // Control and Status Register
    u32 GOTGINT;   // Interrupt Register
    u32 GAHBCFG;   // AHB Configuration Register
    u32 GUSBCFG;   // USB Configuration Register
    u32 GRSTCTL;   // Reset Register
    u32 GINTSTS;   // Interrupt Register
    u32 GINTMSK;   // Interrupt Mask Register
    u32 GRXSTSR;   // Receive Status Debug Read / Status Read and Pop Registers
    u32 GRXSTSP;   // Receive Status Debug Read / Status Read and Pop Registers
    u32 GRXFSIZ;   // Receive FIFO Size Register
    u32 GNPTXFSIZ; // Non-Periodic Transmit FIFO Size Register
    u32 GNPTXSTS;  // Non-Periodic Transmit FIFO / Queue Status Register
    u32 GI2CCTL;   // I2C Access Register
    u32 GPVNDCTL;  // PHY Vendor Control Register
    u32 GGPIO;     // General Purpose Input/Output Register
    u32 GUID;      // User ID Register
    union {
        struct {
            u32 revision_minor : 12;
            u32 revision_major : 4;
            u32 id : 16;
        };
        u32 raw;
    } GSNPSID;     // Synopsys ID Register
    u32 GHWCFG1;   // User HW Config1 Register
    u32 GHWCFG2;   // User HW Config2 Register
    u32 GHWCFG3;   // User HW Config3 Register
    u32 GHWCFG4;   // User HW Config4 Register
    u32 GLPMCFG;   // Core LPM Configuration Register
    u32 GPWRDN;    // Power Down Register
    u32 GDFIFOCFG; // DFIFO Software Config Register
    u32 GADPCTL;   // ADP Timer, Control and Status Register
    u32 _reserved0[0x27];
    u32 HPTXFSIZ;     // Host Periodic Transmit FIFO Size Register
    u32 DPTXFSIZ[16]; // Array of Device Periodic Transmit FIFO-n Size / Device IN Endpoint Transmit FIFO Size Register (DIEPTXFn)
    u32 _reserved1[0xaf];
};
static_assert(AssertSize<CoreGlobalRegisters, 1024>()); // 1KiB

// Part of 18.5.2.3 Host Mode CSR Map
struct HostChannelRegisters {
    union {
        struct {
            u32 maximum_packet_size : 11;
            u32 endpoint_number : 4;
            u32 endpoint_direction : 1;
            u32 _reserved0 : 1;
            u32 low_speed_device : 1;
            u32 endpoint_type : 2;
            u32 multi_count : 2;
            u32 device_address : 7;
            u32 odd_frame : 1;
            u32 channel_disable : 1;
            u32 channel_enable : 1;
        };
        u32 raw;
    } HCCHAR;     // Host Channel-n Characteristics Register
    u32 HCSPLT;   // Host Channel-n Split Control Register
    u32 HCINT;    // Host Channel-n Interrupt Register
    u32 HCINTMSK; // Host Channel-n Interrupt Mask Register
    union {
        struct {
            u32 transfer_size : 19;
            u32 packet_count : 10;
            u32 pid : 2;
            u32 do_ping : 1;
        };
        u32 raw;
    } HCTSIZ;  // Host Channel-n Transfer Size Register
    u32 HCDMA; // Host Channel-n DMA Address Register
    u32 _reserved;
    u32 HCDMAB; // Host Channel-n DMA Buffer Address Register
};
static_assert(AssertSize<HostChannelRegisters, 0x20>());

// 18.5.2.3 Host Mode CSR Map
struct HostModeRegisters {
    u32 HCFG;  // Host Configuration Register
    u32 HFIR;  // Host Frame Interval Register
    u32 HFNUM; // Host Frame Number/Frame Time Remaining Register
    u32 _reserved0;
    u32 HPTXSTS;  // Host Periodic Transmit FIFO/Queue Status Register
    u32 HAINT;    // Host All Channels Interrupt Register
    u32 HAINTMSK; // Host All Channels Interrupt Mask Register
    u32 HFLBAddr; // Host Frame List Base Address Register
    u32 _reserved1[0x8];
    union {
        struct {
            u32 port_connect_status : 1;
            u32 port_connect_detected : 1;
            u32 port_enable : 1;
            u32 port_enable_change : 1;
            u32 port_overcurrent_active : 1;
            u32 port_overcurrent_change : 1;
            u32 port_resume : 1;
            u32 port_suspend : 1;
            u32 port_reset : 1;
            u32 _reserved0 : 1;
            u32 port_line_status : 2;
            u32 port_power : 1;
            u32 port_test_control : 4;
            u32 port_speed : 2;
            u32 _reserved1 : 13;
        };
        u32 raw;
    } HPRT; // Host Port Control and Status Register
    u32 _reserved2[0x2f];
    HostChannelRegisters HC[16]; // Array of Host Channel Registers
    u32 _reserved3[0x40];
};
static_assert(AssertSize<HostModeRegisters, 1024>()); // 1KiB

// 18.5.2.4 Device Mode CSR Map
// TODO: Actually make this one correct, add arrays and correct reserved ranges etc.
struct DeviceModeRegisters {
    u32 DCFG; // Device Configuration Register
    u32 DCTL; // Device Control Register
    u32 DSTS; // Device Status Register
    u32 _reserved0;
    u32 DIEPMSK;           // Device IN Endpoint Common Interrupt Mask Register
    u32 DOEPMSK;           // Device OUT Endpoint Common Interrupt Mask Register
    u32 DAINT;             // Device All Endpoints Interrupt Register
    u32 DAINTMSK;          // Device All Endpoints Interrupt Mask Register
    u32 DTKNQR1;           // Device IN Token Sequence Learning Queue Read Register 1
    u32 DTKNQR2;           // Device IN Token Sequence Learning Queue Read Register 2
    u32 DTKNQR3;           // Device IN Token Sequence Learning Queue Read Register 3
    u32 DTKNQR4;           // Device IN Token Sequence Learning Queue Read Register 4
    u32 DVBUSDIS;          // Device VBUS Discharge Time Register
    u32 DVBUSPULSE;        // Device VBUS Pulsing Time Register
    u32 DTHRCTL;           // Device Threshold Control Register
    u32 DIEPEMPMSK;        // Device IN Endpoint FIFO Empty Interrupt Mask Register
    u32 DEACHINT;          // Device Each Endpoint Interrupt Register
    u32 DEACHINTMSK;       // Device Each Endpoint Interrupt Register Mask
    u32 DIEPEACHMSKn;      // Device Each In Endpoint-n Interrupt Register
    u32 DOEPEACHMSKn;      // Device Each Out Endpoint-n Interrupt Register
    u32 DIEPCTL0;          // Device Control IN Endpoint 0 Control Register
    u32 _reserved1;        //
    u32 DIEPCTLn;          // Device Endpoint-n Control Register
    u32 DIEPINTn;          // Device Endpoint-n Interrupt Register
    u32 _reserved2;        //
    u32 DIEPTSIZ0;         // Device Endpoint 0 Transfer Size Register
    u32 DIEPTSIZn;         // Device Endpoint-n Transfer Size Register
    u32 DIEPDMAn;          // Device Endpoint-n DMA Address Register
    u32 DTXFSTSn;          // Device IN Endpoint Transmit FIFO Status Register (/ u32 DOEPDMABn; // Device Endpoint-n DMA Buffer Address Register)
    u32 DOEPCTL0;          // Device Control OUT Endpoint 0 Control Register
    u32 _reserved3;        //
    u32 DOEPCTLn;          // Device Endpoint-n Control Register
    u32 DOEPINTn;          // Device Endpoint-n Interrupt Register
    u32 _reserved4;        //
    u32 DOEPTSIZ0;         // Device Endpoint 0 Transfer Size Register
    u32 DOEPTSIZn;         // Device Endpoint-n Transfer Size Register
    u32 DOEPDMAn;          // Device Endpoint-n DMA Address Register
    u32 DOEPDMABn;         // Device Endpoint-n DMA Buffer Address Register
    u32 _reserved5[0x15a]; // TODO: This is not correct!
};
// TODO: Verify the DeviceModeRegisters layout!
static_assert(AssertSize<DeviceModeRegisters, 1536>()); // 1.5KiB

// 18.5.2.6 Power and Clock Gating CSR Map
struct PowerAndClockGatingRegisters {
    u32 PCGCCTL; // Power and Clock Gating Control Register
    u32 _reserved0[0x7f];
};
static_assert(AssertSize<PowerAndClockGatingRegisters, 512>()); // 0.5KiB

struct ControlAndStatusRegisters {
    CoreGlobalRegisters core_global_regs;                     // @ 0x0000
    HostModeRegisters host_mode_regs;                         // @ 0x0400
    DeviceModeRegisters device_mode_regs;                     // @ 0x0800
    PowerAndClockGatingRegisters power_and_clock_gating_regs; // @ 0x0E00
};
static_assert(AssertSize<ControlAndStatusRegisters, 4096>());

DWC2Controller::DWC2Controller(ControlAndStatusRegisters volatile* csr_regs)
    : m_csr_regs(csr_regs)
{
}

ErrorOr<NonnullLockRefPtr<DWC2Controller>> DWC2Controller::try_to_initialize()
{
#if ARCH(X86_64)
    return EIO;
#elif ARCH(AARCH64)
    // FIXME: Use TypedMapping, but that currently doesn't work for aarch64.
    auto* csr_regs = RPi::MMIO::the().peripheral<ControlAndStatusRegisters>(0x98'0000);
    auto controller = TRY(adopt_nonnull_lock_ref_or_enomem(new (nothrow) DWC2Controller(csr_regs)));
    TRY(controller->initialize());
    return controller;
#else
#    error "Unknown architecture"
#endif
}

ErrorOr<void> DWC2Controller::initialize()
{
    // TODO: Sent Power-up mailbox message to actually power the USB IF, without this it will not work on real hardware!

    if (m_csr_regs->core_global_regs.GSNPSID.id != 0x4f54) {
        dbgln("DWC2Controller: Unknown DWC Core OTG: 0x{:x}", (u32)m_csr_regs->core_global_regs.GSNPSID.raw);
        return ENOTSUP;
    }
    auto revision_major = m_csr_regs->core_global_regs.GSNPSID.revision_major;
    auto revision_minor = m_csr_regs->core_global_regs.GSNPSID.revision_minor;
    dbgln("DWC2Controller: Revision: {:x}.{:x}", revision_major, revision_minor);

    if (revision_major != 2 or revision_minor != 0x94a)
        dbgln("DWC2Controller: Running with unsupported revision, the driver might not work!");

    dbgln("DWC2Controller: Receive FIFO Size: {}", (u32)m_csr_regs->core_global_regs.GRXFSIZ);

    // dwc2_reset();

    TRY(spawn_async_poll_process());
    TRY(spawn_port_process());

    m_root_hub = TRY(DWC2RootHub::try_create(*this));
    TRY(m_root_hub->setup({}));

    return {};
}

ErrorOr<void> DWC2Controller::spawn_port_process()
{
    TRY(Process::create_kernel_process("DWC2 Hot Plug Task"sv, [&] {
        while (!Process::current().is_dying()) {
            if (m_root_hub && USBManagement::the().drivers_initialized())
                m_root_hub->check_for_port_updates();

            (void)Thread::current()->sleep(Duration::from_seconds(1));
        }
        Process::current().sys$exit(0);
        VERIFY_NOT_REACHED();
    }));
    return {};
}

ErrorOr<void> DWC2Controller::spawn_async_poll_process()
{
    // TRY(Process::create_kernel_process("DWC2 Async Poll Task"sv, [&] {
    //     u16 poll_interval_ms = 1024;
    //     while (!Process::current().is_dying()) {
    //         {
    //             SpinlockLocker locker { m_async_lock };
    //             for (OwnPtr<AsyncTransferHandle>& handle : m_active_async_transfers) {
    //                 if (handle != nullptr) {
    //                     poll_interval_ms = min(poll_interval_ms, handle->ms_poll_interval);
    //                     QueueHead* qh = handle->qh;
    //                     for (auto td = qh->get_first_td(); td != nullptr && !td->active(); td = td->next_td()) {
    //                         if (td->next_td() == nullptr) { // Finished QH
    //                             handle->transfer->invoke_async_callback();
    //                             qh->reinitialize(); // Set the QH to be active again
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //         (void)Thread::current()->sleep(Duration::from_milliseconds(poll_interval_ms));
    //     }
    //     Process::current().sys$exit(0);
    //     VERIFY_NOT_REACHED();
    // }));
    return {};
}

void DWC2Controller::dwc2_reset()
{
}

ErrorOr<void> DWC2Controller::reset()
{
    return {};
}
ErrorOr<void> DWC2Controller::stop()
{
    return {};
}
ErrorOr<void> DWC2Controller::start()
{
    return {};
}

void DWC2Controller::cancel_async_transfer(NonnullLockRefPtr<Transfer>)
{
    dbgln("TODO: cancel_async_transfer");
}

ErrorOr<size_t> DWC2Controller::submit_control_transfer(Transfer& transfer)
{
    // Control transfer has 3 stages:
    // - setup
    // - data
    // - status
    // https://www.beyondlogic.org/usbnutshell/usb4.shtml

    Pipe& pipe = transfer.pipe(); // Short circuit the pipe related to this transfer
    bool direction_in = (transfer.request().request_type & USB_REQUEST_TRANSFER_DIRECTION_DEVICE_TO_HOST) == USB_REQUEST_TRANSFER_DIRECTION_DEVICE_TO_HOST;

    dbgln_if(DWC2_DEBUG, "DWC2: Received control transfer for address {}. Root Hub is at address {}. Pipe type is {}", pipe.device_address(), m_root_hub->device_address(), to_underlying(pipe.type()));

    // Short-circuit the root hub.
    if (pipe.device_address() == m_root_hub->device_address())
        return m_root_hub->handle_control_transfer(transfer);

    int channel = 0; // We actually have 16 channels available

    m_csr_regs->host_mode_regs.HC[channel].HCCHAR.raw = 0;

    m_csr_regs->host_mode_regs.HC[channel].HCCHAR.device_address = pipe.device_address();
    m_csr_regs->host_mode_regs.HC[channel].HCCHAR.endpoint_number = pipe.endpoint_address();
    m_csr_regs->host_mode_regs.HC[channel].HCCHAR.endpoint_direction = 1; // SETUP is always out! However that doesn't matter, also not on real hardware?
    m_csr_regs->host_mode_regs.HC[channel].HCCHAR.low_speed_device = pipe.device_speed() == Pipe::DeviceSpeed::LowSpeed ? 1 : 0;
    m_csr_regs->host_mode_regs.HC[channel].HCCHAR.endpoint_type = to_underlying(pipe.type());
    m_csr_regs->host_mode_regs.HC[channel].HCCHAR.maximum_packet_size = pipe.max_packet_size();
    m_csr_regs->host_mode_regs.HC[channel].HCCHAR.channel_enable = 0;
    m_csr_regs->host_mode_regs.HC[channel].HCCHAR.channel_disable = 0;

    // FIXME: Split control

    // First we send the SETUP packet, which are always the first 8 bytes in the buffer of the transfer.
    int packet_id = 0b11; // setup

    m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.raw = 0;
    m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.pid = packet_id;
    m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.transfer_size = sizeof(USBRequestData);

    m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count = 1;

    m_csr_regs->host_mode_regs.HC[channel].HCDMA = transfer.buffer_physical().get();

    m_csr_regs->host_mode_regs.HC[channel].HCCHAR.multi_count = 1;
    m_csr_regs->host_mode_regs.HC[channel].HCCHAR.channel_enable = 1;

    // TODO: While not transfer complete wait (or something)
    // u32 hcint = m_csr_regs->host_mode_regs.HC[channel].HCINT;
    // dbgln("HCINT: 0x{:x}", hcint);

    // dbgln("Packet count after sending: {}", (u32)m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count);
    // if (m_csr_regs->host_mode_regs.HC[channel].HCINT)

    if (transfer.transfer_data_size() > 0) {
        // Now comes the data packets

        m_csr_regs->host_mode_regs.HC[channel].HCCHAR.endpoint_direction = direction_in ? 1 : 0;

        packet_id = 0b10; // data1

        m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.raw = 0;
        m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.pid = packet_id;
        m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.transfer_size = transfer.transfer_data_size();

        if (pipe.device_speed() == Pipe::DeviceSpeed::LowSpeed)
            m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count = (transfer.transfer_data_size() + 7) / 8;
        else
            m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count = (transfer.transfer_data_size() + pipe.max_packet_size() - 1) / pipe.max_packet_size();

        if (m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count == 0) {
            m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count = 1;
        }

        u32 packet = 0;
        do {
            m_csr_regs->host_mode_regs.HC[channel].HCDMA = transfer.buffer_physical().offset(sizeof(USBRequestData)).offset(packet).get();

            m_csr_regs->host_mode_regs.HC[channel].HCCHAR.multi_count = 1;
            m_csr_regs->host_mode_regs.HC[channel].HCCHAR.channel_enable = 1;

            u32 hcint = m_csr_regs->host_mode_regs.HC[channel].HCINT;
            dbgln("HCINT: 0x{:x}", hcint);

            // dbgln("Packet count after sending: {}", (u32)m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count);
            packet = transfer.transfer_data_size() - m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.transfer_size;
        } while ((u32)m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count > 0);
    }

    // Then we need the status packet (which has zero size (?))
    packet_id = 0b10; // data1

    m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.raw = 0;
    m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.pid = packet_id;
    m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.transfer_size = 0;

    if (pipe.device_speed() == Pipe::DeviceSpeed::LowSpeed)
        m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count = 0;
    else
        m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count = 0;

    if (m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count == 0) {
        m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count = 1;
    }

    // Does the address matter if the length is zero?
    m_csr_regs->host_mode_regs.HC[channel].HCDMA = transfer.buffer_physical().offset(sizeof(USBRequestData)).get();

    m_csr_regs->host_mode_regs.HC[channel].HCCHAR.multi_count = 1;
    m_csr_regs->host_mode_regs.HC[channel].HCCHAR.channel_enable = 1;

    dbgln("Done submit_control_transfer");

    // hcint = m_csr_regs->host_mode_regs.HC[channel].HCINT;
    // dbgln("HCINT: 0x{:x}", hcint);

    // dbgln("Packet count after sending: {}", (u32)m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count);

    (void)direction_in;
    // TransferDescriptor* setup_td = create_transfer_descriptor(pipe, PacketID::SETUP, sizeof(USBRequestData));
    // if (!setup_td)
    //     return ENOMEM;

    // setup_td->set_buffer_address(transfer.buffer_physical().as_ptr());

    // // Create a new descriptor chain
    // TransferDescriptor* last_data_descriptor;
    // TransferDescriptor* data_descriptor_chain;
    // auto buffer_address = Ptr32<u8>(transfer.buffer_physical().as_ptr() + sizeof(USBRequestData));
    // TRY(create_chain(pipe, direction_in ? PacketID::IN : PacketID::OUT, buffer_address, pipe.max_packet_size(), transfer.transfer_data_size(), &data_descriptor_chain, &last_data_descriptor));

    // // Status TD always has toggle set to 1
    // pipe.set_toggle(true);

    // TransferDescriptor* status_td = create_transfer_descriptor(pipe, direction_in ? PacketID::OUT : PacketID::IN, 0);
    // if (!status_td) {
    //     free_descriptor_chain(data_descriptor_chain);
    //     return ENOMEM;
    // }
    // status_td->terminate();

    // // Link transfers together
    // if (data_descriptor_chain) {
    //     setup_td->insert_next_transfer_descriptor(data_descriptor_chain);
    //     last_data_descriptor->insert_next_transfer_descriptor(status_td);
    // } else {
    //     setup_td->insert_next_transfer_descriptor(status_td);
    // }

    // // Cool, everything should be chained together now! Let's print it out
    // if constexpr (UHCI_VERBOSE_DEBUG) {
    //     dbgln("Setup TD");
    //     setup_td->print();
    //     if (data_descriptor_chain) {
    //         dbgln("Data TD");
    //         data_descriptor_chain->print();
    //     }
    //     dbgln("Status TD");
    //     status_td->print();
    // }

    // QueueHead* transfer_queue = allocate_queue_head();
    // if (!transfer_queue) {
    //     free_descriptor_chain(data_descriptor_chain);
    //     return ENOMEM;
    // }

    // transfer_queue->attach_transfer_descriptor_chain(setup_td);
    // transfer_queue->set_transfer(&transfer);

    // enqueue_qh(transfer_queue, m_fs_control_qh_anchor);

    // size_t transfer_size = 0;
    // while (!transfer.complete()) {
    //     dbgln_if(USB_DEBUG, "Control transfer size: {}", transfer_size);
    //     transfer_size = poll_transfer_queue(*transfer_queue);
    // }

    // dequeue_qh(transfer_queue);
    // free_descriptor_chain(transfer_queue->get_first_td());
    // transfer_queue->free();
    // m_queue_head_pool->release_to_pool(transfer_queue);

    // return transfer_size;
    // dbgln("TODO: submit_control_transfer");
    return transfer.transfer_data_size();
}
ErrorOr<size_t> DWC2Controller::submit_bulk_transfer(Transfer&)
{
    dbgln("TODO: submit_bulk_transfer");
    return 0;
}
ErrorOr<void> DWC2Controller::submit_async_interrupt_transfer(NonnullLockRefPtr<Transfer>, u16)
{
    // dbgln_if(UHCI_DEBUG, "DWC2: Received interrupt transfer for address {}. Root Hub is at address {}.", transfer->pipe().device_address(), m_root_hub->device_address());

    // if (ms_interval == 0) {
    //     return EINVAL;
    // }

    // auto async_transfer_handle = TRY(adopt_nonnull_own_or_enomem(new (nothrow) AsyncTransferHandle { transfer, ms_interval }));
    // {
    //     SpinlockLocker locker { m_async_lock };
    //     auto iter = find_if(m_active_async_transfers.begin(), m_active_async_transfers.end(), [](auto& handle) { return handle == nullptr; });
    //     if (iter == m_active_async_transfers.end())
    //         return ENOMEM;
    //     *iter = move(async_handle);
    // }

    // return {};

    // auto& transfer = *transfer_ptr;
    // Pipe& pipe = transfer.pipe(); // Short circuit the pipe related to this transfer

    // dbgln_if(DWC2_DEBUG, "DWC2: Received async interrupt transfer {}. Root Hub is at address {}. Pipe type is {}", pipe.device_address(), m_root_hub->device_address(), to_underlying(pipe.type()));

    // int channel = 0; // We actually have 16 channels available

    // // clear the xfer complete bit
    // m_csr_regs->host_mode_regs.HC[channel].HCINT = 1;

    // // while (true) {
    // m_csr_regs->host_mode_regs.HC[channel].HCCHAR.raw = 0;

    // m_csr_regs->host_mode_regs.HC[channel].HCCHAR.device_address = pipe.device_address();
    // m_csr_regs->host_mode_regs.HC[channel].HCCHAR.endpoint_number = pipe.endpoint_address();
    // m_csr_regs->host_mode_regs.HC[channel].HCCHAR.endpoint_direction = pipe.direction() == Pipe::Direction::In ? 1 : 0;
    // m_csr_regs->host_mode_regs.HC[channel].HCCHAR.low_speed_device = pipe.device_speed() == Pipe::DeviceSpeed::LowSpeed ? 1 : 0;
    // m_csr_regs->host_mode_regs.HC[channel].HCCHAR.endpoint_type = to_underlying(pipe.type());
    // m_csr_regs->host_mode_regs.HC[channel].HCCHAR.maximum_packet_size = pipe.max_packet_size();
    // m_csr_regs->host_mode_regs.HC[channel].HCCHAR.channel_enable = 0;
    // m_csr_regs->host_mode_regs.HC[channel].HCCHAR.channel_disable = 0;

    // // FIXME: Split control

    // // First we send the SETUP packet, which are always the first 8 bytes in the buffer of the transfer.
    // int packet_id = 0b10; // data1

    // m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.raw = 0;
    // m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.pid = packet_id;
    // m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.transfer_size = transfer.transfer_data_size();

    // if (pipe.device_speed() == Pipe::DeviceSpeed::LowSpeed)
    //     m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count = (transfer.transfer_data_size() + 7) / 8;
    // else
    //     m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count = (transfer.transfer_data_size() + pipe.max_packet_size() - 1) / pipe.max_packet_size();

    // if (m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count == 0) {
    //     m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count = 1;
    // }

    // m_csr_regs->host_mode_regs.HC[channel].HCDMA = transfer.buffer_physical().get();

    // m_csr_regs->host_mode_regs.HC[channel].HCCHAR.multi_count = 1;
    // m_csr_regs->host_mode_regs.HC[channel].HCCHAR.channel_enable = 1;

    // // TODO: While not transfer complete wait (or something)
    // u32 hcint = m_csr_regs->host_mode_regs.HC[channel].HCINT;
    // // dbgln("HCINT: 0x{:x}", hcint);

    // // dbgln("Packet count after sending: {}", (u32)m_csr_regs->host_mode_regs.HC[channel].HCTSIZ.packet_count);
    // // if (m_csr_regs->host_mode_regs.HC[channel].HCINT)
    // // dbgln("TODO: submit_async_interrupt_transfer");
    // // break;
    // if (hcint == 0x12) {
    //     // NAK - XFERcomplete - channel halted
    //     // clear NAK and XFERcomplete and try again
    //     m_csr_regs->host_mode_regs.HC[channel].HCINT = 1 | (1 << 4);
    // } else {
    //     m_csr_regs->host_mode_regs.HC[channel].HCINT = 1;
    //     transfer.set_complete();
    // }

    // transfer.invoke_async_callback();
    return {};
}

void DWC2Controller::get_port_status(Badge<DWC2RootHub>, u8 port, HubStatus& hub_port_status)
{
    VERIFY(port < NUMBER_OF_ROOT_PORTS);

    auto hprt = m_csr_regs->host_mode_regs.HPRT.raw;
    dbgln("HPRT0: 0x{:x}", hprt);

    if (m_csr_regs->host_mode_regs.HPRT.port_connect_status)
        hub_port_status.status |= PORT_STATUS_CURRENT_CONNECT_STATUS;

    if (m_csr_regs->host_mode_regs.HPRT.port_connect_detected)
        hub_port_status.change |= PORT_STATUS_CONNECT_STATUS_CHANGED;

    if (m_csr_regs->host_mode_regs.HPRT.port_enable)
        hub_port_status.status |= PORT_STATUS_PORT_ENABLED;

    if (m_csr_regs->host_mode_regs.HPRT.port_enable_change)
        hub_port_status.change |= PORT_STATUS_PORT_ENABLED_CHANGED;

    if (m_csr_regs->host_mode_regs.HPRT.port_speed == 0b00)
        hub_port_status.status |= PORT_STATUS_HIGH_SPEED_DEVICE_ATTACHED;

    if (m_csr_regs->host_mode_regs.HPRT.port_speed == 0b10)
        hub_port_status.status |= PORT_STATUS_LOW_SPEED_DEVICE_ATTACHED;

    if (m_csr_regs->host_mode_regs.HPRT.port_reset)
        hub_port_status.status |= PORT_STATUS_RESET;

    // TODO: Can we detect this somehow?
    hub_port_status.change |= PORT_STATUS_RESET_CHANGED;

    if (m_csr_regs->host_mode_regs.HPRT.port_suspend)
        hub_port_status.status |= PORT_STATUS_SUSPEND;

    // hub_port_status.change |= PORT_STATUS_SUSPEND_CHANGED;

    if (m_csr_regs->host_mode_regs.HPRT.port_power)
        hub_port_status.status |= PORT_STATUS_PORT_POWER;

    if (m_csr_regs->host_mode_regs.HPRT.port_overcurrent_active)
        hub_port_status.status |= PORT_STATUS_OVER_CURRENT;

    if (m_csr_regs->host_mode_regs.HPRT.port_overcurrent_change)
        hub_port_status.change |= PORT_STATUS_OVER_CURRENT_INDICATOR_CHANGED;

    dbgln_if(DWC2_DEBUG, "UHCI: get_port_status status=0x{:04x} change=0x{:04x}", hub_port_status.status, hub_port_status.change);
    // return hub_port_status;
    // dbgln("TODO: get_port_status");
}

void DWC2Controller::reset_port(u8 port)
{
    // We still have to reset the port manually because UHCI does not automatically enable the port after reset.
    // Additionally, the USB 2.0 specification says the SetPortFeature(PORT_ENABLE) request is not specified and that the _ideal_ behavior is to return a Request Error.
    // Source: USB 2.0 Specification Section 11.24.2.7.1.2
    // This means the hub code cannot rely on using it.

    // The check is done by UHCIRootHub and set_port_feature.
    VERIFY(port < NUMBER_OF_ROOT_PORTS);

    // // u16 port_data = port == 0 ? read_portsc1() : read_portsc2();
    // port_data &= UHCI_PORTSC_NON_WRITE_CLEAR_BIT_MASK;
    // port_data |= UHCI_PORTSC_PORT_RESET;
    // if (port == 0)
    //     write_portsc1(port_data);
    // else
    //     write_portsc2(port_data);

    // Wait at least 50 ms for the port to reset.
    // This is T DRSTR in the USB 2.0 Specification Page 186 Table 7-13.
    constexpr u16 reset_delay = 50 * 1000;
    microseconds_delay(reset_delay);

    // port_data &= ~UHCI_PORTSC_PORT_RESET;
    // if (port == 0)
    //     write_portsc1(port_data);
    // else
    //     write_portsc2(port_data);

    // Wait 10 ms for the port to recover.
    // This is T RSTRCY in the USB 2.0 Specification Page 188 Table 7-14.
    constexpr u16 reset_recovery_delay = 10 * 1000;
    microseconds_delay(reset_recovery_delay);

    // port_data = port == 0 ? read_portsc1() : read_portsc2();
    // port_data |= UHCI_PORTSC_PORT_ENABLED;
    // if (port == 0)
    //     write_portsc1(port_data);
    // else
    //     write_portsc2(port_data);

    // dbgln_if(DWC2_DEBUG, "UHCI: Port should be enabled now: {:#04x}", port == 0 ? read_portsc1() : read_portsc2());
    // m_port_reset_change_statuses |= (1 << port);
}

ErrorOr<void> DWC2Controller::set_port_feature(Badge<DWC2RootHub>, u8 port, HubFeatureSelector feature_selector)
{
    // The check is done by UHCIRootHub.
    VERIFY(port < NUMBER_OF_ROOT_PORTS);

    dbgln_if(DWC2_DEBUG, "UHCI: set_port_feature: port={} feature_selector={}", port, (u8)feature_selector);

    switch (feature_selector) {
    case HubFeatureSelector::PORT_POWER:
        // m_csr_regs->host_mode_regs.HPRT.port_power = 1;
        break;
    case HubFeatureSelector::PORT_RESET:
        // TODO: Do some stuff with power register

        m_csr_regs->host_mode_regs.HPRT.port_reset = 1;
        // TODO: Wait some time
        m_csr_regs->host_mode_regs.HPRT.port_reset = 0;

        // reset_port(port);
        break;
    case HubFeatureSelector::PORT_SUSPEND: {
        // u16 port_data = port == 0 ? read_portsc1() : read_portsc2();
        // port_data &= UHCI_PORTSC_NON_WRITE_CLEAR_BIT_MASK;
        // port_data |= UHCI_PORTSC_SUSPEND;

        // if (port == 0)
        //     write_portsc1(port_data);
        // else
        //     write_portsc2(port_data);

        // m_port_suspend_change_statuses |= (1 << port);
        break;
    }
    default:
        dbgln("UHCI: Unknown feature selector in set_port_feature: {}", (u8)feature_selector);
        return EINVAL;
    }

    return {};
    // TODO();
    // dbgln("TODO: set_port_feature");
    // return {};
}
ErrorOr<void> DWC2Controller::clear_port_feature(Badge<DWC2RootHub>, u8 port, HubFeatureSelector feature_selector)
{
    // The check is done by UHCIRootHub.
    VERIFY(port < NUMBER_OF_ROOT_PORTS);

    dbgln_if(DWC2_DEBUG, "UHCI: clear_port_feature: port={} feature_selector={}", port, (u8)feature_selector);

    // u16 port_data = port == 0 ? read_portsc1() : read_portsc2();
    // port_data &= UHCI_PORTSC_NON_WRITE_CLEAR_BIT_MASK;

    switch (feature_selector) {
    case HubFeatureSelector::PORT_ENABLE:
        m_csr_regs->host_mode_regs.HPRT.port_enable = 0;
        // port_data &= ~UHCI_PORTSC_PORT_ENABLED;
        break;
    case HubFeatureSelector::PORT_SUSPEND:
        m_csr_regs->host_mode_regs.HPRT.port_suspend = 0;
        // port_data &= ~UHCI_PORTSC_SUSPEND;
        break;
    case HubFeatureSelector::PORT_POWER:
        m_csr_regs->host_mode_regs.HPRT.port_power = 0;
        break;
    case HubFeatureSelector::C_PORT_CONNECTION:
        // This field is Write Clear.
        m_csr_regs->host_mode_regs.HPRT.port_connect_detected = 1;
        break;
    case HubFeatureSelector::C_PORT_RESET:
        // m_port_reset_change_statuses &= ~(1 << port);
        break;
    case HubFeatureSelector::C_PORT_ENABLE:
        // This field is Write Clear.
        m_csr_regs->host_mode_regs.HPRT.port_enable_change = 1;
        // port_data |= UHCI_PORTSC_PORT_ENABLE_CHANGED;
        break;
    case HubFeatureSelector::C_PORT_SUSPEND:
        // m_csr_regs->host_mode_regs.HPRT.port_suspend_
        // m_port_suspend_change_statuses &= ~(1 << port);
        break;
    default:
        dbgln("UHCI: Unknown feature selector in clear_port_feature: {}", (u8)feature_selector);
        return EINVAL;
    }

    // dbgln_if(DWC2_DEBUG, "UHCI: clear_port_feature: writing 0x{:04x} to portsc{}.", port_data, port + 1);

    // if (port == 0)
    //     write_portsc1(port_data);
    // else
    //     write_portsc2(port_data);

    return {};
}
}
