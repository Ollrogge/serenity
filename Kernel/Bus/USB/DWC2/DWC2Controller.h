/*
 * Copyright (c) 2023, Timon Kruiper <timonkruiper@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <Kernel/Bus/USB/DWC2/DWC2RootHub.h>
#include <Kernel/Bus/USB/USBController.h>
#include <Kernel/Bus/USB/USBHub.h>
#include <Kernel/Forward.h>

namespace Kernel::USB {

struct ControlAndStatusRegisters;

class DWC2Controller final : public USBController {
public:
    static constexpr u8 NUMBER_OF_ROOT_PORTS = 1;
    static ErrorOr<NonnullLockRefPtr<DWC2Controller>> try_to_initialize();

    // ^USBController
    virtual ErrorOr<void> initialize() override;

    virtual ErrorOr<void> reset() override;
    virtual ErrorOr<void> stop() override;
    virtual ErrorOr<void> start() override;

    virtual void cancel_async_transfer(NonnullLockRefPtr<Transfer> transfer) override;
    virtual ErrorOr<size_t> submit_control_transfer(Transfer&) override;
    virtual ErrorOr<size_t> submit_bulk_transfer(Transfer& transfer) override;
    virtual ErrorOr<void> submit_async_interrupt_transfer(NonnullLockRefPtr<Transfer> transfer, u16 ms_interval) override;

    void get_port_status(Badge<DWC2RootHub>, u8, HubStatus&);
    ErrorOr<void> set_port_feature(Badge<DWC2RootHub>, u8, HubFeatureSelector);
    ErrorOr<void> clear_port_feature(Badge<DWC2RootHub>, u8, HubFeatureSelector);

private:
    explicit DWC2Controller(ControlAndStatusRegisters volatile*);

    void reset_port(u8);
    ErrorOr<void> spawn_async_poll_process();
    ErrorOr<void> spawn_port_process();

    void dwc2_reset();

    ControlAndStatusRegisters volatile* m_csr_regs;
    OwnPtr<DWC2RootHub> m_root_hub;
};

}
