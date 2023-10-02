/*
 * Copyright (c) 2021, Luke Wilde <lukew@serenityos.org>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <AK/Error.h>
#include <AK/NonnullOwnPtr.h>
#include <Kernel/Bus/USB/USBHub.h>
#include <Kernel/Bus/USB/USBTransfer.h>
#include <Kernel/Library/NonnullLockRefPtr.h>

namespace Kernel::USB {

class DWC2Controller;

class DWC2RootHub {
public:
    static ErrorOr<NonnullOwnPtr<DWC2RootHub>> try_create(NonnullLockRefPtr<DWC2Controller>);

    DWC2RootHub(NonnullLockRefPtr<DWC2Controller>);
    ~DWC2RootHub() = default;

    ErrorOr<void> setup(Badge<DWC2Controller>);

    u8 device_address() const { return m_hub->address(); }

    ErrorOr<size_t> handle_control_transfer(Transfer& transfer);

    void check_for_port_updates() { m_hub->check_for_port_updates(); }
    bool can_check_for_port_updates() { return m_hub->can_check_for_port_updates(); }

private:
    NonnullLockRefPtr<DWC2Controller> m_dwc2_controller;
    LockRefPtr<Hub> m_hub;
};

}
