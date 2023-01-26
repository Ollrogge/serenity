/*
 * Copyright (c) 2023, Timon Kruiper <timonkruiper@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <Kernel/Arch/aarch64/RPi/GPIO.h>
#include <Kernel/Arch/aarch64/RPi/MMIO.h>
#include <Kernel/Arch/aarch64/RPi/SD.h>

namespace Kernel::RPi {

// SD Host Controller Simplified Specification Version 3.00
// NOTE: The registers must be 32 bits, because of a quirk in the RPI.
struct SDRegisters {
    u32 argument_2;
    u32 block_size_and_block_count;
    u32 argument_1;
    u32 transfer_mode_and_command;
    u32 response_0;
    u32 response_1;
    u32 response_2;
    u32 response_3;
    u32 buffer_data_port;
    u32 present_state;
    u32 host_configuration_0;
    u32 host_configuration_1;
    u32 interrupt_status;
    u32 interrupt_status_enable;
    u32 interrupt_signal_enable;
    u32 host_configuration_2;
    u32 capabilities_0;
    u32 capabilities_1;
    u32 maximum_current_capabilities;
    u32 maximum_current_capabilities_reserved;
    u32 force_event_for_auto_cmd_error_status;
    u32 adma_error_status;
    u32 adma_system_address[2];
    u32 preset_value[4];
    u32 reserved_0[28];
    u32 shared_bus_control;
    u32 reserved_1[6];
    u32 slot_interrupt_status_and_version;
};

SD::SD()
    : m_registers(MMIO::the().peripheral<SDRegisters>(0x30'0000))
{
    // Set up correct GPIO
    auto& gpio = GPIO::the();
    gpio.set_pin_function(21, GPIO::PinFunction::Alternate3); // CD
    gpio.set_pin_high_detect_enable(21, true);

    gpio.set_pin_function(22, GPIO::PinFunction::Alternate3); // SD1_CLK
    gpio.set_pin_function(23, GPIO::PinFunction::Alternate3); // SD1_CMD

    gpio.set_pin_function(24, GPIO::PinFunction::Alternate3); // SD1_DAT0
    gpio.set_pin_function(25, GPIO::PinFunction::Alternate3); // SD1_DAT1
    gpio.set_pin_function(26, GPIO::PinFunction::Alternate3); // SD1_DAT2
    gpio.set_pin_function(27, GPIO::PinFunction::Alternate3); // SD1_DAT3

    gpio.set_pin_pull_up_down_state(Array { 21, 22, 23, 24, 25, 26, 27 }, GPIO::PullUpDownState::PullDown);

    // dbgln("{:p}", &m_registers->slot_interrupt_status_and_version);
    // Version
    dbgln("SD: SD Host Specification Version {}.00", ((m_registers->slot_interrupt_status_and_version >> 16) & 0xff) + 1);

    // Reset SD Host Controller
    m_registers->host_configuration_1 = m_registers->host_configuration_1 | (1 << 24);
    while ((m_registers->host_configuration_1 & (1 << 24)) == (1 << 24)) { }
}

SD& SD::the()
{
    static SD instance;
    return instance;
}

}
