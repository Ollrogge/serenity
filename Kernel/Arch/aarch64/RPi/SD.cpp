/*
 * Copyright (c) 2023, Timon Kruiper <timonkruiper@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <Kernel/Arch/aarch64/ASM_wrapper.h>
#include <Kernel/Arch/aarch64/RPi/GPIO.h>
#include <Kernel/Arch/aarch64/RPi/MMIO.h>
#include <Kernel/Arch/aarch64/RPi/SD.h>

namespace Kernel::RPi {

// control 1 reg, offset 0x2c
enum {
    C1_CLK_INTLEN = (1 << 0),
    C1_CLK_STABLE = (1 << 1),
    C1_CLK_EN = (1 << 2),
    C1_CLK_GENSEL = (1 << 5),
    C1_CLK_FREQ_MS2 = (1 << 6),
    C1_CLK_FREQ_SEL_8 = (1 << 8), // frequency select
    C1_DATA_TOUNIT_MAX = (14 << 16),
    C1_DATA_TOUNIT_DIS = (15 << 16),
    C1_COMPLETE_RESET = (1 << 24),
    C1_SRST_CMD = (1 << 25),
    C1_SRST_DATA = (1 << 26)
};

// simplified physical layer spec
// 4.7.4 Detailed Command Description
enum { CMD_GO_IDLE_STATE = 0 };

#define CDIV_MAX 0x3ff

#define INT_ERROR_MASK 0x017E8000

// capabilities_0 reg, offset 0x40
enum {
    CAP0_BCLK_8 = (((1 << 8) - 1) << 8) // Base clock frequency
};

// capabilities_1 reg, offset 0x44
enum { CAP1_CM_8 = (((1 << 7) - 1) << 16) }; // clock multiplier

// status reg
enum {
    ST_DATA_INHIBIT = (1 << 1), // data line still used
    ST_CMD_INHIBIT = (1 << 0)   // command line still used
};

// int reg
enum {
    INT_CMD_DONE = (1 << 0),
    INT_CMD_TIMEOUT = (1 << 16), // timeout on cmd line
    INT_DATA_TIMEOUT = (1 << 20) // timeout on data line
};

enum { SD_OK = 0, SD_TIMEOUT = -1, SD_ERROR = -2 };

// SD Host Controller Simplified Specification Version 3.00
// NOTE: The registers must be 32 bits, because of a quirk in the RPI.
struct SDRegisters {
    u32 argument_2;                   // 0x00
    u32 block_size_and_block_count;   // 0x04
    u32 argument_1;                   // 0x08
    u32 transfer_mode_and_command;    // 0x0c
    u32 response_0;                   // 0x10
    u32 response_1;                   // 0x14
    u32 response_2;                   // 0x18
    u32 response_3;                   // 0x1c
    u32 buffer_data_port;             // 0x20
    u32 status;                       // 0x24
    u32 control_0;                    // 0x28
    u32 control_1;                    // 0x2c
    u32 interrupt;                    // 0x30
    u32 interrupt_mask;               // 0x34
    u32 interrupt_enable;             // 0x38
    u32 host_control_2;               // 0x3c
    u32 capabilities_0;               // 0x40
    u32 capabilities_1;               // 0x44
    u32 maximum_current_capabilities; // 0x48
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

SD::SD() : m_registers(MMIO::the().peripheral<SDRegisters>(0x30'0000)) {
    // Set up correct GPIO
    auto &gpio = GPIO::the();
    gpio.set_pin_function(21, GPIO::PinFunction::Alternate3); // CD
    gpio.set_pin_high_detect_enable(21, true);

    gpio.set_pin_function(22, GPIO::PinFunction::Alternate3); // SD1_CLK
    gpio.set_pin_function(23, GPIO::PinFunction::Alternate3); // SD1_CMD

    gpio.set_pin_function(24, GPIO::PinFunction::Alternate3); // SD1_DAT0
    gpio.set_pin_function(25, GPIO::PinFunction::Alternate3); // SD1_DAT1
    gpio.set_pin_function(26, GPIO::PinFunction::Alternate3); // SD1_DAT2
    gpio.set_pin_function(27, GPIO::PinFunction::Alternate3); // SD1_DAT3

    gpio.set_pin_pull_up_down_state(Array{21, 22, 23, 24, 25, 26, 27},
                                    GPIO::PullUpDownState::PullDown);

    // dbgln("{:p}", &m_registers->slot_interrupt_status_and_version);
    // Version
    dbgln("SD: SD Host Specification Version {}.00",
          ((m_registers->slot_interrupt_status_and_version >> 16) & 0xff) + 1);

    // Reset SD Host Controller
    m_registers->control_0 = 0;
    m_registers->control_1 =
        set_flags(m_registers->control_1, C1_COMPLETE_RESET);
    while (flags_set(m_registers->control_1, C1_COMPLETE_RESET)) {
    }

    set_clock(25'000'000);

    // enable all interrupts
    m_registers->interrupt_enable = 0xffffffff;
    m_registers->interrupt_mask = 0xffffffff;

    exec_cmd(CMD_GO_IDLE_STATE, 0);
}

SD &SD::the() {
    static SD instance;
    return instance;
}

bool SD::flags_set(u32 reg, u32 flags) { return (reg & flags) == flags; }

bool SD::flags_any_set(u32 reg, u32 flags) { return (reg & flags) > 0; }

u32 SD::clear_flags(u32 reg, u32 flags) { return reg & ~flags; }

u32 SD::set_flags(u32 reg, u32 flags) { return reg | flags; }

// SCLK = core clock / CDIV
// CDIV = 0 -> divisor = 65536
// SPI clock = sys clock / 2 * (speed_field + 1)
// sys clock = 250 MHz
// base freq = 52 MHZ
int SD::set_clock(u32 hz) {
    // wait for data and cmd line to be rdy
    while (flags_set(m_registers->status, ST_CMD_INHIBIT | ST_DATA_INHIBIT)) {
    }

    // SD Clock supply sequence:

    // base (max) clock freq
    u32 bclock = (m_registers->capabilities_0 & CAP0_BCLK_8) >> 8;
    bclock *= 1'000'000;
    u32 div = bclock / hz;

    // calibrate divisor
    if (div < 2) {
        div = 2;
    }

    if (bclock / div > hz) {
        div++;
    }

    div -= 2;

    if (div > CDIV_MAX) {
        div = CDIV_MAX;
    }

    dbgln("SD: base freq: {}, clock divisor: {}", bclock, div);

    // SDCLK Frequency Select
    m_registers->control_1 = m_registers->control_0 | (div << 8);

    // internal clock enable
    m_registers->control_1 = set_flags(m_registers->control_0, C1_CLK_INTLEN);

    // wait for clock to be stable
    while (!flags_set(m_registers->control_1, C1_CLK_STABLE)) {
    }

    // sd clock enable
    m_registers->control_1 = set_flags(m_registers->control_0, C1_CLK_EN);

    dbgln("SD: clock supply sequence finished");

    return 0;
}

// 3.7.1 Transaction Control without Data Transfer Using DAT Line
int SD::exec_cmd(u32 code, u32 arg) {
    while (
        flags_any_set(m_registers->status, ST_CMD_INHIBIT | ST_DATA_INHIBIT)) {
    }

    // issue command
    m_registers->argument_1 = arg;
    m_registers->transfer_mode_and_command = code;

    int ret = wait_for_interrupt(INT_CMD_DONE);

    dbgln("Waiting for int done {}", ret);
    return 0;
}

// 3.7.1.2 The Sequence to Finalize a Command
int SD::wait_for_interrupt(u32 mask) {
    u32 m = mask | INT_ERROR_MASK;
    while (!flags_any_set(m_registers->interrupt, m)) {
    }

    if (flags_any_set(m, INT_CMD_TIMEOUT | INT_DATA_TIMEOUT)) {
        return SD_TIMEOUT;
    }

    if (flags_set(m, INT_ERROR_MASK)) {
        return SD_ERROR;
    }

    return SD_OK;
}

} // namespace Kernel::RPi
