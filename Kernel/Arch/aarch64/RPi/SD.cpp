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
    C1_RESET_CMD = (1 << 25),
    C1_RESET_DATA = (1 << 26)
};

// BCM2835 ARM Peripherals: CMDTM register
typedef struct {
    u8 res0 : 1;
    u8 block_count : 1;
    u8 auto_command : 2;
    u8 direction : 1;
    u8 multiblock : 1;
    u16 res1 : 10;
    u8 response_type : 2;
    u8 res2 : 1;
    u8 crc_enable : 1;
    u8 idx_enable : 1;
    u8 is_data : 1;
    u8 type : 2;
    u8 index : 6;
    u8 res3 : 2;

    operator u32() const {
        return (res3 << 30) | (index << 24) | (type << 22) | (is_data << 21) |
               (idx_enable << 20) | (crc_enable << 19) | (res2 << 18) |
               (response_type << 16) | (res1 << 6) | (multiblock << 5) |
               (direction << 4) | (auto_command << 2) | (block_count << 1) |
               res0;
    }

} emmc_cmd;

#define RES_CMD                                                                \
    { 1, 1, 3, 1, 1, 0xF, 3, 1, 1, 1, 1, 3, 0xF, 3 }

typedef enum { RTNone, RT136, RT48, RT48Busy } cmd_resp_type;

static const emmc_cmd commands[] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    RES_CMD,
    {0, 0, 0, 0, 0, 0, RT136, 0, 1, 0, 0, 0, 2, 0},
    {0, 0, 0, 0, 0, 0, RT48, 0, 1, 0, 0, 0, 3, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0},
    {0, 0, 0, 0, 0, 0, RT136, 0, 0, 0, 0, 0, 5, 0},
    {0, 0, 0, 0, 0, 0, RT48, 0, 1, 0, 0, 0, 6, 0},
    {0, 0, 0, 0, 0, 0, RT48Busy, 0, 1, 0, 0, 0, 7, 0},
    {0, 0, 0, 0, 0, 0, RT48, 0, 1, 0, 0, 0, 8, 0},
    {0, 0, 0, 0, 0, 0, RT136, 0, 1, 0, 0, 0, 9, 0},
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    {0, 0, 0, 0, 0, 0, RT48, 0, 1, 0, 0, 0, 16, 0},
    {0, 0, 0, 1, 0, 0, RT48, 0, 1, 0, 1, 0, 17, 0},
    {0, 1, 1, 1, 1, 0, RT48, 0, 1, 0, 1, 0, 18, 0},
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    {0, 0, 0, 0, 0, 0, RT48, 0, 0, 0, 0, 0, 41, 0},
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    RES_CMD,
    {0, 0, 0, 1, 0, 0, RT48, 0, 1, 0, 1, 0, 51, 0},
    RES_CMD,
    RES_CMD,
    RES_CMD,
    {0, 0, 0, 0, 0, 0, RT48, 0, 1, 0, 0, 0, 55, 0},
};

// simplified physical layer spec
// 4.7.4 Detailed Command Description
enum {
    CMD_GO_IDLE_STATE = 0,
    CMD_IO_SEND_OP_COND = 5,
    CMD_SEND_IF_COND = 8,
    CMD_SEND_OP_COND = 41,
    CMD_APP = 55
};

#define C1_TOUNIT_MAX 0x000e0000

#define CMD_NEEDS_APP 0x80000000
#define CMD_RSPNS_48 0x00020000
#define CMD_ERRORS_MASK 0xfff9c004
#define CDIV_MAX 0x3ff

// support all voltage ranges
#define ACMD41_ARG 0x00FF8000
#define ACMD41_CMD_COMPLETE 0x80000000
#define ACMD41_VOLTAGE 0x00ff8000
#define ACMD41_CCS 0x40000000

// capabilities_0 reg
enum {
    CAP0_BCLK_8 = (((1 << 8) - 1) << 8) // Base clock frequency
};

// capabilities_1 reg
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
    relative_card_address = 0x0;
    operations_conditions_register = 0x0;

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
    while (flags_set(m_registers->control_1,
                     C1_COMPLETE_RESET | C1_RESET_CMD | C1_RESET_DATA)) {
    }

    set_clock(400'000);

    // enable all interrupts
    m_registers->interrupt_enable = 0xffffffff;
    m_registers->interrupt_mask = 0xffffffff;

    int ret = exec_cmd(CMD_GO_IDLE_STATE, 0);
    if (ret != SD_OK) {
        return;
    }

    // Physical layer spec, 4.7.4 Detailed Command Description
    // SD interface condition, 7:0 = check pattern, 11:8 = supply voltage
    // set voltage to default: 2.7-3.6V
    ret = exec_cmd(CMD_SEND_IF_COND, 0x1aa);
    if (ret == SD_ERROR) {
        return;
    }

    bool is_v2_or_higher = false;
    if (ret == SD_OK) {
        is_v2_or_higher = true;

        // We expect the card to echo back voltage range and check pattern
        if ((m_registers->response_0 & 0xfff) != 0x1aa) {
            dbgln("SD: Unusable SD card");
            return;
        }
    }

    // step 5
    ret = exec_cmd(CMD_IO_SEND_OP_COND, 0);
    if (ret == SD_ERROR) {
        return;
    }

    if (ret == SD_OK) {
        sdio_support = true;
        // FIXME: sdio function support
    } else {
        sdio_support = false;
        if (initialize_card(is_v2_or_higher) != SD_OK) {
            return;
        }

        dbgln("SD: Intialization command complete, OCR: {:x}, shdx : {}",
              operations_conditions_register, sdhc);
    }

    // step 11 -> 19 (f8 flag = true)
}

SD &SD::the() {
    static SD instance;
    return instance;
}

bool SD::flags_set(u32 reg, u32 flags) { return (reg & flags) == flags; }

bool SD::flags_any_set(u32 reg, u32 flags) { return (reg & flags) > 0; }

u32 SD::clear_flags(u32 reg, u32 flags) { return reg & ~flags; }

u32 SD::set_flags(u32 reg, u32 flags) { return reg | flags; }

int SD::initialize_card(bool is_v2_or_higher) {
    bool card_busy = true;
    u32 v2_flags = 0x0;
    int ret;
    while (card_busy) {
        if (is_v2_or_higher) {
            v2_flags |= (1 << 30);
        }

        // support all voltage ranges + sdhc if available
        ret = exec_cmd(CMD_SEND_OP_COND, 0x00FF8000 | v2_flags, true);
        if (ret != SD_OK) {
            return ret;
        }

        if (flags_set(m_registers->response_0, ACMD41_CMD_COMPLETE)) {
            operations_conditions_register =
                m_registers->response_0 >> 8 & 0xffff;
            sdhc = flags_set(m_registers->response_0, ACMD41_CCS);
            card_busy = false;
        } else {
            dbgln("SD: Waiting for card initialization to complete");
            Aarch64::Asm::wait_cycles(150);
        }
    }

    return SD_OK;
}

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

    // set data timeout exponent
    m_registers->control_1 = set_flags(m_registers->control_1, C1_TOUNIT_MAX);

    // internal clock enable
    m_registers->control_1 = set_flags(m_registers->control_1, C1_CLK_INTLEN);

    // wait for clock to be stable
    while (!flags_set(m_registers->control_1, C1_CLK_STABLE)) {
    }

    // sd clock enable
    m_registers->control_1 = set_flags(m_registers->control_1, C1_CLK_EN);

    dbgln("SD: clock supply sequence finished");

    return 0;
}

// 3.7.1 Transaction Control without Data Transfer Using DAT Line
int SD::exec_cmd(u32 code, u32 arg, bool is_app_cmd) {
    int ret = SD_OK;

    // application specific command
    // physical layer simpliefied spec: 4.3.9.1
    if (is_app_cmd) {
        if (relative_card_address != 0) {
            // todo;
        }
        ret = exec_cmd(CMD_APP, relative_card_address);
        dbgln("app cmd resp: {}", ret);
        if (ret != SD_OK) {
            dbgln("SD: APP CMD failed");
            return ret;
        }
    }

    while (
        flags_any_set(m_registers->status, ST_CMD_INHIBIT | ST_DATA_INHIBIT)) {
    }

    // issue command
    m_registers->argument_1 = arg;
    m_registers->transfer_mode_and_command = static_cast<u32>(commands[code]);

    ret = wait_for_interrupt(INT_CMD_DONE);

    return ret;
}

// 3.7.1.2 The Sequence to Finalize a Command
int SD::wait_for_interrupt(u32 mask) {
    u32 m = mask | CMD_ERRORS_MASK;
    int cnt = 0x100000;
    while (!flags_any_set(m_registers->interrupt, m) && cnt-- > 0) {
        Aarch64::Asm::wait_cycles(150);
    }

    u32 ret = m_registers->interrupt;

    if (cnt <= 0 || flags_any_set(ret, INT_CMD_TIMEOUT | INT_DATA_TIMEOUT)) {
        return SD_TIMEOUT;
    }

    if (flags_set(ret, CMD_ERRORS_MASK)) {
        return SD_ERROR;
    }

    return SD_OK;
}

} // namespace Kernel::RPi
