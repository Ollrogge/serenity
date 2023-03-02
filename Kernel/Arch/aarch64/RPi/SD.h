/*
 * Copyright (c) 2023, Timon Kruiper <timonkruiper@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <AK/Types.h>

namespace Kernel::RPi {

struct SDRegisters;

class SD {
  public:
    static SD &the();

  private:
    SD();
    int set_clock(unsigned);
    int exec_cmd(u32, u32);
    int wait_for_interrupt(u32);
    static inline bool flags_set(u32, u32);
    static inline bool flags_any_set(u32, u32);
    static inline u32 clear_flags(u32, u32);
    static inline u32 set_flags(u32, u32);

    SDRegisters volatile *m_registers;
};

} // namespace Kernel::RPi
