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
    static SD& the();

private:
    SD();

    SDRegisters volatile* m_registers;
};

}
