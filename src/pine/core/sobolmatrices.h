#pragma once
#include <psl/stdint.h>

namespace pine {

static constexpr int NSobolDimensions = 1024;
static constexpr int SobolMatrixSize = 52;
extern uint32_t SobolMatrices32[NSobolDimensions * SobolMatrixSize];

extern uint64_t VdCSobolMatrices[][SobolMatrixSize];
extern uint64_t VdCSobolMatricesInv[][SobolMatrixSize];

}  // namespace pine