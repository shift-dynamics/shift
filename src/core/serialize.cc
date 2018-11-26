#include "shift-core/serialize.h"

void Shift::Serialize::addSerialEntry(const std::string keyword,
                                      const SerialType serialType, void* data,
                                      int rowDim, int colDim) {
  SerialEntry serialEntry(keyword, serialType, data, rowDim, colDim);
  serialEntries_.insert(serialEntries_.end(), serialEntry);
}
