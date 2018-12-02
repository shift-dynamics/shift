#include "shift-core/serialize.h"

void Shift::Serialize::addSerialEntry(SerialEntry& serialEntry) {
  serialEntries_.insert(serialEntries_.end(), serialEntry);
}

void Shift::SerialEntry::addSerialEntry(SerialEntry& serialEntry) {
  serialEntries_.insert(serialEntries_.end(), serialEntry);
}
