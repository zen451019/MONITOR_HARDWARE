#ifndef SESSION_STORE_H
#define SESSION_STORE_H

#include <cstddef>
#include <cstdint>

class LoRaWANNode;

bool sessionStoreInit(LoRaWANNode& node);
size_t sessionStoreCount();
bool sessionStoreSelect(LoRaWANNode& node, size_t index);
void sessionStorePersist(LoRaWANNode& node, size_t index);
uint32_t sessionStoreFcntUp(size_t index);
uint32_t sessionStoreDevAddr(size_t index);

#endif // SESSION_STORE_H
