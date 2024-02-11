#pragma once
#include <inttypes.h>

namespace modules{
namespace krpc_client {

void Setup();
void Cycle();

void NotifyCommRx();
void NotifyCommTx();
void NotifyCommLineState();

}
}
