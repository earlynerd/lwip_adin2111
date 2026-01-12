#pragma once
#include "LwipIntfDev.h"
#include "utility/adin2111_wrap.h"
//#include <EthernetCompat.h>
#include <LwipEthernet.h>

// Define the interface using the core's template class
using ADIN2111_lwIP = LwipIntfDev<ADIN2111_wrap>;