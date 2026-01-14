#include "tof_provider.h"

#if CONFIG_TOF_PROVIDER_MOCK
#include "tof_provider_mock.c"
#else
#include "tof_provider_vl53.c"
#endif
