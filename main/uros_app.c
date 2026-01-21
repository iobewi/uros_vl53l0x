#include "uros_app.h"

#include "micro_ros_adapter.h"

bool uros_app_start(void)
{
    return micro_ros_adapter_start();
}
