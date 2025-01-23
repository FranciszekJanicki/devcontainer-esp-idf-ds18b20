#include "ds18b20_task.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void app_main(void)
{
    DS18B20::start_task();
}

#ifdef __cplusplus
}
#endif
