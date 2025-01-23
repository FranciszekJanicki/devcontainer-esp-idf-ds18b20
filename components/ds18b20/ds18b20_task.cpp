#include "ds18b20_task.hpp"
#include "ds18b20.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace DS18B20 {

    namespace {

        constexpr auto TAG{"DS18B20"};
        StackType_t task_stack[4096UL];
        StaticTask_t static_task;

        void task(void* parameter) noexcept
        {
            DS18B20 ds18b20{DS18B20::DevAddress::ADDRESS};

            while (true) {
                ESP_LOGI("temperature: %f", ds18b20.get_temperature_scaled());
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }

    }; // namespace

    void start_task() noexcept
    {
        xTaskCreateStaticPinnedToCore(&task, "task", 4096UL, nullptr, 1, task_stack, &static_task, 1);
    }

}; // namespace DS18B20