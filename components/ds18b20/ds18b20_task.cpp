#include "ds18b20_task.hpp"
#include "ds18b20.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace DS18B20 {

    static constexpr auto TASK_TAG{"DS18B20"};
    static constexpr auto TASK_STACK_SIZE{4096UL};
    static StackType_t task_stack[TASK_STACK_SIZE];
    static StaticTask_t static_task;

    static constexpr auto OW_GPIO_PIN{gpio_num_t::GPIO_NUM_0};
    static constexpr auto OW_QUEUE_SIZE{sizeof(rmt_rx_done_event_data_t)};
    static StackType_t ow_queue_storage[OW_QUEUE_SIZE];
    static StaticQueue_t ow_static_queue;

    static void task(void* parameter) noexcept
    {
        OWDevice ow_device{OW_GPIO_PIN, xQueueCreateStatic(1UL, OW_QUEUE_SIZE, ow_queue_storage, &ow_static_queue)};

        DS18B20 ds18b20{.address = DS18B20::DevAddress::ADDRESS, .ow_device = std::move(ow_device)};

        while (true) {
            ESP_LOGI(TASK_TAG, "temperature: %f", ds18b20.get_temperature_scaled());
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    void start_task() noexcept
    {
        xTaskCreateStaticPinnedToCore(&task, TASK_TAG, 4096UL, nullptr, 1, task_stack, &static_task, 1);
    }

}; // namespace DS18B20