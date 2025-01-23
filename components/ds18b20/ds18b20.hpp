#ifndef DS18B20_HPP
#define DS18B20_HPP

#include "ow_device.hpp"
#include <cstdint>
#include <optional>

namespace DS18B20 {

    struct DS18B20 {
        enum struct DevAddress : std::uint64_t {
            ADDRESS = 0,
        };

        enum struct ROM : std::uint16_t {
            READ_ROM = 0x33,
            MATCH_ROM = 0x55,
            SKIP_ROM = 0xCC,
            SEARCH_ROM = 0xF0,
            ALARM_SERACH = 0xEC,
        };

        enum struct MEM : std::uint16_t {
            WRITE_SCRATCHPAD = 0x4E,
            READ_SCRATCHPAD = 0xBE,
            COPY_SCRATCHPAD = 0x48,
            CONVERT_T = 0x44,
            RECALL_E2 = 0xB8,
            READ_POW_SUP = 0xB4,
        };

        using Scaled = float;
        using Raw = std::int16_t;

        static inline std::array<StackType_t, sizeof(rmt_rx_done_event_data_t)> queue_storage;
        static inline StaticQueue_t static_queue;
        static inline OWDevice ow_device{
            gpio_num_t::GPIO_NUM_0,
            xQueueCreateStatic(1UL, sizeof(rmt_rx_done_event_data_t), queue_storage.data(), &static_queue)};

        [[nodiscard]] OptionalRaw get_temperature_raw() const noexcept;
        [[nodiscard]] OptionalScaled get_temperature_scaled() const noexcept;

        DevAddress address{};
    };

}; // namespace DS18B20

#endif // DS18B20_HPP
