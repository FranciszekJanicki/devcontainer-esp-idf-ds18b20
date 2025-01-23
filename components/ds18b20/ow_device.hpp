#ifndef OW_DEVICE_HPP
#define OW_DEVICE_HPP

#include "driver/rmt_encoder.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "hal/rmt_types.h"
#include "soc/clk_tree_defs.h"
#include <cstdint>
#include <span>

namespace DS18B20 {

    struct OWDevice {
    public:
        using Symbols = std::span<rmt_symbol_word_t>;

        OWDevice() noexcept = default;
        OWDevice(gpio_num_t const gpio_num, QueueHandle_t const rx_queue) noexcept;

        OWDevice(OWDevice const& other) noexcept = delete;
        OWDevice(OWDevice&& other) noexcept = default;

        OWDevice& operator=(OWDevice const& other) noexcept = delete;
        OWDevice& operator=(OWDevice&& other) noexcept = default;

        ~OWDevice() noexcept;

        void reset() const noexcept;
        void write16(std::uint16_t const data) const noexcept;
        void write64(std::uint64_t const data) const noexcept;
        [[nodiscard]] std::uint8_t read8() const noexcept;

    private:
        static std::uint8_t parse_symbols(Symbols const symbols) noexcept;

        enum Timing : std::uint16_t {
            TIMING_A = 6,
            TIMING_B = 64,
            TIMING_C = 60,
            TIMING_D = 10,
            TIMING_E = 9,
            TIMING_F = 55,
            TIMING_G = 0,
            TIMING_H = 480,
            TIMING_I = 70,
            TIMING_J = 410,
        };

        static constexpr std::uint16_t BUS_ASSERTED{1};
        static constexpr std::uint16_t BUS_RELEASED{0};
        static constexpr std::uint32_t RESOLUTION_HZ{1000000};
        static constexpr std::uint16_t TX_MEM_BLOCK_SYMBOLS{64};
        static constexpr std::uint16_t TX_QUEUE_DEPTH{4};
        static constexpr std::uint16_t MAX_READ_BITS{64};
        static constexpr std::uint16_t RX_MEM_BLOCK_SYMBOLS{(MAX_READ_BITS + 2)};
        static constexpr std::uint16_t RX_MIN_NS{1000};
        static constexpr std::uint16_t RMT_TIMEOUT_MS{1000};
        static constexpr std::uint16_t RX_MARGIN_US{3};
        static constexpr std::uint16_t RX_BUFFER_SIZE{MAX_READ_BITS};
        using RXBuffer = std::array<rmt_symbol_word_t, RX_BUFFER_SIZE>;

        static constexpr rmt_symbol_word_t SYMBOL_0{.duration0 = TIMING_C,
                                                    .level0 = BUS_ASSERTED,
                                                    .duration1 = TIMING_D,
                                                    .level1 = BUS_RELEASED};

        static constexpr rmt_symbol_word_t SYMBOL_1{.duration0 = TIMING_A,
                                                    .level0 = BUS_ASSERTED,
                                                    .duration1 = TIMING_B,
                                                    .level1 = BUS_RELEASED};

        static constexpr rmt_symbol_word_t SYMBOL_RESET{.duration0 = TIMING_H,
                                                        .level0 = BUS_ASSERTED,
                                                        .duration1 = TIMING_I + TIMING_J,
                                                        .level1 = BUS_RELEASED};

        void initialize(gpio_num_t const gpio_num) noexcept;
        void deinitialize() noexcept;

        void initialize_rx(gpio_num_t const gpio_num) noexcept;
        void initialize_tx(gpio_num_t const gpio_num) noexcept;

        void deinitialize_rx() noexcept;
        void deinitialize_tx() noexcept;

        bool initialized{false};

        QueueHandle_t rx_queue{nullptr};

        rmt_channel_handle_t tx_channel{nullptr};
        rmt_channel_handle_t rx_channel{nullptr};
        rmt_encoder_handle_t copy_encoder{nullptr};
        rmt_encoder_handle_t bytes_encoder{nullptr};

        mutable RXBuffer rx_buffer{};
    };

}; // namespace DS18B20

#endif // OW_DEVICE_HPP
