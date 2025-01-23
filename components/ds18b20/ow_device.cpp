#include "ow_device.hpp"
#include "driver/rmt_encoder.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "hal/rmt_types.h"
#include "soc/clk_tree_defs.h"
#include <array>
#include <cstdint>
#include <span>

static bool IRAM_ATTR rx_event_callback(rmt_channel_handle_t channel,
                                        rmt_rx_done_event_data_t const* data,
                                        void* ctx) noexcept
{
    BaseType_t task_woken{pdFALSE};
    xQueueSendFromISR(static_cast<QueueHandle_t>(ctx), data, &task_woken);
    return task_woken == pdTRUE;
}

namespace DS18B20 {

    OWDevice::OWDevice(gpio_num_t const gpio_num, QueueHandle_t const rx_queue) noexcept : rx_queue{rx_queue}
    {
        this->initialize(gpio_num);
    }

    OWDevice::~OWDevice() noexcept
    {
        this->deinitialize();
    }

    void OWDevice::initialize(const gpio_num_t gpio_num) noexcept
    {
        this->initialize_rx(gpio_num);
        this->initialize_tx(gpio_num);
        this->initialized = true;
    }

    void OWDevice::deinitialize() noexcept
    {
        this->deinitialize_rx();
        this->deinitialize_tx();
        this->initialized = false;
    }

    void OWDevice::write16(std::uint16_t const write_data) noexcept
    {
        if (!this->initialized) {
            return;
        }

        rmt_transmit_config_t const tx_config{.loop_count = 0,
                                              .flags = {.eot_level = BUS_RELEASED, .queue_nonblocking = 0}};
        rmt_transmit(this->tx_channel, this->bytes_encoder, &write_data, 1, &tx_config);

        rmt_tx_wait_all_done(this->tx_channel, RMT_TIMEOUT_MS);
    }

    void OWDevice::write64(std::uint64_t const write_data) noexcept
    {
        if (!this->initialized) {
            return;
        }

        rmt_transmit_config_t const tx_config{.loop_count = 0,
                                              .flags = {.eot_level = BUS_RELEASED, .queue_nonblocking = 0}};
        rmt_transmit(this->tx_channel, this->bytes_encoder, &write_data, 8, &tx_config);

        rmt_tx_wait_all_done(this->tx_channel, RMT_TIMEOUT_MS);
    }

    std::uint8_t OWDevice::read8() noexcept
    {
        if (!this->initialized) {
            std::unreachable();
        }

        rmt_receive_config_t const rx_config{.signal_range_min_ns = RX_MIN_NS,
                                             .signal_range_max_ns = (TIMING_H + TIMING_I) * 1000};
        rmt_receive(this->rx_channel, this->rx_buffer.data(), this->rx_buffer.size(), &rx_config);

        this->write16(0xFF);

        rmt_rx_done_event_data_t event;
            xQueueReceive(this->rx_queue, &event, pdMS_TO_TICKS(RMT_TIMEOUT_MS);

            return parse_symbols(Symbols{event.received_symbols, event.num_symbols});
    }

    void OWDevice::reset() noexcept
    {
        if (!this->initialized) {
            return;
        }

        rmt_receive_config_t const rx_config{.signal_range_min_ns = RX_MIN_NS,
                                             .signal_range_max_ns = (TIMING_H + TIMING_I) * 1000};
        rmt_receive(this->rx_channel, this->rx_buffer.data(), this->rx_buffer.size(), &rx_config);

        rmt_transmit_config_t const tx_config{.loop_count = 0,
                                              .flags = {.eot_level = BUS_RELEASED, .queue_nonblocking = 0}};
        rmt_transmit(this->tx_channel, this->copy_encoder, &SYMBOL_RESET, sizeof(SYMBOL_RESET), &tx_config);

        rmt_rx_done_event_data_t event;
        xQueueReceive(this->rx_queue, &event, pdMS_TO_TICKS(RMT_TIMEOUT_MS));

        rmt_tx_wait_all_done(this->tx_channel, RMT_TIMEOUT_MS);
    }

    std::uint8_t OWDevice::parse_symbols(Symbols const symbols) noexcept
    {
        std::uint8_t ret{};
        for (auto i{0}; i < symbols.size(); ++i) {
            if (i < 8) {
                if (symbols[i].duration0 <= TIMING_A + RX_MARGIN_US &&
                    (symbols[i].duration1 == 0 || symbols[i].duration1 >= TIMING_E)) {
                    ret |= (1U << i);
                } else if (symbols[i].duration0 >= TIMING_A + TIMING_E) {
                    ret &= ~(1U << i);
                }
            }
        }
        return ret;
    }

    void OWDevice::initialize_rx(gpio_num_t const gpio_num) noexcept
    {
        rmt_rx_channel_config_t const rx_channel_config{
            .gpio_num = gpio_num,
            .clk_src = RMT_CLK_SRC_APB,
            .resolution_hz = RESOLUTION_HZ,
            .mem_block_symbols = RX_MEM_BLOCK_SYMBOLS,
            .flags = {.invert_in = false, .with_dma = false, .io_loop_back = false},
            .intr_priority = 1};
        rmt_new_rx_channel(&rx_channel_config, &this->rx_channel);

        rmt_rx_event_callbacks_t const rx_callbacks{.on_recv_done = &rx_event_callback};
        rmt_rx_register_event_callbacks(this->rx_channel, &rx_callbacks, this->rx_queue);

        rmt_enable(this->rx_channel);
    }

    void OWDevice::initialize_tx(gpio_num_t const gpio_num) noexcept
    {
        rmt_tx_channel_config_t const tx_channel_config{
            .gpio_num = gpio_num,
            .clk_src = RMT_CLK_SRC_APB,
            .resolution_hz = RESOLUTION_HZ,
            .mem_block_symbols = TX_MEM_BLOCK_SYMBOLS,
            .trans_queue_depth = TX_QUEUE_DEPTH,
            .intr_priority = 1,
            .flags = {.invert_out = true, .with_dma = false, .io_loop_back = true, .io_od_mode = true}};
        rmt_new_tx_channel(&tx_channel_config, &this->tx_channel);

        rmt_enable(this->tx_channel);

        rmt_copy_encoder_config_t const copy_encoder_config{};
        rmt_new_copy_encoder(&copy_encoder_config, &this->copy_encoder);

        rmt_bytes_encoder_config_t const bytes_encoder_config{.bit0 = SYMBOL_0,
                                                              .bit1 = SYMBOL_1,
                                                              .flags = {.msb_first = false}};
        rmt_new_bytes_encoder(&bytes_encoder_config, &this->bytes_encoder);
    }

    void OWDevice::deinitialize_rx() noexcept
    {
        rmt_disable(this->rx_channel);

        rmt_del_channel(this->rx_channel);
    }

    void OWDevice::deinitialize_tx() noexcept
    {
        rmt_disable(this->tx_channel);
        rmt_del_channel(this->tx_channel);

        rmt_del_encoder(this->copy_encoder);

        rmt_del_encoder(this->bytes_encoder);
    }

}; // namespace DS18B20