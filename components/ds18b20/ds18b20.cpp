#include "ds18b20.hpp"
#include <utility>

using Raw = DS18B20::DS18B20::Raw;
using Scaled = DS18B20::DS18B20::Scaled;

namespace DS18B20 {

    Raw DS18B20::get_temperature_raw() const noexcept
    {
        this->ow_device.reset();
        this->ow_device.write16(static_cast<std::uint16_t>(ROM::SKIP_ROM));
        this->ow_device.write16(static_cast<std::uint16_t>(MEM::CONVERT_T));

        while (this->ow_device.read8() == 0) {
            ;
        }

        this->ow_device.reset();
        this->ow_device.write16(static_cast<std::uint16_t>(ROM::MATCH_ROM));
        this->ow_device.write64(static_cast<std::uint16_t>(this->address));
        this->ow_device.write16(static_cast<std::uint16_t>(MEM::READ_SCRATCHPAD));

        return static_cast<Raw>(this->ow_device.read8()) | static_cast<Raw>(this->ow_device.read8() << 8);
    }

    Scaled DS18B20::get_temperature_scaled() const noexcept
    {
        return static_cast<Scaled>(this->get_temperature_raw()) / 16.0F;
    }

}; // namespace DS18B20