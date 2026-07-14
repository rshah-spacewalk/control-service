#pragma once

#include <array>
#include <ecrt.h>
#include <string>

#include "ethercat/Master.hpp"
#include <gravity/fmt_config.hpp>
#include <gravity/Logger.hpp>

namespace gravity
{
    template <typename T>
    int sdo_read(const uint16_t slave_position, const uint16_t index, const uint8_t subindex, T &value)
    {
        // spdlog::info("SDO READ [{} : 0x{:04X} : 0x{:02X}]", slave_position, index, subindex);

        size_t result_size = 0;
        uint32_t abort_code = 0;
        size_t size = sizeof(T);
        uint8_t upload_buffer[size];

        int result_status = ecrt_master_sdo_upload(
            get_master_ptr(),
            slave_position,
            index,
            subindex,
            upload_buffer, // Pointer to data buffer
            size,          // Maximum expected size of data
            &result_size,  // Pointer to store actual size uploaded
            &abort_code    // Pointer to store abort code on failure
        );

        if (result_status != 0)
        {
            auto err = fmt::format("SDO upload failed for 0x{:x}:0x{:x}, status={}, abort=0x{:x}",
                                   index, subindex, result_status, abort_code);
            std::cout << err << std::endl;
            throw std::runtime_error(err);
        }

        if (result_size != size)
        {
            auto err = fmt::format("SDO size mismatch for 0x{:x}:0x{:x} (expected {}, got {})",
                                   index, subindex, size, result_size);
            std::cout << err << std::endl;
            throw std::runtime_error(err);
        }

        // spdlog::debug("SDO READ [{} : {:#x} : {:#x}] -> {:#x} == {} == {}",
        //               slave_position, index, subindex, upload_data, upload_data, to_binary_string(upload_data));

        value = from_little_endian_bytes<T>(upload_buffer);
        return result_status;
    }

    template <typename T>
    int sdo_write(uint16_t slave_position, uint16_t index, uint8_t subindex, T value)
    {
        // spdlog::info("SDO WRITE [{} : 0x{:04X} : 0x{:02X}]", slave_position, index, subindex);

        uint32_t abort_code = 0;
        auto write_data = to_little_endian_bytes(value);

        int result = ecrt_master_sdo_download(
            get_master_ptr(),
            slave_position,
            index,
            subindex,
            write_data.data(),
            write_data.size(),
            &abort_code);

        if (result != 0)
        {
            auto err = fmt::format(
                "SDO write failed: slave={}, index=0x{:04x}, subindex=0x{:02x}, abort=0x{:08x}, status={}",
                slave_position, index, subindex, abort_code, result);
            std::cout << err << std::endl;
            throw std::runtime_error(err);
        }

        spdlog::info("[{}] SDO WRITE [{} : {:#x} : {:#x}] -> {:#x} == {} == {}",
                     (result ? "Fail" : "Success"), slave_position, index, subindex, value,
                     value, binary_str(value));
        return result;
    }

}
