#pragma once

#include <array>
#include <ecrt.h>
#include <string>

#include <gravity/fmt_config.hpp>
#include <gravity/Logger.hpp>

namespace gravity
{
    template <typename T>
    T sdo_read(const uint16_t &slave_position, const uint16_t &index, const uint8_t &subindex)
    {
        size_t result_size = 0;
        uint32_t abort_code = 0;
        size_t size = sizeof(T);
        uint8_t upload_buffer[size];

        const uint16_t master_index = 0;
        ec_master_t *master_ptr = ecrt_open_master(master_index);
        if (master_ptr == nullptr)
        {
            throw std::runtime_error("Read Error: EC_MASTER invalid");
        }

        int result_status = ecrt_master_sdo_upload(
            master_ptr,
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
            auto msg = fmt::format("SDO upload failed for 0x{:x}:0x{:x}, status={}, abort=0x{:x}",
                                   index, subindex, result_status, abort_code);
            throw std::runtime_error(msg);
        }

        if (result_size != size)
        {
            auto msg = fmt::format("SDO size mismatch for 0x{:x}:0x{:x} (expected {}, got {})",
                                   index, subindex, size, result_size);
            throw std::runtime_error(msg);
        }
        T upload_data = from_little_endian_bytes<T>(upload_buffer);
        return upload_data;

        // _log->debug("SDO READ [{} : {:#x} : {:#x}] -> {:#x} == {} == {}",
        //             slave_position, index, subindex, upload_data, upload_data, to_binary_string(upload_data));
    }

    template <typename T>
    void sdo_write(uint16_t slave_position, uint16_t index, uint8_t subindex, T value)
    {
        uint32_t abort_code = 0;
        auto write_data = to_little_endian_bytes(value);

        const uint16_t master_index = 0;
        ec_master_t *master_ptr = ecrt_open_master(master_index);
        if (master_ptr == nullptr)
        {
            throw std::runtime_error("Read Error: EC_MASTER invalid");
        }

        int result = ecrt_master_sdo_download(
            master_ptr,
            slave_position,
            index,
            subindex,
            write_data.data(),
            write_data.size(),
            &abort_code);

        if (result != 0)
        {
            auto msg = fmt::format(
                "SDO write failed: slave={}, index=0x{:04x}, subindex=0x{:02x}, abort=0x{:08x}, status={}",
                slave_position, index, subindex, abort_code, result);
            throw std::runtime_error(msg);
        }

        // _log->debug("SDO WRITE [{} : {:#x} : {:#x}] -> {:#x} == {} == {}", slave_position, index, subindex, value,
        //             value, to_binary_string(value));
    }

}
