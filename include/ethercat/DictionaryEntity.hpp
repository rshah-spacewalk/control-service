#pragma once
#include <ecrt.h>
#include <exception>
#include <cstdint>
#include <string>
#include <stdexcept>
#include "ethercat/SdoEx.hpp"
#include <gravity/Transforms.hpp>

namespace gravity
{
    enum class MappingType
    {
        RX_PDO,
        TX_PDO,
        NONE
    };
    // remove string varaibles
    class DictionaryEntity
    {
    protected:
        const std::string name;
        const uint8_t position;
        const uint16_t index;
        const uint8_t subindex;
        const std::string uom;
        const MappingType mapping_type;
        uint32_t offset{};

    public:
        DictionaryEntity(
            std::string name,
            const uint8_t position,
            const uint16_t index,
            const uint8_t subindex,
            std::string uom,
            const MappingType mapping_type) : name(std::move(name)),
                                              position(position),
                                              index(index),
                                              subindex(subindex),
                                              uom(std::move(uom)),
                                              mapping_type(mapping_type)
        {
        }

        const std::string &getName() const noexcept { return name; }
        uint16_t getIndex() const noexcept { return index; }
        uint8_t getSubindex() const noexcept { return subindex; }
        uint8_t getPosition() const noexcept { return position; }
        const std::string &getUom() const noexcept { return uom; }
        MappingType getMappingType() const noexcept { return mapping_type; }

        uint32_t &getOffset() noexcept { return offset; } // return a ref

        virtual ~DictionaryEntity() = default;

        virtual const uint32_t getEntryKey() const = 0;

        virtual const size_t get_size_bytes() const = 0;

        virtual void read_from_domain(const uint8_t *domain) = 0;
        virtual void write_to_domain(uint8_t *domain) const = 0;
    };

    template <typename T>
    class DataObject : public DictionaryEntity
    {
        static_assert(
            std::is_same_v<T, uint8_t> || std::is_same_v<T, int8_t> ||
                std::is_same_v<T, uint16_t> || std::is_same_v<T, int16_t> ||
                std::is_same_v<T, uint32_t> || std::is_same_v<T, int32_t> ||
                std::is_same_v<T, uint64_t> || std::is_same_v<T, int64_t>,
            "DictionaryEntry<T>: T must be one of the 8/16/32/64-bit signed or "
            "unsigned integer types supported by the EtherCAT domain macros");

        const T min_value;
        const T max_value;
        const T default_value;
        T value{};

        void checkRange(T new_value) const
        {
            if (mapping_type != MappingType::RX_PDO)
            {
                throw std::runtime_error("Attempted write to read-only entry: " + name);
            }
            if (new_value < min_value || new_value > max_value)
            {
                throw std::out_of_range("Value out of range for: " + name);
            }
        }

    public:
        DataObject(
            const std::string &name,
            const uint8_t position,
            const uint16_t index,
            const uint8_t subindex,
            T min_value,
            T max_value,
            T default_value,
            const std::string &uom,
            const MappingType mapping_type)

            : DictionaryEntity(name, position, index, subindex, uom, mapping_type),
              min_value(min_value),
              max_value(max_value),
              default_value(default_value),
              value(default_value)
        {
            // spdlog::info(
            //     "[{:<32}] [{} : 0x{:04X} : 0x{:02X}] [{:<12} -> {:<12}: {:<12}] {}",
            //     getName(), getPosition(), getIndex(), getSubindex(),
            //     getMinValue(), getMaxValue(), getDefaultValue(),
            //     util::tf::enum_str(getMappingType()));
        }

        // Getter & Setters --------
        const T &getMinValue() const noexcept { return min_value; }
        const T &getMaxValue() const noexcept { return max_value; }
        const T &getDefaultValue() const noexcept { return default_value; }
        const uint32_t getEntryKey() const override
        {
            return (static_cast<uint32_t>(index) << 16) |
                   (static_cast<uint32_t>(subindex) << 8) |
                   static_cast<uint32_t>(sizeof(T) * 8);
        }

        // READ & WRITE -------

        T read_pdo() const { return value; }

        void write_pdo(const T &new_value)
        {
            value = new_value;
            // spdlog::info("SDO write [{} : 0x{:04X} : 0x{:02X}]", position, index, subindex);
        }

        T read_sdo()
        {
            // spdlog::info("SDO read [{} : 0x{:04X} : 0x{:02X}] []", position, index, subindex);
            int resp = sdo_read<T>(position, index, subindex, value);
            return value;
        }

        int write_sdo(T new_value)
        {
            checkRange(new_value);
            return sdo_write<T>(position, index, subindex, new_value);
        }

        size_t const get_size_bytes() const override
        {
            return sizeof(T);
        }

        void read_from_domain(const uint8_t *domain) override
        {
            if (domain != nullptr)
            {
                const uint8_t *p = domain + offset;
                // 8 bit
                if constexpr (std::is_same_v<T, uint8_t>)
                    value = EC_READ_U8(p);
                else if constexpr (std::is_same_v<T, int8_t>)
                    value = EC_READ_S8(p);
                // 16 bit
                else if constexpr (std::is_same_v<T, uint16_t>)
                    value = EC_READ_U16(p);
                else if constexpr (std::is_same_v<T, int16_t>)
                    value = EC_READ_S16(p);
                // 32 bit
                else if constexpr (std::is_same_v<T, uint32_t>)
                    value = EC_READ_U32(p);
                else if constexpr (std::is_same_v<T, int32_t>)
                    value = EC_READ_S32(p);
                // 64 bit
                else if constexpr (std::is_same_v<T, uint64_t>)
                    value = EC_READ_U64(p);
                else if constexpr (std::is_same_v<T, int64_t>)
                    value = EC_READ_S64(p);
            }
            else
            {
                throw std::runtime_error("Domain ptr null");
            }
        }

        void write_to_domain(uint8_t *domain) const override
        {
            if (domain != nullptr)
            {
                uint8_t *p = domain + offset;
                // 8 bit
                if constexpr (std::is_same_v<T, uint8_t>)
                    EC_WRITE_U8(p, value);
                else if constexpr (std::is_same_v<T, int8_t>)
                    EC_WRITE_S8(p, value);
                // 16 bit
                else if constexpr (std::is_same_v<T, uint16_t>)
                    EC_WRITE_U16(p, value);
                else if constexpr (std::is_same_v<T, int16_t>)
                    EC_WRITE_S16(p, value);
                // 32 bit
                else if constexpr (std::is_same_v<T, uint32_t>)
                    EC_WRITE_U32(p, value);
                else if constexpr (std::is_same_v<T, int32_t>)
                    EC_WRITE_S32(p, value);
                // 64 bit
                else if constexpr (std::is_same_v<T, uint64_t>)
                    EC_WRITE_U64(p, value);
                else if constexpr (std::is_same_v<T, int64_t>)
                    EC_WRITE_S64(p, value);
            }
            else
            {
                throw std::runtime_error("Domain ptr null");
            }
        }
    };

} // namespace gravity::mover