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
        RX_PDO, // write
        TX_PDO, // read only
        NONE
    };
    // remove string varaibles
    class DictionaryEntity
    {
    protected:
        const uint8_t position;
        const uint16_t index;
        const uint8_t subindex;
        const MappingType mapping_type;
        uint32_t offset{};
        uint64_t sequence{};

    public:
        DictionaryEntity(
            const uint8_t position,
            const uint16_t index,
            const uint8_t subindex,
            const MappingType mapping_type)
            : position(position),
              index(index),
              subindex(subindex),
              mapping_type(mapping_type)
        {
        }

        uint16_t getIndex() const noexcept { return index; }
        uint8_t getSubindex() const noexcept { return subindex; }
        uint8_t getPosition() const noexcept { return position; }
        MappingType getMappingType() const noexcept { return mapping_type; }
        uint32_t &getOffset() noexcept { return offset; } // return a ref
        uint64_t &getSequence() noexcept { return sequence; }

        virtual ~DictionaryEntity() = default;
        virtual const uint32_t getEntryKey() const = 0;
        virtual const size_t get_size_bytes() const = 0;

        virtual void write_to_domain(uint8_t *domain, const uint64_t _s) = 0;
        virtual void read_from_domain(uint8_t *domain, const uint64_t _s) = 0;

        // BOOST_DESCRIBE_CLASS(
        //     DictionaryEntity, (),
        //     (getIndex, getSubindex, getPosition, getMappingType, getOffset), // public
        //     (),                                                              // protected funcs (none)
        //     (position, index, subindex, mapping_type, offset))               // protected data
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
        std::atomic<T> value{};

        void checkRange(T new_value) const
        {
            if (mapping_type != MappingType::RX_PDO)
            {
                auto err = fmt::format("Attempted write to read-only entry: 0x{:04x}", index);
                throw std::runtime_error(err);
            }
            if (new_value < min_value || new_value > max_value)
            {
                auto err = fmt::format("Value out of range for: 0x{:04x}", index);
                throw std::out_of_range(err);
            }
        }

    public:
        DataObject(
            const uint8_t position,
            const uint16_t index,
            const uint8_t subindex,
            T min_value,
            T max_value,
            T default_value,
            const MappingType mapping_type)

            : DictionaryEntity(position, index, subindex, mapping_type),
              min_value(min_value),
              max_value(max_value),
              default_value(default_value),
              value(default_value)
        {
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

        T read_pdo() const { return value.load(std::memory_order_acquire); }

        void write_pdo(const T &_value)
        {
            checkRange(_value);
            value.store(_value, std::memory_order_release);
        }

        T read_sdo() const
        {
            T _value{};
            int resp = sdo_read<T>(position, index, subindex, _value);
            return _value;
        }

        int write_sdo(const T _value) const
        {
            checkRange(_value);
            return sdo_write<T>(position, index, subindex, _value);
        }

        size_t const get_size_bytes() const override
        {
            return sizeof(T);
        }

        void read_from_domain(uint8_t *domain, const uint64_t _s) override
        {
            if (domain == nullptr)
            {
                throw std::runtime_error("Domain ptr null");
            }

            sequence = _s;
            const uint8_t *p = domain + offset;
            T temp{};

            if constexpr (std::is_same_v<T, uint8_t>)
                temp = EC_READ_U8(p);
            else if constexpr (std::is_same_v<T, int8_t>)
                temp = EC_READ_S8(p);
            else if constexpr (std::is_same_v<T, uint16_t>)
                temp = EC_READ_U16(p);
            else if constexpr (std::is_same_v<T, int16_t>)
                temp = EC_READ_S16(p);
            else if constexpr (std::is_same_v<T, uint32_t>)
                temp = EC_READ_U32(p);
            else if constexpr (std::is_same_v<T, int32_t>)
                temp = EC_READ_S32(p);
            else if constexpr (std::is_same_v<T, uint64_t>)
                temp = EC_READ_U64(p);
            else if constexpr (std::is_same_v<T, int64_t>)
                temp = EC_READ_S64(p);
            else
                static_assert(!sizeof(T), "Unsupported type for read_from_domain");

            value.store(temp, std::memory_order_release);
        }
        void write_to_domain(uint8_t *domain, const uint64_t _s) override
        {
            if (domain == nullptr)
            {
                throw std::runtime_error("Domain ptr null");
            }

            sequence = _s;
            uint8_t *p = domain + offset;
            const T temp = value.load(std::memory_order_acquire);

            if constexpr (std::is_same_v<T, uint8_t>)
                EC_WRITE_U8(p, temp);
            else if constexpr (std::is_same_v<T, int8_t>)
                EC_WRITE_S8(p, temp);
            else if constexpr (std::is_same_v<T, uint16_t>)
                EC_WRITE_U16(p, temp);
            else if constexpr (std::is_same_v<T, int16_t>)
                EC_WRITE_S16(p, temp);
            else if constexpr (std::is_same_v<T, uint32_t>)
                EC_WRITE_U32(p, temp);
            else if constexpr (std::is_same_v<T, int32_t>)
                EC_WRITE_S32(p, temp);
            else if constexpr (std::is_same_v<T, uint64_t>)
                EC_WRITE_U64(p, temp);
            else if constexpr (std::is_same_v<T, int64_t>)
                EC_WRITE_S64(p, temp);
            else
                static_assert(!sizeof(T), "Unsupported type for write_to_domain");
        }
        // BOOST_DESCRIBE_CLASS(
        //     DataObject, (DictionaryEntity),
        //     (getMinValue, getMaxValue, getDefaultValue, getEntryKey), // public
        //     (),
        //     (min_value, max_value, default_value, value))
    };

} // namespace gravity::mover