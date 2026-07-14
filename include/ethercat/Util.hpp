#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <boost/describe.hpp>
#include <boost/mp11.hpp>
#include <iostream>
#include <ecrt.h>
#include <gravity/Transforms.hpp>

namespace gravity
{

    template <typename T>
    bool get_bit(T control_word, int pos)
    {
        T mask = static_cast<T>(1) << pos;
        return (control_word & mask) != 0;
    }

    template <typename T>
    T set_bit(T control_word, int pos, bool value)
    {
        T mask = 1 << pos;
        if (value)
        {
            control_word |= mask;
        }
        else
        {
            control_word &= ~mask;
        }
        return control_word;
    }

    template <typename T>
    T from_little_endian_bytes(const uint8_t *data)
    {
        T value = 0;
        for (size_t i = 0; i < sizeof(T); ++i)
        {
            value |= static_cast<T>(data[i]) << (8 * i);
        }
        return value;
    }

    template <typename T>
    std::array<uint8_t, sizeof(T)> to_little_endian_bytes(T value)
    {
        std::array<uint8_t, sizeof(T)> bytes{};
        for (size_t i = 0; i < sizeof(T); ++i)
        {
            bytes[i] = static_cast<uint8_t>((value >> (i * 8)) & 0xFF);
        }
        return bytes;
    }

    template <typename T>
    std::string binary_str(T value)
    {
        std::string bits;
        for (int i = sizeof(T) * 8 - 1; i >= 0; --i)
        {
            bits += ((value >> i) & 1) ? '1' : '0';
            if (i % 4 == 0 && i != 0)
                bits += ' ';
        }
        return bits;
    }

    template <typename T>
    inline std::string hex_str(const T val, int width = sizeof(T) * 2)
    {
        static_assert(std::is_integral<T>::value, "hex_str requires an integral type");
        std::ostringstream oss;
        oss << "0x"
            << std::uppercase << std::hex
            << std::setw(width) << std::setfill('0')
            << static_cast<std::make_unsigned_t<T>>(val);
        return oss.str();
    }

    inline std::string al_state_str(const uint8_t &al_state)
    {
        return util::tf::enum_str<ec_al_state_t>(static_cast<ec_al_state_t>(al_state));
    }

    template <typename T>
    void print_boost_structs(const T &obj)
    {
        using namespace boost::describe;
        using Bases = describe_bases<T, mod_public | mod_protected>;
        using Members = describe_members<T, mod_public | mod_protected>;

        // walk base classes first
        boost::mp11::mp_for_each<Bases>([&](auto B)
                                        {
        using BaseType = typename decltype(B)::type;
        print_boost_structs(static_cast<const BaseType&>(obj)); });

        boost::mp11::mp_for_each<Members>(
            [&](auto D)
            { std::cout << std::left << std::setw(20) << D.name
                        << " = " << std::boolalpha << (obj.*D.pointer) << "\n"; });
    }

} // namespace gravity
