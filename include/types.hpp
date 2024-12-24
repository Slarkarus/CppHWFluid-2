#pragma once
#include <bits/stdc++.h>

template <typename type, size_t k, typename bigger_type>
struct TemplateFixed
{
    type v;
    static constexpr size_t N = sizeof(type) * 8;
    static constexpr size_t K = k;

    constexpr TemplateFixed(int v_) : TemplateFixed(static_cast<long long>(v_)) {}
    constexpr TemplateFixed(float f_) : v(f_ * (((type)1) << K)) {}
    constexpr TemplateFixed(double f_) : v(f_ * (((type)1) << K)) {}
    constexpr TemplateFixed(long long v_) : v(((type)v_) << K) {}
    constexpr TemplateFixed() : v(0) {}
    
    template <typename type_, size_t k_, typename bigger_type_>
    constexpr TemplateFixed(const TemplateFixed<type_, k_, bigger_type_> &other) {
        auto v_ = other.v;
        if (other.K > K) {
            v_ >>= (other.K - K);
        }

        v = static_cast<type>(v_);
        if (K > other.K) {
            v <<= (K - other.K);
        }
    }

    static constexpr TemplateFixed from_raw(type x)
    {
        TemplateFixed ret;
        ret.v = x;
        return ret;
    }

    auto operator<=>(const TemplateFixed &) const = default;
    bool operator==(const TemplateFixed &) const = default;

    friend TemplateFixed operator+(TemplateFixed a, TemplateFixed b) { return TemplateFixed::from_raw(a.v + b.v); };
    friend TemplateFixed operator-(TemplateFixed a, TemplateFixed b) { return TemplateFixed::from_raw(a.v - b.v); };
    friend TemplateFixed operator*(TemplateFixed a, TemplateFixed b) { return TemplateFixed::from_raw(((bigger_type)a.v * b.v) >> K); };
    friend TemplateFixed operator/(TemplateFixed a, TemplateFixed b) { return TemplateFixed::from_raw(((bigger_type)a.v << K) / b.v); };

    friend TemplateFixed &operator+=(TemplateFixed &a, TemplateFixed b) { return a = a + b; };
    friend TemplateFixed &operator-=(TemplateFixed &a, TemplateFixed b) { return a = a - b; };
    friend TemplateFixed &operator*=(TemplateFixed &a, TemplateFixed b) { return a = a * b; };
    friend TemplateFixed &operator/=(TemplateFixed &a, TemplateFixed b) { return a = a / b; };

    friend TemplateFixed operator-(TemplateFixed a) { return TemplateFixed::from_raw(-a.v); };

    friend TemplateFixed abs(TemplateFixed a)
    {
        if (a.v < 0)
            a.v = -a.v;
        return a;
    }
    friend TemplateFixed &operator<<(std::ostream &output, const TemplateFixed &x)
    {
        return output << x.v / (double)((type)1 << K);
    }
    friend std::istream &operator>>(std::istream &input, TemplateFixed &x)
    {
        double v_;
        input >> v_;
        x = TemplateFixed(v_);
        return input;
    }

    explicit operator double() const { return (double)v / (double)((type)1 << K); }
};

template <size_t N>
struct FixedType
{
    using type = void;
    using bigger_type = void;
};

template <>
struct FixedType<8>
{
    using type = int8_t;
    using bigger_type = int16_t;
};

template <>
struct FixedType<16>
{
    using type = int16_t;
    using bigger_type = int32_t;
};

template <>
struct FixedType<32>
{
    using type = int32_t;
    using bigger_type = int64_t;
};

template <>
struct FixedType<64>
{
    using type = int64_t;
    using bigger_type = __int128_t;
};

template <size_t N>
struct FastFixedType
{
    using type = void;
    using bigger_type = void;
};

template <>
struct FastFixedType<8>
{
    using type = int_fast8_t;
    using bigger_type = int_fast16_t;
};

template <>
struct FastFixedType<16>
{
    using type = int_fast16_t;
    using bigger_type = int_fast32_t;
};

template <>
struct FastFixedType<32>
{
    using type = int_fast32_t;
    using bigger_type = int_fast64_t;
};

template <>
struct FastFixedType<64>
{
    using type = int_fast64_t;
    using bigger_type = __int128_t;
};

template <size_t N>
struct TrueFastFixedType
{
    static constexpr size_t get_size()
    {
        if (N > 32)
        {
            return 64;
        }
        if (N > 16)
        {
            return 32;
        }
        if (N > 8)
        {
            return 16;
        }
        return 8;
    }
    using type = typename FastFixedType<get_size()>::type;
    using bigger_type = typename FastFixedType<get_size()>::type;
};

template <size_t N = 64, size_t K = 32>
using Fixed = TemplateFixed<typename FixedType<N>::type, K, typename FixedType<N>::bigger_type>;

template <size_t N = 32, size_t K = 16>
using FastFixed = TemplateFixed<typename TrueFastFixedType<N>::type, K, typename TrueFastFixedType<N>::bigger_type>;
