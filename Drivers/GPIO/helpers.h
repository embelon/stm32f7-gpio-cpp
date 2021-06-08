
#pragma once

#include <stdint.h>

template <uint32_t N, uint8_t bit>
struct _trailing_zeros
{
	enum { value = (N & 1ul) ? (0ul) : (1ul + _trailing_zeros< (N >> 1), bit - 1 >::value) };
};

template <uint32_t N>
struct _trailing_zeros<N, 0>
{
	enum { value = (N & 1ul) ? (0ul) : (1ul) };
};

template <uint32_t N>
struct TrailingZeros
{
	enum { value = _trailing_zeros<N, 31>::value };
};

template <uint32_t N, uint8_t bit>
struct _leading_zeros
{
	enum { value = (N & 0x80000000) ? (0ul) : (1ul + _leading_zeros< (N << 1), bit - 1 >::value) };
};

template <uint32_t N>
struct _leading_zeros<N, 0>
{
	enum { value = (N & 0x80000000) ? (0ul) : (1ul) };
};

template <uint32_t N>
struct LeadingZeros
{
	enum { value = _leading_zeros<N, 31>::value };
};

template <uint8_t bit, uint8_t ...bits>
struct BitNumsToMask
{
	static const uint16_t mask = (1u << bit) | BitNumsToMask<bits...>::mask;
};

template <uint8_t bit>
struct BitNumsToMask<bit>
{
	static const uint16_t mask = (1u << bit);
};
