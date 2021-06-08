
#pragma once

#include <stdint.h>

extern "C"
{
	#include "stm32f746xx.h"
}

#include "helpers.h"

enum class GPIO_Port : uint32_t
{
	A = GPIOA_BASE,
	B = GPIOB_BASE,
	C = GPIOC_BASE,
	D = GPIOD_BASE,
	E = GPIOE_BASE,
	F = GPIOF_BASE,
	G = GPIOG_BASE,
	H = GPIOH_BASE,
	I = GPIOI_BASE,
	J = GPIOJ_BASE,
	K = GPIOK_BASE
};

enum class GPIO_Mode : uint32_t
{
		input = 0,
		output = 1,
		alternate = 2,
		analog = 3	
};

enum class GPIO_Driver
{
	push_pull = 0,
	open_drain
};

enum class GPIO_Speed : uint32_t
{
	low = 0,
	medium = 1,
	high = 2,
	very_high = 3
};

enum class GPIO_PullUpDown : uint32_t
{
	floating = 0,
	up = 1,
	down = 2
};

template <GPIO_Mode mode, GPIO_Port port, uint16_t bit>
class GPIO_bit;

template <GPIO_Port port, uint16_t bit>
class GPIO_bit_helper
{
private:
  static_assert( (bit < 16), "Invalid GPIO bit number, outside [0-15] range." );	

	GPIO_bit_helper() = delete;	
		
	static volatile GPIO_TypeDef * const reg()
	{
		return reinterpret_cast<GPIO_TypeDef*>(port);
	}
		
	constexpr static uint32_t portShift()
	{
		return (static_cast<uint32_t>(port) - static_cast<uint32_t>(GPIO_Port::A)) / (static_cast<uint32_t>(GPIO_Port::B) - static_cast<uint32_t>(GPIO_Port::A));
	}
	
	static inline void init(GPIO_Mode mode, GPIO_Driver driver, GPIO_Speed speed, GPIO_PullUpDown pu_pd)
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN_Msk << portShift();		
		
		reg()->PUPDR = (reg()->PUPDR & ~(GPIO_PUPDR_PUPDR0 << (2 * bit))) | (static_cast<uint32_t>(pu_pd) << (2* bit));
		reg()->OSPEEDR = (reg()->OSPEEDR & ~(GPIO_OSPEEDR_OSPEEDR0 << (2 * bit))) | (static_cast<uint32_t>(speed) << (2 * bit));
		reg()->OTYPER = (reg()->OTYPER & ~(GPIO_OTYPER_OT0 << bit)) | (static_cast<uint32_t>(driver) << bit);
		reg()->MODER = (reg()->MODER & ~(GPIO_MODER_MODER0 << (2 * bit))) | (static_cast<uint32_t>(mode) << (2 * bit));
	}
	
  friend class GPIO_bit<GPIO_Mode::input, port, bit>;
	friend class GPIO_bit<GPIO_Mode::analog, port, bit>;
	friend class GPIO_bit<GPIO_Mode::output, port, bit>;
	friend class GPIO_bit<GPIO_Mode::alternate, port, bit>;
};

// Default case => GPIO_Mode::input
template <GPIO_Mode mode, GPIO_Port port, uint16_t bit>
class GPIO_bit
{
public:
	GPIO_bit(GPIO_PullUpDown pu_pd)
	{
		init(pu_pd);
	}

	static inline void init(GPIO_PullUpDown pu_pd)
	{
		GPIO_bit_helper<port, bit>::init(mode, GPIO_Driver::open_drain, GPIO_Speed::low, pu_pd);
	}
	
	operator bool()
	{
		return ( GPIO_bit_helper<port, bit>::reg()->IDR & (GPIO_IDR_ID0 << bit) ) ? 1 : 0;
	}
};

template <GPIO_Port port, uint16_t bit>
class GPIO_bit<GPIO_Mode::analog, port, bit>
{
public:
	GPIO_bit()
	{
		init();
	}

	static inline void init()
	{
		GPIO_bit_helper<port, bit>::init(GPIO_Mode::analog, GPIO_Driver::open_drain, GPIO_Speed::low, GPIO_PullUpDown::floating);
	}
};

template <GPIO_Port port, uint16_t bit>
class GPIO_bit<GPIO_Mode::output, port, bit>
{
public:
	GPIO_bit(GPIO_Driver driver, GPIO_Speed speed, GPIO_PullUpDown pu_pd, bool value)
	{
		init(driver, speed, pu_pd, value);
	}

	static inline void init(GPIO_Driver driver, GPIO_Speed speed, GPIO_PullUpDown pu_pd, bool value)
	{
		GPIO_bit_helper<port, bit>::init(GPIO_Mode::output, driver, speed, pu_pd);

		GPIO_bit_helper<port, bit>::reg()->BSRR = value ? (GPIO_BSRR_BS0 << bit) : (GPIO_BSRR_BR0 << bit);
	}

	void operator=(bool value)
	{
		GPIO_bit_helper<port, bit>::reg()->BSRR = value ? (GPIO_BSRR_BS0 << bit) : (GPIO_BSRR_BR0 << bit);		
	}

	operator bool()
	{
		return ( GPIO_bit_helper<port, bit>::reg()->IDR & (GPIO_IDR_ID0 << bit) ? 1 : 0 );
	}

	inline void flip()
	{		
		operator=( GPIO_bit_helper<port, bit>::reg()->ODR & (GPIO_ODR_OD0 << bit) ? 0 : 1 );
	}

	bool outState()
	{
		return ( GPIO_bit_helper<port, bit>::reg()->ODR & (GPIO_ODR_OD0 << bit) ? 1 : 0 );
	}
};

template <GPIO_Port port, uint16_t bit>
class GPIO_bit<GPIO_Mode::alternate, port, bit>
{
public:
	GPIO_bit(GPIO_Driver driver, GPIO_Speed speed, GPIO_PullUpDown pu_pd, uint8_t function)
	{
		init(driver, speed, pu_pd, function);
	}
	
	static inline void init(GPIO_Driver driver, GPIO_Speed speed, GPIO_PullUpDown pu_pd, uint32_t function)
	{
		GPIO_bit_helper<port, bit>::init(GPIO_Mode::alternate, driver, speed, pu_pd);

		volatile GPIO_TypeDef * const reg = GPIO_bit_helper<port, bit>::reg();
		constexpr uint32_t group = (bit >> 3) & 1;
		constexpr uint32_t shift = (bit & 7) << 2;
		reg->AFR[group] = (reg->AFR[group] & ~(GPIO_AFRL_AFRL0_Msk << shift)) | ((function & GPIO_AFRL_AFRL0_Msk) << shift);		
	}
};



struct ConditionalExpandPattern
{
	static inline constexpr uint32_t calculate(uint32_t bitmask, uint32_t pattern, uint8_t width)
	{
		uint32_t value = 0;		
		pattern &= (1ul << width) - 1;
		for (int32_t rem = 32; rem > 0; rem -= width)
		{
			value |= (bitmask & 1) ? pattern : 0;
			bitmask >>= 1;
			pattern <<= width;
		}
		
	  return value;
	}
};

template <GPIO_Port port, uint16_t bitmask, GPIO_Mode mode>
class GPIO_bits;

template <GPIO_Port port, uint16_t bitmask>
class GPIO_bits_helper
{
private:
	GPIO_bits_helper() = delete;	
		
	static volatile GPIO_TypeDef * const reg()
	{
		return reinterpret_cast<GPIO_TypeDef*>(port);
	}
		
	constexpr static uint32_t portShift()
	{
		return (static_cast<uint32_t>(port) - static_cast<uint32_t>(GPIO_Port::A)) / (static_cast<uint32_t>(GPIO_Port::B) - static_cast<uint32_t>(GPIO_Port::A));
	}
	
	static inline void init(GPIO_Mode mode, GPIO_Driver driver, GPIO_Speed speed, GPIO_PullUpDown pu_pd)
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN_Msk << portShift();				
		
		reg()->PUPDR = (reg()->PUPDR & ~ConditionalExpandPattern::calculate(bitmask, GPIO_PUPDR_PUPDR0_Msk, 2))
			| ConditionalExpandPattern::calculate(bitmask, static_cast<uint32_t>(pu_pd), 2);					
		reg()->OSPEEDR = (reg()->OSPEEDR & ~ConditionalExpandPattern::calculate(bitmask, GPIO_OSPEEDR_OSPEEDR0_Msk, 2))
			| ConditionalExpandPattern::calculate(bitmask, static_cast<uint32_t>(speed), 2);
		reg()->OTYPER = (reg()->OTYPER & ~bitmask) | bitmask;
		reg()->MODER = (reg()->MODER & ~ConditionalExpandPattern::calculate(bitmask, GPIO_MODER_MODER0_Msk, 2))
			| ConditionalExpandPattern::calculate(bitmask, static_cast<uint32_t>(mode), 2);
	}
/*	
	static inline void init(GPIO_Mode mode, GPIO_Driver driver, GPIO_Speed speed, GPIO_PullUpDown pu_pd)
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN_Msk << portShift();		
		
		reg()->PUPDR = (reg()->PUPDR & ~ConditionalExpandNBitPattern<bitmask, GPIO_PUPDR_PUPDR0_Msk, 2>::value)
			| (ConditionalExpandNBitPattern<bitmask, static_cast<uint32_t>(GPIO_PullUpDown::up), 2>::value * (pu_pd == GPIO_PullUpDown::up ? 1 : 0))
			| (ConditionalExpandNBitPattern<bitmask, static_cast<uint32_t>(GPIO_PullUpDown::down), 2>::value * (pu_pd == GPIO_PullUpDown::down ? 1 : 0))
			| (ConditionalExpandNBitPattern<bitmask, static_cast<uint32_t>(GPIO_PullUpDown::floating), 2>::value * (pu_pd == GPIO_PullUpDown::floating ? 1 : 0));		
		reg()->OSPEEDR = (reg()->OSPEEDR & ~ConditionalExpandNBitPattern<bitmask, GPIO_OSPEEDR_OSPEEDR0_Msk, 2>::value)
			| (ConditionalExpandNBitPattern<bitmask, static_cast<uint32_t>(GPIO_Speed::low), 2>::value * (speed == GPIO_Speed::low ? 1 : 0))
			| (ConditionalExpandNBitPattern<bitmask, static_cast<uint32_t>(GPIO_Speed::medium), 2>::value * (speed == GPIO_Speed::medium ? 1 : 0))
			| (ConditionalExpandNBitPattern<bitmask, static_cast<uint32_t>(GPIO_Speed::high), 2>::value * (speed == GPIO_Speed::high ? 1 : 0))
			| (ConditionalExpandNBitPattern<bitmask, static_cast<uint32_t>(GPIO_Speed::very_high), 2>::value * (speed == GPIO_Speed::very_high ? 1 : 0));
		reg()->OTYPER = (reg()->OTYPER & ~bitmask) 
			| (ConditionalExpandNBitPattern<bitmask, static_cast<uint32_t>(GPIO_Driver::open_drain), 1>::value * (driver == GPIO_Driver::open_drain ? 1 : 0))
			| (ConditionalExpandNBitPattern<bitmask, static_cast<uint32_t>(GPIO_Driver::push_pull), 1>::value * (driver == GPIO_Driver::push_pull ? 1 : 0));
		reg()->MODER = (reg()->OTYPER & ~ConditionalExpandNBitPattern<bitmask, GPIO_MODER_MODER0_Msk, 2>::value)
			| (ConditionalExpandNBitPattern<bitmask, static_cast<uint32_t>(GPIO_Mode::input), 2>::value * (mode == GPIO_Mode::input ? 1 : 0))
			| (ConditionalExpandNBitPattern<bitmask, static_cast<uint32_t>(GPIO_Mode::analog), 2>::value * (mode == GPIO_Mode::analog ? 1 : 0))
			| (ConditionalExpandNBitPattern<bitmask, static_cast<uint32_t>(GPIO_Mode::output), 2>::value * (mode == GPIO_Mode::output ? 1 : 0))
			| (ConditionalExpandNBitPattern<bitmask, static_cast<uint32_t>(GPIO_Mode::alternate), 2>::value * (mode == GPIO_Mode::alternate ? 1 : 0));
	}
*/	
  friend class GPIO_bits<port, bitmask, GPIO_Mode::input>;
	friend class GPIO_bits<port, bitmask, GPIO_Mode::analog>;
	friend class GPIO_bits<port, bitmask, GPIO_Mode::output>;
	friend class GPIO_bits<port, bitmask, GPIO_Mode::alternate>;
};

// Default case => GPIO_Mode::input
template <GPIO_Port port, uint16_t bitmask, GPIO_Mode mode>
class GPIO_bits
{
public:
	GPIO_bits(GPIO_PullUpDown pu_pd)
	{
		init(pu_pd);
	}

	static inline void init(GPIO_PullUpDown pu_pd)
	{
		GPIO_bits_helper<port, bitmask>::init(mode, GPIO_Driver::open_drain, GPIO_Speed::low, pu_pd);
	}
	
	operator uint16_t()
	{
		return GPIO_bits_helper<port, bitmask>::reg()->IDR & bitmask;
	}
};

template <GPIO_Port port, uint16_t bitmask>
class GPIO_bits<port, bitmask, GPIO_Mode::analog>
{
public:
	GPIO_bits()
	{
		init();
	}

	static inline void init()
	{
		GPIO_bits_helper<port, bitmask>::init(GPIO_Mode::analog, GPIO_Driver::open_drain, GPIO_Speed::low, GPIO_PullUpDown::floating);
	}
};

template <GPIO_Port port, uint16_t bitmask>
class GPIO_bits<port, bitmask, GPIO_Mode::output>
{
public:
	GPIO_bits(GPIO_Driver driver, GPIO_Speed speed, GPIO_PullUpDown pu_pd, bool value)
	{
		init(driver, speed, pu_pd, value);
	}

	static inline void init(GPIO_Driver driver, GPIO_Speed speed, GPIO_PullUpDown pu_pd, uint16_t value)
	{
		GPIO_bits_helper<port, bitmask>::init(GPIO_Mode::output, driver, speed, pu_pd);

		GPIO_bits_helper<port, bitmask>::reg()->BSRR = (static_cast<uint32_t>(value & bitmask) << GPIO_BSRR_BS0_Pos) | (static_cast<uint32_t>(~value & bitmask) << GPIO_BSRR_BR0_Pos);
	}

	void operator=(uint16_t value)
	{
		GPIO_bits_helper<port, bitmask>::reg()->BSRR = (static_cast<uint32_t>(value & bitmask) << GPIO_BSRR_BS0_Pos) | (static_cast<uint32_t>(~value & bitmask) << GPIO_BSRR_BR0_Pos);
	}

	operator uint16_t()
	{
		return GPIO_bits_helper<port, bitmask>::reg()->IDR & bitmask;
	}

	uint16_t outState()
	{
		return GPIO_bits_helper<port, bitmask>::reg()->ODR & bitmask;
	}
};

template <GPIO_Port port, uint16_t bitmask>
class GPIO_bits<port, bitmask, GPIO_Mode::alternate>
{
public:
	GPIO_bits(GPIO_Driver driver, GPIO_Speed speed, GPIO_PullUpDown pu_pd, uint8_t function)
	{
		init(driver, speed, pu_pd, function);
	}
	
	static inline void init(GPIO_Driver driver, GPIO_Speed speed, GPIO_PullUpDown pu_pd, uint32_t function)
	{
		GPIO_bits_helper<port, bitmask>::init(GPIO_Mode::alternate, driver, speed, pu_pd);

//		assert( (function < 16), "Invalid Alternate Function number" );		
		volatile GPIO_TypeDef * const reg = GPIO_bits_helper<port, bitmask>::reg();
		reg->AFR[0] = (reg->AFR[0] & ~ConditionalExpandPattern::calculate(bitmask & 0x00ff, 0x0f, 4))
			| ConditionalExpandPattern::calculate(bitmask & 0x00ff, function & 0x0f, 4);
		reg->AFR[1] = (reg->AFR[1] & ~ConditionalExpandPattern::calculate(bitmask >> 8, 0x0f, 4))
			| ConditionalExpandPattern::calculate(bitmask >> 8, function & 0x0f, 4);		
//		reg->AFR[0] = (reg->AFR[0] & ~AF4_ConfigMultiple::config(bitmask & 0x00ff, 0x0f)) | AF4_ConfigMultiple::config(bitmask & 0x00ff, function);
//		reg->AFR[1] = (reg->AFR[1] & ~AF4_ConfigMultiple::config(bitmask >> 8, 0x0f)) | AF4_ConfigMultiple::config(bitmask >> 8, function);
	}
};
