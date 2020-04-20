/**
 * @file pin.hpp
 * @brief An STM32 pin representation.
 *
 * @author Varadi, Gyorgy, aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#ifndef __PIN_HPP__
#define __PIN_HPP__

#include "main.h"

#include <climits>
#include <cstring>

class Pin {
public:
	Pin(GPIO_TypeDef *port, uint16_t pin) : port(port), pin(pin) { };

	inline void high() { HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET); }
	inline void low() { HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); }
	inline void pulse() { high(); for (int i = 0; i < 1000; i++) { }; low(); }

	inline volatile bool isSet() { return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET; }

	inline bool operator==(const Pin& other) { return this->port == other.port && this->pin == other.pin; }

protected:
	GPIO_TypeDef *port;
	uint16_t pin;
};

#endif // __PIN_HPP__
