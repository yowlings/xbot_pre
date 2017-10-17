/**
 * @file /xbot_driver/include/xbot_driver/modules/led_array.hpp
 *
 * @brief Definitions for manipulating the led's.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef XBOT_LED_ARRAY_HPP_
#define XBOT_LED_ARRAY_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot {

/*****************************************************************************
** Enums
*****************************************************************************/

/**
 * The led's count from left to right.
 */
enum LedNumber {
  Led1 = 0,  //!< Led1
  Led2 = 1   //!< Led2
};

enum LedColour {
  Black = 0x00,
  Red = 0x100,
  Green = 0x200,
  Orange = 0x300,
};

} // namespace xbot

#endif /* LED_ARRAY_HPP_ */
