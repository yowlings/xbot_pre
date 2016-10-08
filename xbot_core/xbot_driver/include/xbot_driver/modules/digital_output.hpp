/**
 * @file /xbot_driver/include/xbot_driver/modules/digital_output.hpp
 *
 * @brief Digital output flags.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef XBOT_DIGITAL_OUTPUT_HPP_
#define XBOT_DIGITAL_OUTPUT_HPP_

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace xbot
{

/*****************************************************************************
 ** Structures
 *****************************************************************************/
/**
 * Convenient class for setting values for digital output pins.
 */
struct DigitalOutput {
  DigitalOutput() {
    for ( unsigned int i = 0; i < 4; ++i ) {
      values[i] = false;
      mask[i] = false;
    }
  }
  bool values[4]; /**< Digital on or off for pins 0-3 respectively. **/
  bool mask[4]; /**< Set indices to true to set a pin, false to ignore. **/
};


} // namespace xbot

#endif /* XBOT_DIGITAL_OUTPUT_HPP_ */
