/**
 * @file include/xbot_driver/event_manager.hpp
 *
 * @brief The event manager - sigslot interface.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef XBOT_BUTTON_EVENT_HPP_
#define XBOT_BUTTON_EVENT_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <stdint.h>
#include <vector>
#include <ecl/sigslots.hpp>

#include "packets/core_sensors.hpp"
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot {

/*****************************************************************************
** Event Structures
*****************************************************************************/


struct PowerEvent {
  enum Event {
    Unplugged         = 0,
    PluggedToAdapter  = 1,
    PluggedToDockbase = 2,
    ChargeCompleted   = 3,
    BatteryLow        = 4,
    BatteryCritical   = 5
  } event;
};

struct RobotEvent {
  enum State {
    Offline,
    Online,
    Unknown  // at startup
  } state;
};

/*****************************************************************************
** Interfaces
*****************************************************************************/

class xbot_PUBLIC EventManager {
public:
  EventManager() {
    last_state.charger    = 0;
    last_state.battery    = 0;
    last_robot_state      = RobotEvent::Unknown;
  }

  void init(const std::string &sigslots_namespace);
  void update(const CoreSensors::Data &new_state);
  void update(bool is_plugged, bool is_alive);

private:
  CoreSensors::Data last_state;
  RobotEvent::State last_robot_state;

  ecl::Signal<const PowerEvent&>  sig_power_event;
  ecl::Signal<const RobotEvent&>  sig_robot_event;
};


} // namespace xbot

#endif /* XBOT_BUTTON_EVENT_HPP_ */
