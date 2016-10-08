/**
 * @file /xbot_driver/src/driver/event_manager.cpp
 *
 * @brief Implementation of the event black magic.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/xbot_driver/event_manager.hpp"
#include "../../include/xbot_driver/modules/battery.hpp"
#include "../../include/xbot_driver/packets/core_sensors.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot {

/*****************************************************************************
** Implementation
*****************************************************************************/

void EventManager::init ( const std::string &sigslots_namespace ) {
  sig_power_event.connect(sigslots_namespace  + std::string("/power_event"));
  sig_robot_event.connect(sigslots_namespace  + std::string("/robot_event"));
}

/**
 * Update with incoming data and emit events if necessary.
 * @param new_state  Updated core sensors state
 * @param cliff_data Cliff sensors readings (we include them as an extra information on cliff events)
 */
void EventManager::update(const CoreSensors::Data &new_state)
{
  // ------------
  // Power System Event
  // ------------

  if (last_state.charger != new_state.charger)
  {
    Battery battery_new(new_state.battery, new_state.charger);
    Battery battery_last(last_state.battery, last_state.charger);

    if (battery_last.charging_state != battery_new.charging_state)
    {
      PowerEvent event;
      switch (battery_new.charging_state)
      {
        case Battery::Discharging:
          event.event = PowerEvent::Unplugged;
          break;
        case Battery::Charged:
          event.event = PowerEvent::ChargeCompleted;
          break;
        case Battery::Charging:
          if (battery_new.charging_source == Battery::Adapter)
            event.event = PowerEvent::PluggedToAdapter;
          else
            event.event = PowerEvent::PluggedToDockbase;
          break;
      }
      sig_power_event.emit(event);
    }
  }

  if (last_state.battery > new_state.battery)
  {
    Battery battery_new(new_state.battery, new_state.charger);
    Battery battery_last(last_state.battery, last_state.charger);

    if (battery_last.level() != battery_new.level())
    {
      PowerEvent event;
      switch (battery_new.level())
      {
        case Battery::Low:
          event.event = PowerEvent::BatteryLow;
          break;
        case Battery::Dangerous:
          event.event = PowerEvent::BatteryCritical;
          break;
        default:
          break;
      }
      sig_power_event.emit(event);
    }
  }

  last_state = new_state;
}


/**
 * Emit events if the robot gets online/offline.
 * @param is_plugged Is the USB cable connected?.
 * @param is_alive Is the robot alive?.
 */
void EventManager::update(bool is_plugged, bool is_alive)
{
  RobotEvent::State robot_state =
      (is_plugged && is_alive)?RobotEvent::Online:RobotEvent::Offline;
  if (last_robot_state != robot_state)
  {
    RobotEvent event;
    event.state = robot_state;

    sig_robot_event.emit(event);

    last_robot_state = robot_state;
  }
}

} // namespace xbot
