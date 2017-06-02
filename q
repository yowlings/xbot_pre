[1mdiff --git a/xbot_core/xbot_driver/include/xbot_driver/command.hpp b/xbot_core/xbot_driver/include/xbot_driver/command.hpp[m
[1mindex 140e6f9..ebb4f21 100644[m
[1m--- a/xbot_core/xbot_driver/include/xbot_driver/command.hpp[m
[1m+++ b/xbot_core/xbot_driver/include/xbot_driver/command.hpp[m
[36m@@ -19,7 +19,6 @@[m
 [m
 #include <ecl/containers.hpp>[m
 #include "packet_handler/payload_base.hpp"[m
[31m-#include "modules/led_array.hpp"[m
 #include "modules.hpp"[m
 #include "macros.hpp"[m
 [m
[1mdiff --git a/xbot_core/xbot_driver/include/xbot_driver/event_manager.hpp b/xbot_core/xbot_driver/include/xbot_driver/event_manager.hpp[m
[1mdeleted file mode 100644[m
[1mindex 099d467..0000000[m
[1m--- a/xbot_core/xbot_driver/include/xbot_driver/event_manager.hpp[m
[1m+++ /dev/null[m
[36m@@ -1,84 +0,0 @@[m
[31m-/**[m
[31m- * @file include/xbot_driver/event_manager.hpp[m
[31m- *[m
[31m- * @brief The event manager - sigslot interface.[m
[31m- *[m
[31m- * License: BSD[m
[31m- *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE[m
[31m- **/[m
[31m-/*****************************************************************************[m
[31m-** Ifdefs[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-#ifndef XBOT_BUTTON_EVENT_HPP_[m
[31m-#define XBOT_BUTTON_EVENT_HPP_[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Includes[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-#include <stdint.h>[m
[31m-#include <vector>[m
[31m-#include <ecl/sigslots.hpp>[m
[31m-[m
[31m-#include "packets/core_sensors.hpp"[m
[31m-#include "macros.hpp"[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Namespaces[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-namespace xbot {[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Event Structures[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-[m
[31m-struct PowerEvent {[m
[31m-  enum Event {[m
[31m-    Unplugged         = 0,[m
[31m-    PluggedToAdapter  = 1,[m
[31m-    PluggedToDockbase = 2,[m
[31m-    ChargeCompleted   = 3,[m
[31m-    BatteryLow        = 4,[m
[31m-    BatteryCritical   = 5[m
[31m-  } event;[m
[31m-};[m
[31m-[m
[31m-struct RobotEvent {[m
[31m-  enum State {[m
[31m-    Offline,[m
[31m-    Online,[m
[31m-    Unknown  // at startup[m
[31m-  } state;[m
[31m-};[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Interfaces[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-class xbot_PUBLIC EventManager {[m
[31m-public:[m
[31m-  EventManager() {[m
[31m-    last_state.charger    = 0;[m
[31m-    last_state.battery    = 0;[m
[31m-    last_robot_state      = RobotEvent::Unknown;[m
[31m-  }[m
[31m-[m
[31m-  void init(const std::string &sigslots_namespace);[m
[31m-  void update(const CoreSensors::Data &new_state);[m
[31m-  void update(bool is_plugged, bool is_alive);[m
[31m-[m
[31m-private:[m
[31m-  CoreSensors::Data last_state;[m
[31m-  RobotEvent::State last_robot_state;[m
[31m-[m
[31m-  ecl::Signal<const PowerEvent&>  sig_power_event;[m
[31m-  ecl::Signal<const RobotEvent&>  sig_robot_event;[m
[31m-};[m
[31m-[m
[31m-[m
[31m-} // namespace xbot[m
[31m-[m
[31m-#endif /* XBOT_BUTTON_EVENT_HPP_ */[m
[1mdiff --git a/xbot_core/xbot_driver/include/xbot_driver/modules.hpp b/xbot_core/xbot_driver/include/xbot_driver/modules.hpp[m
[1mindex ee7b7a9..612b293 100644[m
[1m--- a/xbot_core/xbot_driver/include/xbot_driver/modules.hpp[m
[1m+++ b/xbot_core/xbot_driver/include/xbot_driver/modules.hpp[m
[36m@@ -17,11 +17,8 @@[m
 ** Includes[m
 *****************************************************************************/[m
 [m
[31m-#include "modules/battery.hpp"[m
[31m-#include "modules/digital_output.hpp"[m
[31m-#include "modules/led_array.hpp"[m
[32m+[m
 #include "modules/diff_drive.hpp"[m
[31m-#include "modules/sound.hpp"[m
 #include "modules/acceleration_limiter.hpp"[m
 [m
 #endif /* XBOT_MODULES_HPP_ */[m
[1mdiff --git a/xbot_core/xbot_driver/include/xbot_driver/modules/battery.hpp b/xbot_core/xbot_driver/include/xbot_driver/modules/battery.hpp[m
[1mdeleted file mode 100644[m
[1mindex 729be0f..0000000[m
[1m--- a/xbot_core/xbot_driver/include/xbot_driver/modules/battery.hpp[m
[1m+++ /dev/null[m
[36m@@ -1,74 +0,0 @@[m
[31m-/**[m
[31m- * @file /xbot_driver/include/xbot_driver/modules/battery.hpp[m
[31m- *[m
[31m- * @brief Human friendly batter indicator class.[m
[31m- *[m
[31m- * License: BSD[m
[31m- *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE[m
[31m- **/[m
[31m-/*****************************************************************************[m
[31m-** Ifdefs[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-#ifndef XBOT_BATTERY_HPP_[m
[31m-#define XBOT_BATTERY_HPP_[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Includes[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-#include <stdint.h>[m
[31m-#include "../packets/core_sensors.hpp"[m
[31m-#include "../macros.hpp"[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Namespaces[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-namespace xbot {[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Interfaces[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-/**[m
[31m- * @brief  Battery level module.[m
[31m- *[m
[31m- * Currently hard codes the battery status. It might be useful to provide[m
[31m- * some configurable parameters for this module in the future.[m
[31m- **/[m
[31m-class xbot_PUBLIC Battery {[m
[31m-public:[m
[31m-  enum Source {[m
[31m-    None,[m
[31m-    Adapter,[m
[31m-    Dock[m
[31m-  };[m
[31m-  enum Level {[m
[31m-    Dangerous,[m
[31m-    Low,[m
[31m-    Healthy,[m
[31m-    Maximum[m
[31m-  };[m
[31m-  enum State {[m
[31m-    Discharging,[m
[31m-    Charged,[m
[31m-    Charging[m
[31m-  };[m
[31m-[m
[31m-  Battery() {} /**< Default constructor. **/[m
[31m-  Battery (const uint8_t &new_voltage, const uint8_t &charger_flag);[m
[31m-  Level level() const;[m
[31m-  float percent() const;[m
[31m-[m
[31m-  static double capacity;[m
[31m-  static double low;[m
[31m-  static double dangerous;[m
[31m-  double voltage;[m
[31m-  State charging_state;[m
[31m-  Source charging_source;[m
[31m-};[m
[31m-[m
[31m-} // namespace xbot[m
[31m-[m
[31m-#endif /* XBOT_BATTERY_HPP_ */[m
[1mdiff --git a/xbot_core/xbot_driver/include/xbot_driver/modules/digital_output.hpp b/xbot_core/xbot_driver/include/xbot_driver/modules/digital_output.hpp[m
[1mdeleted file mode 100644[m
[1mindex d3fe2f3..0000000[m
[1m--- a/xbot_core/xbot_driver/include/xbot_driver/modules/digital_output.hpp[m
[1m+++ /dev/null[m
[36m@@ -1,43 +0,0 @@[m
[31m-/**[m
[31m- * @file /xbot_driver/include/xbot_driver/modules/digital_output.hpp[m
[31m- *[m
[31m- * @brief Digital output flags.[m
[31m- *[m
[31m- * License: BSD[m
[31m- *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE[m
[31m- **/[m
[31m-/*****************************************************************************[m
[31m- ** Ifdefs[m
[31m- *****************************************************************************/[m
[31m-[m
[31m-#ifndef XBOT_DIGITAL_OUTPUT_HPP_[m
[31m-#define XBOT_DIGITAL_OUTPUT_HPP_[m
[31m-[m
[31m-/*****************************************************************************[m
[31m- ** Namespaces[m
[31m- *****************************************************************************/[m
[31m-[m
[31m-namespace xbot[m
[31m-{[m
[31m-[m
[31m-/*****************************************************************************[m
[31m- ** Structures[m
[31m- *****************************************************************************/[m
[31m-/**[m
[31m- * Convenient class for setting values for digital output pins.[m
[31m- */[m
[31m-struct DigitalOutput {[m
[31m-  DigitalOutput() {[m
[31m-    for ( unsigned int i = 0; i < 4; ++i ) {[m
[31m-      values[i] = false;[m
[31m-      mask[i] = false;[m
[31m-    }[m
[31m-  }[m
[31m-  bool values[4]; /**< Digital on or off for pins 0-3 respectively. **/[m
[31m-  bool mask[4]; /**< Set indices to true to set a pin, false to ignore. **/[m
[31m-};[m
[31m-[m
[31m-[m
[31m-} // namespace xbot[m
[31m-[m
[31m-#endif /* XBOT_DIGITAL_OUTPUT_HPP_ */[m
[1mdiff --git a/xbot_core/xbot_driver/include/xbot_driver/modules/led_array.hpp b/xbot_core/xbot_driver/include/xbot_driver/modules/led_array.hpp[m
[1mdeleted file mode 100644[m
[1mindex 93c28e3..0000000[m
[1m--- a/xbot_core/xbot_driver/include/xbot_driver/modules/led_array.hpp[m
[1m+++ /dev/null[m
[36m@@ -1,47 +0,0 @@[m
[31m-/**[m
[31m- * @file /xbot_driver/include/xbot_driver/modules/led_array.hpp[m
[31m- *[m
[31m- * @brief Definitions for manipulating the led's.[m
[31m- *[m
[31m- * License: BSD[m
[31m- *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE[m
[31m- **/[m
[31m-/*****************************************************************************[m
[31m-** Ifdefs[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-#ifndef XBOT_LED_ARRAY_HPP_[m
[31m-#define XBOT_LED_ARRAY_HPP_[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Includes[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Namespaces[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-namespace xbot {[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Enums[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-/**[m
[31m- * The led's count from left to right.[m
[31m- */[m
[31m-enum LedNumber {[m
[31m-  Led1 = 0,  //!< Led1[m
[31m-  Led2 = 1   //!< Led2[m
[31m-};[m
[31m-[m
[31m-enum LedColour {[m
[31m-  Black = 0x00,[m
[31m-  Red = 0x100,[m
[31m-  Green = 0x200,[m
[31m-  Orange = 0x300,[m
[31m-};[m
[31m-[m
[31m-} // namespace xbot[m
[31m-[m
[31m-#endif /* LED_ARRAY_HPP_ */[m
[1mdiff --git a/xbot_core/xbot_driver/include/xbot_driver/modules/sound.hpp b/xbot_core/xbot_driver/include/xbot_driver/modules/sound.hpp[m
[1mdeleted file mode 100644[m
[1mindex 0655c50..0000000[m
[1m--- a/xbot_core/xbot_driver/include/xbot_driver/modules/sound.hpp[m
[1m+++ /dev/null[m
[36m@@ -1,43 +0,0 @@[m
[31m-/**[m
[31m- * @file /xbot_driver/include/xbot_driver/modules/sound.hpp[m
[31m- *[m
[31m- * @brief Flags and id's for commanding sound sequences.[m
[31m- *[m
[31m- * License: BSD[m
[31m- *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE[m
[31m- **/[m
[31m-/*****************************************************************************[m
[31m-** Ifdefs[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-#ifndef XBOT_SOUND_HPP_[m
[31m-#define XBOT_SOUND_HPP_[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Includes[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Namespaces[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-namespace xbot {[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Enums[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-enum SoundSequences[m
[31m-{[m
[31m-  On = 0x0, /**< Turn on **/[m
[31m-  Off = 0x1, /**< Turn off **/[m
[31m-  Recharge = 0x2, /**< Recharging starting **/[m
[31m-  Button = 0x3, /**< Button pressed  **/[m
[31m-  Error = 0x4, /**< Error sound **/[m
[31m-  CleaningStart = 0x5, /**< Cleaning started **/[m
[31m-  CleaningEnd = 0x6, /**< Cleaning ended **/[m
[31m-};[m
[31m-[m
[31m-} // namespace xbot[m
[31m-[m
[31m-#endif /* XBOT_SOUND_HPP_ */[m
[1mdiff --git a/xbot_core/xbot_driver/include/xbot_driver/packets/core_sensors.hpp b/xbot_core/xbot_driver/include/xbot_driver/packets/core_sensors.hpp[m
[1mindex 3fdb9e8..adfbfbb 100644[m
[1m--- a/xbot_core/xbot_driver/include/xbot_driver/packets/core_sensors.hpp[m
[1m+++ b/xbot_core/xbot_driver/include/xbot_driver/packets/core_sensors.hpp[m
[36m@@ -38,21 +38,37 @@[m [mpublic:[m
   CoreSensors() : packet_handler::payloadBase(false, 42) {};[m
 [m
   struct Data {[m
[31m-     float power_voltage;[m
[31m-     uint16_t infred_1;[m
[31m-     u_int16_t infred_2;[m
[31m-     float current_1;[m
[31m-     float current_2;[m
[31m-     float current_3;[m
[31m-     float echo_1;[m
[31m-     float echo_2;[m
[31m-     float echo_3;[m
[31m-     float echo_4;[m
[31m-     uint16_t left_encoder;[m
[31m-     uint16_t right_encoder;[m
[31m-     uint16_t up_encoder;[m
[31m-     uint8_t charger;[m
[31m-     uint8_t battery;[m
[32m+[m[32m     float battery_voltage;[m
[32m+[m[32m     uint16_t rear_left_infred;[m
[32m+[m[32m     uint16_t rear_center_infred;[m
[32m+[m[32m     uint16_t rear_right_infred;[m
[32m+[m[32m     float front_left_current;[m
[32m+[m[32m     float front_right_current;[m
[32m+[m[32m     float rear_left_current;[m
[32m+[m[32m     float rear_right_current;[m
[32m+[m[32m     float up_down_current;[m
[32m+[m[32m     float front_left_echo;[m
[32m+[m[32m     float front_center_echo;[m
[32m+[m[32m     float front_right_echo;[m
[32m+[m[32m     uint16_t front_left_encoder;[m
[32m+[m[32m     uint16_t front_right_encoder;[m
[32m+[m[32m     uint16_t rear_left_encoder;[m
[32m+[m[32m     uint16_t rear_right_encoder;[m
[32m+[m[32m     uint16_t up_down_encoder;[m
[32m+[m[32m     short acce_x;[m
[32m+[m[32m     short acce_y;[m
[32m+[m[32m     short acce_z;[m
[32m+[m[32m     short gyro_x;[m
[32m+[m[32m     short gyro_y;[m
[32m+[m[32m     short gyro_z;[m
[32m+[m[32m     short mag_x;[m
[32m+[m[32m     short mag_y;[m
[32m+[m[32m     short mag_z;[m
[32m+[m[32m     float pressure;[m
[32m+[m[32m     short yaw;[m
[32m+[m[32m     short pitch;[m
[32m+[m[32m     short roll;[m
[32m+[m[32m     unsigned int timestamp;[m
   } data;[m
 [m
   struct Flags {[m
[1mdiff --git a/xbot_core/xbot_driver/include/xbot_driver/parameters.hpp b/xbot_core/xbot_driver/include/xbot_driver/parameters.hpp[m
[1mindex 18be16b..504bfd8 100644[m
[1m--- a/xbot_core/xbot_driver/include/xbot_driver/parameters.hpp[m
[1m+++ b/xbot_core/xbot_driver/include/xbot_driver/parameters.hpp[m
[36m@@ -18,7 +18,6 @@[m
  *****************************************************************************/[m
 [m
 #include <string>[m
[31m-#include "modules/battery.hpp"[m
 [m
 /*****************************************************************************[m
  ** Namespaces[m
[36m@@ -41,9 +40,6 @@[m [mpublic:[m
     sigslots_namespace("/xbot"),[m
     simulation(false),[m
     enable_acceleration_limiter(true),[m
[31m-    battery_capacity(Battery::capacity),[m
[31m-    battery_low(Battery::low),[m
[31m-    battery_dangerous(Battery::dangerous),[m
     linear_acceleration_limit(0.3),[m
     linear_deceleration_limit(-0.3*1.2),[m
     angular_acceleration_limit(3.5),[m
[36m@@ -55,9 +51,6 @@[m [mpublic:[m
   std::string sigslots_namespace;  /**< @brief The first part of a sigslot connection namespace ["/xbot"] **/[m
   bool simulation;                 /**< @brief Whether to put the motors in loopback mode or not [false] **/[m
   bool enable_acceleration_limiter;/**< @brief Enable or disable the acceleration limiter [true] **/[m
[31m-  double battery_capacity;         /**< @brief Capacity voltage of the battery [16.5V] **/ /* defaults defined in battery.cpp */[m
[31m-  double battery_low;              /**< @brief Threshold for battery level warnings [14.0V] **/  /* defaults defined in battery.cpp */[m
[31m-  double battery_dangerous;        /**< @brief Threshold for battery level in danger of depletion [13.2V] **/  /* defaults defined in battery.cpp */[m
 [m
   double linear_acceleration_limit;[m
   double linear_deceleration_limit;[m
[1mdiff --git a/xbot_core/xbot_driver/include/xbot_driver/xbot.hpp b/xbot_core/xbot_driver/include/xbot_driver/xbot.hpp[m
[1mindex 7f63d03..22e247f 100644[m
[1m--- a/xbot_core/xbot_driver/include/xbot_driver/xbot.hpp[m
[1m+++ b/xbot_core/xbot_driver/include/xbot_driver/xbot.hpp[m
[36m@@ -25,7 +25,6 @@[m
 #include <ecl/threads/mutex.hpp>[m
 #include <ecl/exceptions/standard_exception.hpp>[m
 #include "parameters.hpp"[m
[31m-#include "event_manager.hpp"[m
 #include "command.hpp"[m
 #include "modules.hpp"[m
 #include "packets.hpp"[m
[36m@@ -120,7 +119,6 @@[m [mpublic:[m
   float getHeading() const;[m
   int getDebugSensors() const;[m
   float getAngularVelocity() const;[m
[31m-  Battery batteryStatus() const { return Battery(core_sensors.data.battery, core_sensors.data.charger); }[m
   unsigned char getHeightPercent() {return HeightPercent;}[m
   unsigned char getCameraDegree(){return CameraDegree;}[m
   unsigned char getPlatformDegree(){return PlatformDegree;}[m
[36m@@ -214,20 +212,14 @@[m [mprivate:[m
   Command xbot_command; // used to maintain some state about the command history[m
   Command::Buffer command_buffer;[m
 [m
[31m-  /*********************[m
[31m-  ** Events[m
[31m-  **********************/[m
[31m-  EventManager event_manager;[m
 [m
   /*********************[m
   ** Signals[m
   **********************/[m
[31m-  ecl::Signal<> sig_stream_data, sig_controller_info;[m
[32m+[m[32m  ecl::Signal<> sig_stream_data;[m
   ecl::Signal<const std::string&> sig_debug, sig_info, sig_warn, sig_error;[m
[31m-  ecl::Signal<const std::vector<std::string>&> sig_named;[m
   ecl::Signal<Command::Buffer&> sig_raw_data_command; // should be const, but pushnpop is not fully realised yet for const args in the formatters.[m
   ecl::Signal<PacketFinder::BufferType&> sig_raw_data_stream; // should be const, but pushnpop is not fully realised yet for const args in the formatters.[m
[31m-  ecl::Signal<const std::vector<float>&> sig_raw_control_command;[m
 };[m
 [m
 } // namespace xbot[m
[1mdiff --git a/xbot_core/xbot_driver/src/driver/battery.cpp b/xbot_core/xbot_driver/src/driver/battery.cpp[m
[1mdeleted file mode 100644[m
[1mindex 87f492c..0000000[m
[1m--- a/xbot_core/xbot_driver/src/driver/battery.cpp[m
[1m+++ /dev/null[m
[36m@@ -1,74 +0,0 @@[m
[31m-/**[m
[31m- * @file /xbot_driver/src/driver/battery.cpp[m
[31m- *[m
[31m- * @brief Battery/charging source implementation[m
[31m- *[m
[31m- * License: BSD[m
[31m- *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE[m
[31m- **/[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Includes[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-#include "../../include/xbot_driver/modules/battery.hpp"[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Namespaces[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-namespace xbot {[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Statics[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-double Battery::capacity  = 16.5;[m
[31m-double Battery::low       = 14.0;[m
[31m-double Battery::dangerous = 13.2;[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Implementation[m
[31m-*****************************************************************************/[m
[31m-/**[m
[31m- * Configures the battery status given the current sensor readings.[m
[31m- *[m
[31m- * @param new_voltage : measured voltage*10[m
[31m- * @param charger_flag : bit flag representing charging status and source[m
[31m- */[m
[31m-Battery::Battery (const uint8_t &new_voltage, const uint8_t &charger_flag) :[m
[31m-  voltage(static_cast<double>(new_voltage)/10.0)[m
[31m-{[m
[31m-  uint8_t state = (charger_flag & CoreSensors::Flags::BatteryStateMask);[m
[31m-  if ( state == CoreSensors::Flags::Charging) {[m
[31m-    charging_state = Charging;[m
[31m-  } else if ( state == CoreSensors::Flags::Charged ) {[m
[31m-    charging_state = Charged;[m
[31m-    capacity = voltage;[m
[31m-  } else {[m
[31m-    charging_state = Discharging;[m
[31m-  }[m
[31m-[m
[31m-  if (charging_state == Discharging) {[m
[31m-    charging_source = None;[m
[31m-  } else if ( charger_flag & CoreSensors::Flags::AdapterType ) {[m
[31m-    charging_source = Adapter;[m
[31m-  } else {[m
[31m-    charging_source = Dock;[m
[31m-  }[m
[31m-};[m
[31m-[m
[31m-Battery::Level Battery::level() const {[m
[31m-  if ( charging_state == Charged ) { return Maximum; }[m
[31m-  if ( voltage > low ) { return Healthy; }[m
[31m-  if ( voltage > dangerous ) { return Low; }[m
[31m-  return Dangerous;[m
[31m-}[m
[31m-[m
[31m-float Battery::percent() const {[m
[31m-  // convert battery voltage to percent: 100% -> capacity / 5% -> dangerous[m
[31m-  float percent = ((95*(voltage - dangerous)) / (capacity - dangerous)) + 5;[m
[31m-  return std::max(std::min(percent, 100.0f), 0.0f);[m
[31m-}[m
[31m-[m
[31m-} // namespace xbot[m
[1mdiff --git a/xbot_core/xbot_driver/src/driver/core_sensors.cpp b/xbot_core/xbot_driver/src/driver/core_sensors.cpp[m
[1mindex 59ed17d..ea722a8 100644[m
[1m--- a/xbot_core/xbot_driver/src/driver/core_sensors.cpp[m
[1m+++ b/xbot_core/xbot_driver/src/driver/core_sensors.cpp[m
[36m@@ -57,42 +57,60 @@[m [mbool CoreSensors::deserialise(ecl::PushAndPop<unsigned char> & byteStream)[m
 //    return false;[m
 //  }[m
 [m
[31m-  unsigned char header_id;[m
[31m-  buildVariable(header_id, byteStream);[m
 //  std::cout<<"header_id:"<<(unsigned int)header_id<<std::endl;[m
[31m-[m
[31m-  if( header_id != Header::CoreSensors ) return false;[m
   unsigned char power_num;[m
   buildVariable(power_num, byteStream);[m
 //  std::cout<<"power_num:"<<(unsigned int)power_num<<std::endl;[m
 [m
[31m-  build_special_variable(data.power_voltage,byteStream);[m
[32m+[m[32m  build_special_variable(data.battery_voltage,byteStream);[m
 //  std::cout<<"power_voltage:"<<data.power_voltage<<std::endl;[m
   unsigned char infred_num;[m
   buildVariable(infred_num, byteStream);[m
[31m-  buildVariable(data.infred_1,byteStream);[m
[31m-  buildVariable(data.infred_2,byteStream);[m
[32m+[m[32m  buildVariable(data.rear_left_infred,byteStream);[m
[32m+[m[32m  buildVariable(data.rear_center_infred,byteStream);[m
[32m+[m[32m  buildVariable(data.rear_right_infred,byteStream);[m
 [m
   unsigned char current_num;[m
   buildVariable(current_num,byteStream);[m
[31m-  build_special_variable(data.current_1,byteStream);[m
[31m-  build_special_variable(data.current_2,byteStream);[m
[31m-  build_special_variable(data.current_3,byteStream);[m
[32m+[m[32m  build_special_variable(data.front_left_current,byteStream);[m
[32m+[m[32m  build_special_variable(data.front_right_current,byteStream);[m
[32m+[m[32m  build_special_variable(data.rear_left_current,byteStream);[m
[32m+[m[32m  build_special_variable(data.rear_right_current,byteStream);[m
[32m+[m[32m  build_special_variable(data.up_down_current,byteStream);[m
 [m
   unsigned char echo_num;[m
   buildVariable(echo_num,byteStream);[m
[31m-  build_special_variable(data.echo_1,byteStream);[m
[31m-  build_special_variable(data.echo_2,byteStream);[m
[31m-  build_special_variable(data.echo_3,byteStream);[m
[31m-  build_special_variable(data.echo_4,byteStream);[m
[32m+[m[32m  build_special_variable(data.front_left_echo,byteStream);[m
[32m+[m[32m  build_special_variable(data.front_center_echo,byteStream);[m
[32m+[m[32m  build_special_variable(data.front_right_echo,byteStream);[m
 [m
   unsigned char encoder_num;[m
   buildVariable(encoder_num, byteStream);[m
[31m-  buildVariable(data.left_encoder, byteStream);[m
[31m-  buildVariable(data.right_encoder, byteStream);[m
[31m-  buildVariable(data.up_encoder, byteStream);[m
[31m-[m
[31m-  std::cout<<"time:"<<time(0)<<"|left_encoder:"<<data.left_encoder<<std::endl;[m
[32m+[m[32m  buildVariable(data.front_left_encoder, byteStream);[m
[32m+[m[32m  buildVariable(data.front_right_encoder, byteStream);[m
[32m+[m[32m  buildVariable(data.rear_left_encoder, byteStream);[m
[32m+[m[32m  buildVariable(data.rear_right_encoder, byteStream);[m
[32m+[m[32m  buildVariable(data.up_down_encoder, byteStream);[m
[32m+[m
[32m+[m[32m  unsigned char imu_num;[m
[32m+[m[32m  buildVariable(imu_num, byteStream);[m
[32m+[m[32m  buildVariable(data.acce_x, byteStream);[m
[32m+[m[32m  buildVariable(data.acce_y, byteStream);[m
[32m+[m[32m  buildVariable(data.acce_z, byteStream);[m
[32m+[m[32m  buildVariable(data.gyro_x, byteStream);[m
[32m+[m[32m  buildVariable(data.gyro_y, byteStream);[m
[32m+[m[32m  buildVariable(data.gyro_z, byteStream);[m
[32m+[m[32m  buildVariable(data.mag_x, byteStream);[m
[32m+[m[32m  buildVariable(data.mag_y, byteStream);[m
[32m+[m[32m  buildVariable(data.mag_z, byteStream);[m
[32m+[m[32m  buildVariable(data.pressure, byteStream);[m
[32m+[m[32m  buildVariable(data.yaw,byteStream);[m
[32m+[m[32m  buildVariable(data.pitch, byteStream);[m
[32m+[m[32m  buildVariable(data.roll, byteStream);[m
[32m+[m[32m  buildVariable(data.timestamp, byteStream);[m
[32m+[m
[32m+[m
[32m+[m[32m//  std::cout<<"time:"<<time(0)<<"|left_encoder:"<<data.left_encoder<<std::endl;[m
 [m
 [m
 //  std::cout<<"power:"<<data.power_voltage<<"|Echo1:"<<data.echo_1<<"|Echo2:"<<data.echo_2<<"|Echo3:"<<data.echo_3<<"|Echo4:"<<data.echo_4<<std::endl;[m
[1mdiff --git a/xbot_core/xbot_driver/src/driver/event_manager.cpp b/xbot_core/xbot_driver/src/driver/event_manager.cpp[m
[1mdeleted file mode 100644[m
[1mindex 20d5497..0000000[m
[1m--- a/xbot_core/xbot_driver/src/driver/event_manager.cpp[m
[1m+++ /dev/null[m
[36m@@ -1,118 +0,0 @@[m
[31m-/**[m
[31m- * @file /xbot_driver/src/driver/event_manager.cpp[m
[31m- *[m
[31m- * @brief Implementation of the event black magic.[m
[31m- *[m
[31m- * License: BSD[m
[31m- *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE[m
[31m- **/[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Includes[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-#include "../../include/xbot_driver/event_manager.hpp"[m
[31m-#include "../../include/xbot_driver/modules/battery.hpp"[m
[31m-#include "../../include/xbot_driver/packets/core_sensors.hpp"[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Namespaces[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-namespace xbot {[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Implementation[m
[31m-*****************************************************************************/[m
[31m-[m
[31m-void EventManager::init ( const std::string &sigslots_namespace ) {[m
[31m-  sig_power_event.connect(sigslots_namespace  + std::string("/power_event"));[m
[31m-  sig_robot_event.connect(sigslots_namespace  + std::string("/robot_event"));[m
[31m-}[m
[31m-[m
[31m-/**[m
[31m- * Update with incoming data and emit events if necessary.[m
[31m- * @param new_state  Updated core sensors state[m
[31m- * @param cliff_data Cliff sensors readings (we include them as an extra information on cliff events)[m
[31m- */[m
[31m-void EventManager::update(const CoreSensors::Data &new_state)[m
[31m-{[m
[31m-  // ------------[m
[31m-  // Power System Event[m
[31m-  // ------------[m
[31m-[m
[31m-  if (last_state.charger != new_state.charger)[m
[31m-  {[m
[31m-    Battery battery_new(new_state.battery, new_state.charger);[m
[31m-    Battery battery_last(last_state.battery, last_state.charger);[m
[31m-[m
[31m-    if (battery_last.charging_state != battery_new.charging_state)[m
[31m-    {[m
[31m-      PowerEvent event;[m
[31m-      switch (battery_new.charging_state)[m
[31m-      {[m
[31m-        case Battery::Discharging:[m
[31m-          event.event = PowerEvent::Unplugged;[m
[31m-          break;[m
[31m-        case Battery::Charged:[m
[31m-          event.event = PowerEvent::ChargeCompleted;[m
[31m-          break;[m
[31m-        case Battery::Charging:[m
[31m-          if (battery_new.charging_source == Battery::Adapter)[m
[31m-            event.event = PowerEvent::PluggedToAdapter;[m
[31m-          else[m
[31m-            event.event = PowerEvent::PluggedToDockbase;[m
[31m-          break;[m
[31m-      }[m
[31m-      sig_power_event.emit(event);[m
[31m-    }[m
[31m-  }[m
[31m-[m
[31m-  if (last_state.battery > new_state.battery)[m
[31m-  {[m
[31m-    Battery battery_new(new_state.battery, new_state.charger);[m
[31m-    Battery battery_last(last_state.battery, last_state.charger);[m
[31m-[m
[31m-    if (battery_last.level() != battery_new.level())[m
[31m-    {[m
[31m-      PowerEvent event;[m
[31m-      switch (battery_new.level())[m
[31m-      {[m
[31m-        case Battery::Low:[m
[31m-          event.event = PowerEvent::BatteryLow;[m
[31m-          break;[m
[31m-        case Battery::Dangerous:[m
[31m-          event.event = PowerEvent::BatteryCritical;[m
[31m-          break;[m
[31m-        default:[m
[31m-          break;[m
[31m-      }[m
[31m-      sig_power_event.emit(event);[m
[31m-    }[m
[31m-  }[m
[31m-[m
[31m-  last_state = new_state;[m
[31m-}[m
[31m-[m
[31m-[m
[31m-/**[m
[31m- * Emit events if the robot gets online/offline.[m
[31m- * @param is_plugged Is the USB cable connected?.[m
[31m- * @param is_alive Is the robot alive?.[m
[31m- */[m
[31m-void EventManager::update(bool is_plugged, bool is_alive)[m
[31m-{[m
[31m-  RobotEvent::State robot_state =[m
[31m-      (is_plugged && is_alive)?RobotEvent::Online:RobotEvent::Offline;[m
[31m-  if (last_robot_state != robot_state)[m
[31m-  {[m
[31m-    RobotEvent event;[m
[31m-    event.state = robot_state;[m
[31m-[m
[31m-    sig_robot_event.emit(event);[m
[31m-[m
[31m-    last_robot_state = robot_state;[m
[31m-  }[m
[31m-}[m
[31m-[m
[31m-} // namespace xbot[m
[1mdiff --git a/xbot_core/xbot_driver/src/driver/version_info.cpp.in b/xbot_core/xbot_driver/src/driver/version_info.cpp.in[m
[1mdeleted file mode 100644[m
[1mindex 9a83e4f..0000000[m
[1m--- a/xbot_core/xbot_driver/src/driver/version_info.cpp.in[m
[1m+++ /dev/null[m
[36m@@ -1,11 +0,0 @@[m
[31m-#include "xbot_driver/version_info.hpp"[m
[31m-[m
[31m-namespace xbot[m
[31m-{[m
[31m-[m
[31m-std::string VersionInfo::getSoftwareVersion()[m
[31m-{[m
[31m-  return std::string("@xbot_driver_VERSION@");[m
[31m-}[m
[31m-[m
[31m-} //namespace xbot[m
[1mdiff --git a/xbot_core/xbot_driver/src/driver/xbot.cpp b/xbot_core/xbot_driver/src/driver/xbot.cpp[m
[1mindex c5596f0..53d114a 100644[m
[1m--- a/xbot_core/xbot_driver/src/driver/xbot.cpp[m
[1m+++ b/xbot_core/xbot_driver/src/driver/xbot.cpp[m
[36m@@ -82,21 +82,17 @@[m [mvoid Xbot::init(Parameters &parameters) throw (ecl::StandardException)[m
   }[m
   this->parameters = parameters;[m
   std::string sigslots_namespace = parameters.sigslots_namespace;[m
[31m-  event_manager.init(sigslots_namespace);[m
 [m
   // connect signals[m
[31m-  sig_controller_info.connect(sigslots_namespace + std::string("/controller_info"));[m
   sig_stream_data.connect(sigslots_namespace + std::string("/stream_data"));[m
   sig_raw_data_command.connect(sigslots_namespace + std::string("/raw_data_command"));[m
   sig_raw_data_stream.connect(sigslots_namespace + std::string("/raw_data_stream"));[m
[31m-  sig_raw_control_command.connect(sigslots_namespace + std::string("/raw_control_command"));[m
   //sig_serial_timeout.connect(sigslots_namespace+std::string("/serial_timeout"));[m
 [m
   sig_debug.connect(sigslots_namespace + std::string("/ros_debug"));[m
   sig_info.connect(sigslots_namespace + std::string("/ros_info"));[m
   sig_warn.connect(sigslots_namespace + std::string("/ros_warn"));[m
   sig_error.connect(sigslots_namespace + std::string("/ros_error"));[m
[31m-  sig_named.connect(sigslots_namespace + std::string("/ros_named"));[m
 [m
   try {[m
     serial.open(parameters.device_port, ecl::BaudRate_115200, ecl::DataBits_8, ecl::StopBits_1, ecl::NoParity);  // this will throw exceptions - NotFoundError, OpenError[m
[36m@@ -119,10 +115,6 @@[m [mvoid Xbot::init(Parameters &parameters) throw (ecl::StandardException)[m
   packet_finder.configure(sigslots_namespace, stx, etx, 1, 256, 1, true);[m
   acceleration_limiter.init(parameters.enable_acceleration_limiter);[m
 [m
[31m-  // in case the user changed these from the defaults[m
[31m-  Battery::capacity = parameters.battery_capacity;[m
[31m-  Battery::low = parameters.battery_low;[m
[31m-  Battery::dangerous = parameters.battery_dangerous;[m
 [m
 [m
   thread.start(&Xbot::spin, *this);[m
[36m@@ -189,7 +181,6 @@[m [mvoid Xbot::spin()[m
         sig_info.emit("device is connected.");[m
         is_connected = true;[m
         serial.block(4000); // blocks by default, but just to be clear![m
[31m-        event_manager.update(is_connected, is_alive);[m
       }[m
       catch (const ecl::StandardException &e)[m
       {[m
[36m@@ -221,7 +212,6 @@[m [mvoid Xbot::spin()[m
         is_alive = false;[m
         sig_debug.emit("Timed out while waiting for incoming bytes.");[m
       }[m
[31m-      event_manager.update(is_connected, is_alive);[m
       continue;[m
     }[m
     else[m
[36m@@ -257,34 +247,15 @@[m [mvoid Xbot::spin()[m
 //        std::cout << "remains: " << data_buffer.size() << " | ";[m
 //        std::cout << "local_buffer: " << local_buffer.size() << " | ";[m
 //        std::cout << std::endl;[m
[31m-        switch (data_buffer[0])[m
[31m-        {[m
[31m-          // these come with the streamed feedback[m
[31m-            case Header::CoreSensors:[m
[31m-//[m
[31m-//            std::cout<<"come into core sensors"<<std::endl;                [m
[31m-                if( !core_sensors.deserialise(data_buffer) )[m
[31m-                    { fixPayload(data_buffer); break; }[m
[31m-                sig_stream_data.emit();[m
[31m-                break;[m
[31m-            case Header::ImuSensors:[m
[31m-//                sig_raw_data_stream.emit(local_buffer);[m
[31m-                if( !imu_sensors.deserialise(data_buffer) )[m
[31m-                    { fixPayload(data_buffer);[m
[31m-                    break; }[m
[31m-//                sig_stream_data.emit();[m
[31m-                break;[m
[31m-[m
[31m-            default: // in the case of unknown or mal-formed sub-payload[m
[31m-                fixPayload(data_buffer);[m
[31m-                break;[m
[31m-        }[m
[32m+[m[32m          if( !core_sensors.deserialise(data_buffer) )[m
[32m+[m[32m              { fixPayload(data_buffer); break; }[m
[32m+[m[32m          sig_stream_data.emit();[m
[32m+[m
       }[m
       //std::cout << "---" << std::endl;[m
       unlockDataAccess();[m
 [m
       is_alive = true;[m
[31m-      event_manager.update(is_connected, is_alive);[m
       last_signal_time.stamp();[m
 //      sig_stream_data.emit();[m
       sendBaseControlCommand(); // send the command packet to mainboard;[m
[36m@@ -351,12 +322,12 @@[m [mfloat Xbot::getHeading() const[m
 [m
 int Xbot::getDebugSensors() const[m
 {[m
[31m-    return (static_cast<int>(core_sensors.data.left_encoder));[m
[32m+[m[32m    return (static_cast<int>(core_sensors.data.front_left_encoder));[m
 }[m
 float Xbot::getAngularVelocity() const[m
 {[m
   // raw data angles are in hundredths of a degree, convert to radians.[m
[31m-  return (static_cast<float>(imu_sensors.data.yaw) / 10.0) * ecl::pi / 180.0;[m
[32m+[m[32m  return (static_cast<float>(core_sensors.data.yaw) / 10.0) * ecl::pi / 180.0;[m
 }[m
 [m
 [m
[36m@@ -376,7 +347,7 @@[m [mvoid Xbot::resetOdometry()[m
   diff_drive.reset();[m
 [m
   // Issue #274: use current imu reading as zero heading to emulate reseting gyro[m
[31m-  heading_offset = (static_cast<float>(imu_sensors.data.yaw) / 10.0) * ecl::pi / 180.0;[m
[32m+[m[32m  heading_offset = (static_cast<float>(core_sensors.data.yaw) / 10.0) * ecl::pi / 180.0;[m
 }[m
 [m
 void Xbot::getWheelJointStates(float &wheel_left_angle, float &wheel_left_angle_rate, float &wheel_right_angle,[m
[36m@@ -398,7 +369,7 @@[m [mvoid Xbot::getWheelJointStates(float &wheel_left_angle, float &wheel_left_angle_[m
  */[m
 void Xbot::updateOdometry(ecl::Pose2D<double> &pose_update, ecl::linear_algebra::Vector3d &pose_update_rates)[m
 {[m
[31m-  diff_drive.update(imu_sensors.data.timestamp, core_sensors.data.left_encoder, core_sensors.data.right_encoder,[m
[32m+[m[32m  diff_drive.update(core_sensors.data.timestamp, core_sensors.data.front_left_encoder, core_sensors.data.front_right_encoder,[m
                       pose_update, pose_update_rates);[m
 }[m
 [m
[1mdiff --git a/xbot_core/xbot_driver/src/test/initialisation.cpp b/xbot_core/xbot_driver/src/test/initialisation.cpp[m
[1mindex f4eddde..d0e6a6a 100644[m
[1m--- a/xbot_core/xbot_driver/src/test/initialisation.cpp[m
[1m+++ b/xbot_core/xbot_driver/src/test/initialisation.cpp[m
[36m@@ -25,10 +25,6 @@[m [mpublic:[m
     parameters.enable_acceleration_limiter = false;[m
     // If your battery levels are showing significant variance from factory defaults, adjust thresholds.[m
     // This will affect the led on the front of the robot as well as when signals are emitted by the driver.[m
[31m-    parameters.battery_capacity = 16.5;[m
[31m-    parameters.battery_low = 14.0;[m
[31m-    parameters.battery_dangerous = 13.2;[m
[31m-[m
     // initialise - it will throw an exception if parameter validation or initialisation fails.[m
     try {[m
       xbot.init(parameters);[m
[1mdiff --git a/xbot_core/xbot_driver/src/test/sigslots.cpp b/xbot_core/xbot_driver/src/test/sigslots.cpp[m
[1mindex b47c389..9554445 100644[m
[1m--- a/xbot_core/xbot_driver/src/test/sigslots.cpp[m
[1m+++ b/xbot_core/xbot_driver/src/test/sigslots.cpp[m
[36m@@ -45,7 +45,7 @@[m [mpublic:[m
    */[m
   void processStreamData() {[m
     xbot::CoreSensors::Data data = xbot.getCoreSensorData();[m
[31m-    std::cout << "Encoders [" <<  data.left_encoder << "," << data.right_encoder << "]" << std::endl;[m
[32m+[m[32m    std::cout << "Encoders [" <<  data.front_left_encoder << "," << data.front_right_encoder << "]" << std::endl;[m
   }[m
 [m
 private:[m
[1mdiff --git a/xbot_msgs/CMakeLists.txt b/xbot_msgs/CMakeLists.txt[m
[1mindex bd3ff51..0ff6de0 100644[m
[1m--- a/xbot_msgs/CMakeLists.txt[m
[1m+++ b/xbot_msgs/CMakeLists.txt[m
[36m@@ -6,19 +6,12 @@[m [madd_message_files(DIRECTORY msg[m
                       FILES [m
                       SensorState.msg[m
                       DebugSensor.msg[m
[31m-                      BumperEvent.msg[m
[31m-                      CliffEvent.msg[m
[31m-                      RobotStateEvent.msg[m
[31m-                      PowerSystemEvent.msg[m
[31m-                      WheelDropEvent.msg[m
[31m-                      ControllerInfo.msg[m
                       DockInfraRed.msg[m
                       KeyboardInput.msg[m
[31m-                      MotorPower.msg[m
[31m-                      ScanAngle.msg[m
                       XbotState.msg[m
[31m-                      face_pose.msg[m
                       ImuNine.msg[m
[32m+[m[32m                      ImuYRP.msg[m
[32m+[m[32m                      Echos.msg[m
                       NaviState.msg[m
                  )[m
 [m
[1mdiff --git a/xbot_msgs/msg/BumperEvent.msg b/xbot_msgs/msg/BumperEvent.msg[m
[1mdeleted file mode 100644[m
[1mindex 530244e..0000000[m
[1m--- a/xbot_msgs/msg/BumperEvent.msg[m
[1m+++ /dev/null[m
[36m@@ -1,16 +0,0 @@[m
[31m-# Provides a bumper event.[m
[31m-# This message is generated whenever a particular bumper is pressed or released.[m
[31m-# Note that, despite bumper field on SensorState messages, state field is not a[m
[31m-# bitmask, but the new state of a single sensor.[m
[31m-[m
[31m-# bumper[m
[31m-uint8 LEFT   = 0[m
[31m-uint8 CENTER = 1[m
[31m-uint8 RIGHT  = 2[m
[31m-[m
[31m-# state[m
[31m-uint8 RELEASED = 0[m
[31m-uint8 PRESSED  = 1[m
[31m-[m
[31m-uint8 bumper[m
[31m-uint8 state[m
[1mdiff --git a/xbot_msgs/msg/CliffEvent.msg b/xbot_msgs/msg/CliffEvent.msg[m
[1mdeleted file mode 100644[m
[1mindex ef72c3f..0000000[m
[1m--- a/xbot_msgs/msg/CliffEvent.msg[m
[1m+++ /dev/null[m
[36m@@ -1,20 +0,0 @@[m
[31m-# Provides a cliff sensor event.[m
[31m-# This message is generated whenever a particular cliff sensor signals that the[m
[31m-# robot approaches or moves away from a cliff.[m
[31m-# Note that, despite cliff field on SensorState messages, state field is not a[m
[31m-# bitmask, but the new state of a single sensor.[m
[31m-[m
[31m-# cliff sensor[m
[31m-uint8 LEFT   = 0[m
[31m-uint8 CENTER = 1[m
[31m-uint8 RIGHT  = 2[m
[31m-[m
[31m-# cliff sensor state[m
[31m-uint8 FLOOR = 0[m
[31m-uint8 CLIFF = 1[m
[31m-[m
[31m-uint8 sensor[m
[31m-uint8 state[m
[31m-[m
[31m-# distance to floor when cliff was detected[m
[31m-uint16 bottom[m
\ No newline at end of file[m
[1mdiff --git a/xbot_msgs/msg/ControllerInfo.msg b/xbot_msgs/msg/ControllerInfo.msg[m
[1mdeleted file mode 100644[m
[1mindex e37b2e2..0000000[m
[1m--- a/xbot_msgs/msg/ControllerInfo.msg[m
[1m+++ /dev/null[m
[36m@@ -1,9 +0,0 @@[m
[31m-# Controller info message, contains PID parameters[m
[31m-[m
[31m-uint8 DEFAULT   =  0[m
[31m-uint8 USER_CONFIGURED =  1[m
[31m-[m
[31m-uint8 type[m
[31m-float64 p_gain #should be positive[m
[31m-float64 i_gain #should be positive[m
[31m-float64 d_gain #should be positive[m
[1mdiff --git a/xbot_msgs/msg/DockInfraRed.msg b/xbot_msgs/msg/DockInfraRed.msg[m
[1mindex 7abf90b..f8031e0 100644[m
[1m--- a/xbot_msgs/msg/DockInfraRed.msg[m
[1m+++ b/xbot_msgs/msg/DockInfraRed.msg[m
[36m@@ -1,14 +1,7 @@[m
 # Docking base ir sensors messages.[m
 # Generated on the proximity of the docking base to assist the automatic docking.[m
 [m
[31m-uint8 NEAR_LEFT   =  1[m
[31m-uint8 NEAR_CENTER =  2[m
[31m-uint8 NEAR_RIGHT  =  4[m
[31m-uint8 FAR_LEFT    =  8[m
[31m-uint8 FAR_CENTER  = 16[m
[31m-uint8 FAR_RIGHT   = 32[m
[31m-[m
 Header header[m
[31m-uint8 left[m
[31m-uint8 center[m
[31m-uint8 right[m
[32m+[m[32muint8 rear_left_infred[m
[32m+[m[32muint8 rear_center_infred[m
[32m+[m[32muint8 rear_right_infred[m
[1mdiff --git a/xbot_msgs/msg/MotorPower.msg b/xbot_msgs/msg/MotorPower.msg[m
[1mdeleted file mode 100644[m
[1mindex 31e2391..0000000[m
[1m--- a/xbot_msgs/msg/MotorPower.msg[m
[1m+++ /dev/null[m
[36m@@ -1,7 +0,0 @@[m
[31m-# Turn on/off Xbot's motors[m
[31m-[m
[31m-# State[m
[31m-uint8 OFF = 0[m
[31m-uint8 ON  = 1[m
[31m-[m
[31m-uint8 state[m
\ No newline at end of file[m
[1mdiff --git a/xbot_msgs/msg/PowerSystemEvent.msg b/xbot_msgs/msg/PowerSystemEvent.msg[m
[1mdeleted file mode 100644[m
[1mindex d5103bc..0000000[m
[1m--- a/xbot_msgs/msg/PowerSystemEvent.msg[m
[1m+++ /dev/null[m
[36m@@ -1,14 +0,0 @@[m
[31m-# Power system events[m
[31m-# This message is generated by important changes in the power system:[m
[31m-#  - plug/unplug to the docking base or adapter[m
[31m-#  - transitions to low/critical battery levels[m
[31m-#  - battery charge completed[m
[31m-[m
[31m-uint8 UNPLUGGED           = 0[m
[31m-uint8 PLUGGED_TO_ADAPTER  = 1[m
[31m-uint8 PLUGGED_TO_DOCKBASE = 2[m
[31m-uint8 CHARGE_COMPLETED    = 3[m
[31m-uint8 BATTERY_LOW         = 4[m
[31m-uint8 BATTERY_CRITICAL    = 5[m
[31m-[m
[31m-uint8 event[m
[1mdiff --git a/xbot_msgs/msg/RobotStateEvent.msg b/xbot_msgs/msg/RobotStateEvent.msg[m
[1mdeleted file mode 100644[m
[1mindex 3aaf712..0000000[m
[1m--- a/xbot_msgs/msg/RobotStateEvent.msg[m
[1m+++ /dev/null[m
[36m@@ -1,7 +0,0 @@[m
[31m-# Provides a robot state event[m
[31m-# This message is generated whenever the robot gets online/offline[m
[31m-[m
[31m-uint8 ONLINE  = 1[m
[31m-uint8 OFFLINE = 0[m
[31m-[m
[31m-uint8 state[m
[1mdiff --git a/xbot_msgs/msg/ScanAngle.msg b/xbot_msgs/msg/ScanAngle.msg[m
[1mdeleted file mode 100644[m
[1mindex 0385066..0000000[m
[1m--- a/xbot_msgs/msg/ScanAngle.msg[m
[1m+++ /dev/null[m
[36m@@ -1,2 +0,0 @@[m
[31m-Header header[m
[31m-float64 scan_angle[m
\ No newline at end of file[m
[1mdiff --git a/xbot_msgs/msg/SensorState.msg b/xbot_msgs/msg/SensorState.msg[m
[1mindex c845d44..94e56dd 100644[m
[1m--- a/xbot_msgs/msg/SensorState.msg[m
[1m+++ b/xbot_msgs/msg/SensorState.msg[m
[36m@@ -1,19 +1,12 @@[m
 # Xbot Sensor Data Messages[m
[31m-#[m
[31m-# For more direct simple interactions (buttons, leds, gyro, motor velocity[m
[31m-# etc) use the other topics. This provides detailed information about the[m
[31m-# entire state package that is transmitted at 50Hz from the robot.[m
[31m-#[m
[31m-[m
[31m-[m
[31m-[m
[31m-[m
[31m-[m
[31m-# Over current states[m
[31m-uint8 OVER_CURRENT_LEFT_WHEEL  = 1[m
[31m-uint8 OVER_CURRENT_RIGHT_WHEEL = 2[m
[31m-uint8 OVER_CURRENT_BOTH_WHEELS = 3[m
[32m+[m[32m#except IMU sensor,echo sensors and infrared[m
 [m
[32m+[m[32m# Over current states, the sum of each channel represents they are over current at the same time, for example, 15 indicates that front_left,front_right,rear_left,rear_right four channels are all over current as 15=1+2+4+8[m
[32m+[m[32muint8 OVER_CURRENT_FRONT_LEFT_WHEEL  = 1[m
[32m+[m[32muint8 OVER_CURRENT_FRONT_RIGHT_WHEEL = 2[m
[32m+[m[32muint8 OVER_CURRENT_REAR_LEFT_WHEEL = 4[m
[32m+[m[32muint8 OVER_CURRENT_REAR_RIGHT_WHEEL = 8[m
[32m+[m[32muint8 OVER_CURRENT_UP_DOWN_WHEEL = 16[m
 [m
 [m
 ###### MESSAGE ######[m
[36m@@ -21,18 +14,32 @@[m [muint8 OVER_CURRENT_BOTH_WHEELS = 3[m
 Header header[m
 [m
 ###################[m
[31m-# Core Packet[m
[32m+[m[32m# Timestamp and battery voltage[m
 ###################[m
 uint32 time_stamp      # milliseconds starting when turning on Xbot (max. 65536, then starts from 0 again)[m
[31m-uint16 left_encoder    # accumulated ticks left wheel starting with turning on Xbot (max. 65535)[m
[31m-uint16 right_encoder   # accumulated ticks right wheel starting with turning on Xbot (max. 65535)[m
[31m-uint8  charger         # see charger states[m
[31m-uint8  battery         # battery voltage in 0.1V (ex. 16.1V -> 161)[m
[32m+[m[32mfloat32  battery_voltage         # battery voltage in float (24V~29.5V)[m
 [m
[32m+[m[32m###################[m
[32m+[m[32m# 5 channel encoders[m
[32m+[m[32m###################[m
[32m+[m[32muint16 front_left_encoder    # accumulated ticks left wheel starting with turning on Xbot (max. 65535)[m
[32m+[m[32muint16 front_right_encoder   # accumulated ticks right wheel starting with turning on Xbot (max. 65535)[m
[32m+[m[32muint16 rear_left_encoder[m
[32m+[m[32muint16 rear_right_encoder[m
[32m+[m[32muint16 up_down_encoder[m
[32m+[m
[32m+[m
[32m+[m[32m###################[m
[32m+[m[32m# 5 channel current[m
[32m+[m[32m###################[m
[32m+[m[32mfloat32 front_left_current[m
[32m+[m[32mfloat32 front_right_current[m
[32m+[m[32mfloat32 rear_left_current[m
[32m+[m[32mfloat32 rear_right_current[m
[32m+[m[32mfloat32 up_down_current[m
 [m
 ###################[m
[31m-# Current Packet[m
[32m+[m[32m# over current indicator[m
 ###################[m
[31m-uint8[] current        # motor current for the left and right motor in 10mA (ex. 12 -> 120mA)[m
 uint8   over_current   # see over current states[m
 [m
[1mdiff --git a/xbot_msgs/msg/WheelDropEvent.msg b/xbot_msgs/msg/WheelDropEvent.msg[m
[1mdeleted file mode 100644[m
[1mindex 34475b5..0000000[m
[1m--- a/xbot_msgs/msg/WheelDropEvent.msg[m
[1m+++ /dev/null[m
[36m@@ -1,16 +0,0 @@[m
[31m-# Provides a wheel drop event.[m
[31m-# This message is generated whenever one of the wheels is dropped (robot fell[m
[31m-# or was raised) or raised (normal condition).[m
[31m-# Note that, despite wheel_drop field on SensorState messages, state field is[m
[31m-# not a bitmask, but the new state of a single sensor.[m
[31m-[m
[31m-# wheel[m
[31m-uint8 LEFT  = 0[m
[31m-uint8 RIGHT = 1[m
[31m-[m
[31m-# state[m
[31m-uint8 RAISED  = 0[m
[31m-uint8 DROPPED = 1[m
[31m-[m
[31m-uint8 wheel[m
[31m-uint8 state[m
[1mdiff --git a/xbot_msgs/msg/XbotState.msg b/xbot_msgs/msg/XbotState.msg[m
[1mindex 6d53476..f308a5d 100644[m
[1m--- a/xbot_msgs/msg/XbotState.msg[m
[1m+++ b/xbot_msgs/msg/XbotState.msg[m
[36m@@ -1,4 +1,5 @@[m
[32m+[m[32mHeader header[m
 uint8 height_percent[m
 uint8 platform_degree[m
 uint8 camera_degree[m
[31m-float32 power[m
[41m+[m
[1mdiff --git a/xbot_msgs/msg/face_pose.msg b/xbot_msgs/msg/face_pose.msg[m
[1mdeleted file mode 100644[m
[1mindex c0017cb..0000000[m
[1m--- a/xbot_msgs/msg/face_pose.msg[m
[1m+++ /dev/null[m
[36m@@ -1,5 +0,0 @@[m
[31m-int32 face_x[m
[31m-int32 face_y[m
[31m-int32 face_z[m
[31m-int32 image_width[m
[31m-int32 image_height[m
[1mdiff --git a/xbot_node/include/xbot_node/xbot_ros.hpp b/xbot_node/include/xbot_node/xbot_ros.hpp[m
[1mindex 65d2918..c399504 100644[m
[1m--- a/xbot_node/include/xbot_node/xbot_ros.hpp[m
[1m+++ b/xbot_node/include/xbot_node/xbot_ros.hpp[m
[36m@@ -55,13 +55,10 @@[m
 #include <sensor_msgs/JointState.h>[m
 #include <sensor_msgs/Imu.h>[m
 #include <ecl/sigslots.hpp>[m
[31m-#include <xbot_msgs/ControllerInfo.h>[m
 #include <xbot_msgs/DockInfraRed.h>[m
[31m-#include <xbot_msgs/MotorPower.h>[m
[31m-#include <xbot_msgs/PowerSystemEvent.h>[m
[31m-#include <xbot_msgs/RobotStateEvent.h>[m
 #include <xbot_msgs/SensorState.h>[m
 #include <xbot_msgs/DebugSensor.h>[m
[32m+[m[32m#include <xbot_msgs/Echos.h>[m
 #include <xbot_driver/xbot.hpp>[m
 #include <xbot_msgs/XbotState.h>[m
 #include <xbot_msgs/ImuNine.h>[m
[36m@@ -96,17 +93,19 @@[m [mprivate:[m
   /*********************[m
    ** Ros Comms[m
    **********************/[m
[31m-  ros::Publisher controller_info_publisher;[m
[31m-  ros::Publisher imu_data_publisher, sensor_state_publisher, joint_state_publisher, dock_ir_publisher, raw_imu_data_publisher;[m
[31m-  ros::Publisher robot_event_publisher;[m
[31m-  ros::Publisher power_event_publisher;[m
[31m-  ros::Publisher raw_data_command_publisher, raw_data_stream_publisher, raw_control_command_publisher;[m
[32m+[m[32m  ros::Publisher imu_data_publisher;[m
[32m+[m[32m  ros::Publisher raw_imu_data_publisher;[m
[32m+[m[32m  ros::Publisher sensor_state_publisher;[m
[32m+[m[32m  ros::Publisher joint_state_publisher;[m
[32m+[m[32m  ros::Publisher dock_ir_publisher;[m
[32m+[m[32m  ros::Publisher echo_data_publisher;[m
[32m+[m[32m  ros::Publisher raw_control_command_publisher;[m
 [m
[31m-  ros::Publisher debug_sensors_publisher, robot_state_publisher;[m
[32m+[m[32m  ros::Publisher debug_sensors_publisher;[m
[32m+[m[32m  ros::Publisher robot_state_publisher;[m
 [m
   ros::Subscriber velocity_command_subscriber;[m
[31m-  ros::Subscriber controller_info_command_subscriber;[m
[31m-  ros::Subscriber motor_power_subscriber, reset_odometry_subscriber;[m
[32m+[m[32m  ros::Subscriber reset_odometry_subscriber;[m
   ros::Subscriber motor_control_subscriber;[m
 [m
   void advertiseTopics(ros::NodeHandle& nh);[m
[36m@@ -117,18 +116,12 @@[m [mprivate:[m
   **********************/[m
   void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr);[m
   void subscribeResetOdometry(const std_msgs::EmptyConstPtr);[m
[31m-  void subscribeMotorPower(const xbot_msgs::MotorPowerConstPtr msg);[m
[31m-  void subscribeControllerInfoCommand(const xbot_msgs::ControllerInfoConstPtr msg);[m
   void subscribeMotorControlCommand(const xbot_msgs::XbotState msg);[m
 [m
   /*********************[m
    ** SigSlots[m
    **********************/[m
   ecl::Slot<> slot_stream_data;[m
[31m-  ecl::Slot<> slot_controller_info;[m
[31m-  ecl::Slot<Command::Buffer&> slot_raw_data_command;[m
[31m-  ecl::Slot<PacketFinder::BufferType&> slot_raw_data_stream;[m
[31m-  ecl::Slot<const std::vector<short>&> slot_raw_control_command;[m
 [m
   /*********************[m
    ** Slot Callbacks[m
[36m@@ -139,17 +132,12 @@[m [mprivate:[m
   void publishRawInertia();[m
   void publishSensorState();[m
   void publishDockIRData();[m
[31m-  void publishControllerInfo();[m
[31m-[m
[32m+[m[32m  void publishEchoData();[m
   void publishDebugSensors();[m
   void publishRobotState();[m
 [m
 [m
 [m
[31m-  void publishRawDataCommand(Command::Buffer &buffer);[m
[31m-  void publishRawDataStream(PacketFinder::BufferType &buffer);[m
[31m-  void publishRawControlCommand(const std::vector<short> &velocity_commands);[m
[31m-[m
 };[m
 [m
 } // namespace xbot[m
[1mdiff --git a/xbot_node/src/library/slot_callbacks.cpp b/xbot_node/src/library/slot_callbacks.cpp[m
[1mindex 3308a70..5ddb1c3 100644[m
[1m--- a/xbot_node/src/library/slot_callbacks.cpp[m
[1m+++ b/xbot_node/src/library/slot_callbacks.cpp[m
[36m@@ -50,6 +50,7 @@[m [mvoid XbotRos::processStreamData() {[m
   publishWheelState();[m
   publishSensorState();[m
   publishDockIRData();[m
[32m+[m[32m  publishEchoData();[m
   publishInertia();[m
   publishRawInertia();[m
   publishDebugSensors();[m
[36m@@ -70,14 +71,18 @@[m [mvoid XbotRos::publishSensorState()[m
     if (sensor_state_publisher.getNumSubscribers() > 0) {[m
       xbot_msgs::SensorState state;[m
       CoreSensors::Data data = xbot.getCoreSensorData();[m
[31m-      state.header.stamp = ros::Time::now();[m
[31m-      state.left_encoder = data.left_encoder;[m
[31m-      state.right_encoder = data.right_encoder;[m
[31m-      state.charger = data.charger;[m
[31m-      state.battery = data.battery;[m
[31m-[m
[31m-[m
[31m-[m
[32m+[m[32m      state.time_stamp = data.timestamp;[m
[32m+[m[32m      state.battery_voltage = data.battery_voltage;[m
[32m+[m[32m      state.front_left_encoder = data.front_left_encoder;[m
[32m+[m[32m      state.front_right_encoder = data.front_right_encoder;[m
[32m+[m[32m      state.rear_left_encoder = data.rear_left_encoder;[m
[32m+[m[32m      state.rear_right_encoder = data.rear_right_encoder;[m
[32m+[m[32m      state.up_down_encoder = data.up_down_encoder;[m
[32m+[m[32m      state.up_down_current = data.up_down_current;[m
[32m+[m[32m      state.front_left_current = data.front_left_current;[m
[32m+[m[32m      state.front_right_current = data.front_right_current;[m
[32m+[m[32m      state.rear_left_current = data.rear_left_current;[m
[32m+[m[32m      state.rear_right_current = data.rear_right_current;[m
 [m
       sensor_state_publisher.publish(state);[m
     }[m
[36m@@ -153,15 +158,15 @@[m [mvoid XbotRos::publishRawInertia()[m
   {[m
     // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature[m
     xbot_msgs::ImuNinePtr msg(new xbot_msgs::ImuNine);[m
[31m-    msg->accex = xbot.getImuSensorData().acce_x;[m
[31m-    msg->gyrox = xbot.getImuSensorData().gyro_x;[m
[31m-    msg->magx = xbot.getImuSensorData().mag_x;[m
[31m-    msg->accey = xbot.getImuSensorData().acce_y;[m
[31m-    msg->gyroy = xbot.getImuSensorData().gyro_y;[m
[31m-    msg->magy = xbot.getImuSensorData().mag_y;[m
[31m-    msg->accez = xbot.getImuSensorData().acce_z;[m
[31m-    msg->gyroz = xbot.getImuSensorData().gyro_z;[m
[31m-    msg->magz = xbot.getImuSensorData().mag_z;[m
[32m+[m[32m    msg->accex = xbot.getCoreSensorData().acce_x;[m
[32m+[m[32m    msg->gyrox = xbot.getCoreSensorData().gyro_x;[m
[32m+[m[32m    msg->magx = xbot.getCoreSensorData().mag_x;[m
[32m+[m[32m    msg->accey = xbot.getCoreSensorData().acce_y;[m
[32m+[m[32m    msg->gyroy = xbot.getCoreSensorData().gyro_y;[m
[32m+[m[32m    msg->magy = xbot.getCoreSensorData().mag_y;[m
[32m+[m[32m    msg->accez = xbot.getCoreSensorData().acce_z;[m
[32m+[m[32m    msg->gyroz = xbot.getCoreSensorData().gyro_z;[m
[32m+[m[32m    msg->magz = xbot.getCoreSensorData().mag_z;[m
 [m
     raw_imu_data_publisher.publish(msg);[m
   }[m
[36m@@ -177,8 +182,8 @@[m [mvoid XbotRos::publishDebugSensors()[m
 [m
         msg->header.frame_id = "encoder";[m
         msg->header.stamp = ros::Time::now();[m
[31m-        msg->data.push_back(data_debug.left_encoder);[m
[31m-        msg->data.push_back(data_debug.right_encoder);[m
[32m+[m[32m        msg->data.push_back(data_debug.front_left_encoder);[m
[32m+[m[32m        msg->data.push_back(data_debug.front_right_encoder);[m
         msg->heading=xbot.getHeading();[m
         debug_sensors_publisher.publish(msg);[m
 //        r.sleep();[m
[36m@@ -195,11 +200,9 @@[m [mvoid XbotRos::publishRobotState()[m
         xbot_msgs::XbotStatePtr msg(new xbot_msgs::XbotState);[m
         CoreSensors::Data data = xbot.getCoreSensorData();[m
 [m
[31m-        msg->power = data.power_voltage;[m
         msg->height_percent = xbot.getHeightPercent();[m
         msg->platform_degree = xbot.getPlatformDegree();[m
         msg->camera_degree = xbot.getCameraDegree();[m
[31m-[m
         robot_state_publisher.publish(msg);[m
         r.sleep();[m
 [m
[36m@@ -216,137 +219,38 @@[m [mvoid XbotRos::publishDockIRData()[m
 [m
             // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature[m
             xbot_msgs::DockInfraRedPtr msg(new xbot_msgs::DockInfraRed);[m
[31m-            CoreSensors::Data data_echo = xbot.getCoreSensorData();[m
[32m+[m[32m            CoreSensors::Data data_infred = xbot.getCoreSensorData();[m
             msg->header.frame_id = "dock_ir_link";[m
             msg->header.stamp = ros::Time::now();[m
[31m-            msg->left=(data_echo.echo_1<=0.07)?1:8;[m
[31m-            msg->center=(data_echo.echo_2<=0.2)?2:16;[m
[31m-            msg->right=(data_echo.echo_3<=0.07)?4:32;[m
[31m-//            msg->danger = ((data_echo.echo_1<=0.07)||data_echo.echo_2<=0.2||data_echo.echo_3<=0.07);[m
[31m-            ROS_ERROR("echo_left:%f|echo_center:%f|echo_right:%f",data_echo.echo_1,data_echo.echo_2,data_echo.echo_3);[m
[32m+[m[32m            msg->rear_left_infred=(data_infred.front_left_echo<=0.07)?1:8;[m
[32m+[m[32m            msg->rear_center_infred=(data_infred.front_center_echo<=0.2)?2:16;[m
[32m+[m[32m            msg->rear_right_infred=(data_infred.front_right_echo<=0.07)?4:32;[m
             dock_ir_publisher.publish(msg);[m
         }[m
     }[m
 }[m
 [m
[31m-/*****************************************************************************[m
[31m-** Non Default Stream Packets[m
[31m-*****************************************************************************/[m
[31m-/**[m
[31m- * @brief Publish fw, hw, sw version information.[m
[31m- *[m
[31m- * The driver will only gather this data when initialising so it is[m
[31m- * important that this publisher is latched.[m
[31m- */[m
[31m-[m
[31m-void XbotRos::publishControllerInfo()[m
[32m+[m[32mvoid XbotRos::publishEchoData()[m
 {[m
[31m-  if (ros::ok())[m
[32m+[m[32m  if(ros::ok())[m
   {[m
[31m-    xbot_msgs::ControllerInfoPtr msg(new xbot_msgs::ControllerInfo);[m
[31m-[m
[32m+[m[32m    if(echo_data_publisher.getNumSubscribers()>0)[m
[32m+[m[32m    {[m
[32m+[m[32m      xbot_msgs::EchosPtr msg(new xbot_msgs::Echos());[m
[32m+[m[32m      CoreSensors::Data data_echo = xbot.getCoreSensorData();[m
[32m+[m[32m      msg->header.frame_id = "echo_link";[m
[32m+[m[32m      msg->header.stamp = ros::Time::now();[m
[32m+[m[32m      int near_left = (data_echo.front_left_echo<=0.07)?1:0;[m
[32m+[m[32m      int near_center = (data_echo.front_center_echo<=0.2)?2:0;[m
[32m+[m[32m      int near_right = (data_echo.front_right_echo<=0.07)?4:0;[m
[32m+[m[32m      msg->near = near_left+near_center+near_right;[m
[32m+[m[32m      echo_data_publisher.publish(msg);[m
 [m
[31m-    controller_info_publisher.publish(msg);[m
[32m+[m[32m    }[m
   }[m
[31m-}[m
[31m-[m
[31m-/*****************************************************************************[m
[31m-** Events[m
[31m-*****************************************************************************/[m
[31m-[m
 [m
 [m
 [m
[31m-[m
[31m-[m
[31m-[m
[31m-[m
[31m-/**[m
[31m- * @brief Prints the raw data stream to a publisher.[m
[31m- *[m
[31m- * This is a lazy publisher, it only publishes if someone is listening. It publishes the[m
[31m- * hex byte values of the raw data commands. Useful for debugging command to protocol[m
[31m- * byte packets to the firmware.[m
[31m- *[m
[31m- * The signal which calls this[m
[31m- * function is sending a copy of the buffer (don't worry about mutexes). Be ideal if we used[m
[31m- * const PacketFinder::BufferType here, but haven't updated PushPop to work with consts yet.[m
[31m- *[m
[31m- * @param buffer[m
[31m- */[m
[31m-void XbotRos::publishRawDataCommand(Command::Buffer &buffer)[m
[31m-{[m
[31m-  if ( raw_data_command_publisher.getNumSubscribers() > 0 ) { // do not do string processing if there is no-one listening.[m
[31m-    std::ostringstream ostream;[m
[31m-    Command::Buffer::Formatter format;[m
[31m-    ostream << format(buffer); // convert to an easily readable hex string.[m
[31m-    std_msgs::String s;[m
[31m-    s.data = ostream.str();[m
[31m-    if (ros::ok())[m
[31m-    {[m
[31m-      raw_data_command_publisher.publish(s);[m
[31m-    }[m
[31m-  }[m
[31m-}[m
[31m-/**[m
[31m- * @brief Prints the raw data stream to a publisher.[m
[31m- *[m
[31m- * This is a lazy publisher, it only publishes if someone is listening. It publishes the[m
[31m- * hex byte values of the raw data (incoming) stream. Useful for checking when bytes get[m
[31m- * mangled.[m
[31m- *[m
[31m- * The signal which calls this[m
[31m- * function is sending a copy of the buffer (don't worry about mutexes). Be ideal if we used[m
[31m- * const PacketFinder::BufferType here, but haven't updated PushPop to work with consts yet.[m
[31m- *[m
[31m- * @param buffer[m
[31m- */[m
[31m-void XbotRos::publishRawDataStream(PacketFinder::BufferType &buffer)[m
[31m-{[m
[31m-  if ( raw_data_stream_publisher.getNumSubscribers() > 0 ) { // do not do string processing if there is no-one listening.[m
[31m-    /*std::cout << "size: [" << buffer.size() << "], asize: [" << buffer.asize() << "]" << std::endl;[m
[31m-    std::cout << "leader: " << buffer.leader << ", follower: " << buffer.follower  << std::endl;[m
[31m-    {[m
[31m-      std::ostringstream ostream;[m
[31m-      PacketFinder::BufferType::Formatter format;[m
[31m-      ostream << format(buffer); // convert to an easily readable hex string.[m
[31m-      //std::cout << ostream.str() << std::endl;[m
[31m-      std_msgs::String s;[m
[31m-      s.data = ostream.str();[m
[31m-      if (ros::ok())[m
[31m-      {[m
[31m-        raw_data_stream_publisher.publish(s);[m
[31m-      }[m
[31m-    }*/[m
[31m-    {[m
[31m-      std::ostringstream ostream;[m
[31m-      ostream << "{ " ;[m
[31m-      ostream << std::setfill('0') << std::uppercase;[m
[31m-      for (unsigned int i=0; i < buffer.size(); i++)[m
[31m-          ostream << std::hex << std::setw(2) << static_cast<unsigned int>(buffer[i]) << " " << std::dec;[m
[31m-      ostream << "}";[m
[31m-      //std::cout << ostream.str() << std::endl;[m
[31m-      std_msgs::StringPtr msg(new std_msgs::String);[m
[31m-      msg->data = ostream.str();[m
[31m-      if (ros::ok())[m
[31m-      {[m
[31m-        raw_data_stream_publisher.publish(msg);[m
[31m-      }[m
[31m-    }[m
[31m-  }[m
[31m-}[m
[31m-[m
[31m-void XbotRos::publishRawControlCommand(const std::vector<short> &velocity_commands)[m
[31m-{[m
[31m-  if ( raw_control_command_publisher.getNumSubscribers() > 0 ) {[m
[31m-    std_msgs::Int16MultiArrayPtr msg(new std_msgs::Int16MultiArray);[m
[31m-    msg->data = velocity_commands;[m
[31m-    if (ros::ok())[m
[31m-    {[m
[31m-      raw_control_command_publisher.publish(msg);[m
[31m-    }[m
[31m-  }[m
[31m-  return;[m
 }[m
 [m
 } // namespace xbot[m
[1mdiff --git a/xbot_node/src/library/subscriber_callbacks.cpp b/xbot_node/src/library/subscriber_callbacks.cpp[m
[1mindex 0e15171..29bb616 100644[m
[1m--- a/xbot_node/src/library/subscriber_callbacks.cpp[m
[1m+++ b/xbot_node/src/library/subscriber_callbacks.cpp[m
[36m@@ -95,40 +95,12 @@[m [mvoid XbotRos::subscribeResetOdometry(const std_msgs::EmptyConstPtr /* msg */)[m
   return;[m
 }[m
 [m
[31m-void XbotRos::subscribeMotorPower(const xbot_msgs::MotorPowerConstPtr msg)[m
[31m-{[m
[31m-  if (msg->state == xbot_msgs::MotorPower::ON)[m
[31m-  {[m
[31m-    ROS_INFO_STREAM("Xbot : Firing up the motors. [" << name << "]");[m
[31m-    xbot.enable();[m
[31m-    odometry.resetTimeout();[m
[31m-  }[m
[31m-  else if (msg->state == xbot_msgs::MotorPower::OFF)[m
[31m-  {[m
[31m-    xbot.disable();[m
[31m-    ROS_INFO_STREAM("Xbot : Shutting down the motors. [" << name << "]");[m
[31m-    odometry.resetTimeout();[m
[31m-  }[m
[31m-  else[m
[31m-  {[m
[31m-    ROS_ERROR_STREAM("Xbot : Motor power command specifies unknown state '" << (unsigned int)msg->state[m
[31m-                     << "'. [" << name << "]");[m
[31m-  }[m
[31m-}[m
[31m-[m
 void XbotRos::subscribeMotorControlCommand(const xbot_msgs::XbotState msg)[m
 {[m
     xbot.setLiftControl(msg.height_percent);[m
     xbot.setPlatformCameraControl(msg.platform_degree, msg.camera_degree);[m
 }[m
 [m
[31m-void XbotRos::subscribeControllerInfoCommand(const xbot_msgs::ControllerInfoConstPtr msg)[m
[31m-{[m
[31m-  if( msg->p_gain < 0.0f ||  msg->i_gain < 0.0f ||  msg->d_gain < 0.0f) {[m
[31m-    ROS_ERROR_STREAM("Xbot : All controller gains should be positive. [" << name << "]");[m
[31m-    return;[m
[31m-  }[m
[31m-  return;[m
[31m-}[m
[32m+[m
 [m
 } // namespace xbot[m
[1mdiff --git a/xbot_node/src/library/xbot_ros.cpp b/xbot_node/src/library/xbot_ros.cpp[m
[1mindex 0a905eb..fa990d2 100644[m
[1m--- a/xbot_node/src/library/xbot_ros.cpp[m
[1m+++ b/xbot_node/src/library/xbot_ros.cpp[m
[36m@@ -59,11 +59,7 @@[m [mnamespace xbot[m
  */[m
 XbotRos::XbotRos(std::string& node_name) :[m
     name(node_name), cmd_vel_timed_out_(false), serial_timed_out_(false),[m
[31m-    slot_controller_info(&XbotRos::publishControllerInfo, *this),[m
[31m-    slot_stream_data(&XbotRos::processStreamData, *this),[m
[31m-    slot_raw_data_command(&XbotRos::publishRawDataCommand, *this),[m
[31m-    slot_raw_data_stream(&XbotRos::publishRawDataStream, *this),[m
[31m-    slot_raw_control_command(&XbotRos::publishRawControlCommand, *this)[m
[32m+[m[32m    slot_stream_data(&XbotRos::processStreamData, *this)[m
 {[m
 [m
 [m
[36m@@ -90,20 +86,12 @@[m [mbool XbotRos::init(ros::NodeHandle& nh)[m
    ** Slots[m
    **********************/[m
   slot_stream_data.connect(name + std::string("/stream_data"));[m
[31m-  slot_controller_info.connect(name + std::string("/controller_info"));[m
[31m-  slot_raw_data_command.connect(name + std::string("/raw_data_command"));[m
[31m-  slot_raw_data_stream.connect(name + std::string("/raw_data_stream"));[m
[31m-  slot_raw_control_command.connect(name + std::string("/raw_control_command"));[m
[31m-[m
   /*********************[m
    ** Driver Parameters[m
    **********************/[m
   Parameters parameters;[m
 [m
   nh.param("acceleration_limiter", parameters.enable_acceleration_limiter, false);[m
[31m-  nh.param("battery_capacity", parameters.battery_capacity, Battery::capacity);[m
[31m-  nh.param("battery_low", parameters.battery_low, Battery::low);[m
[31m-  nh.param("battery_dangerous", parameters.battery_dangerous, Battery::dangerous);[m
 [m
   parameters.sigslots_namespace = name; // name is automatically picked up by device_nodelet parent.[m
   if (!nh.getParam("device_port", parameters.device_port))[m
[36m@@ -284,16 +272,12 @@[m [mvoid XbotRos::advertiseTopics(ros::NodeHandle& nh)[m
 [m
   /*********************[m
   ** Xbot Esoterics[m
[31m-  **********************/[m
[31m-  controller_info_publisher = nh.advertise < xbot_msgs::ControllerInfo > ("controller_info",  100, true); // latched publisher[m
[31m-  power_event_publisher  = nh.advertise < xbot_msgs::PowerSystemEvent > ("events/power_system", 100);[m
[31m-  robot_event_publisher  = nh.advertise < xbot_msgs::RobotStateEvent > ("events/robot_state", 100, true); // also latched[m
[32m+[m[32m  **********************/[m[41m  [m
   sensor_state_publisher = nh.advertise < xbot_msgs::SensorState > ("sensors/core", 100);[m
   dock_ir_publisher = nh.advertise < xbot_msgs::DockInfraRed > ("sensors/dock_ir", 100);[m
[32m+[m[32m  echo_data_publisher = nh.advertise < xbot_msgs::Echos > ("sensors/echo", 100);[m
   imu_data_publisher = nh.advertise < sensor_msgs::Imu > ("sensors/imu_data", 100);[m
   raw_imu_data_publisher = nh.advertise < xbot_msgs::ImuNine > ("sensors/imu_data_raw", 100);[m
[31m-  raw_data_command_publisher = nh.advertise< std_msgs::String > ("debug/raw_data_command", 100);[m
[31m-  raw_data_stream_publisher = nh.advertise< std_msgs::String > ("debug/raw_data_stream", 100);[m
   raw_control_command_publisher = nh.advertise< std_msgs::Int16MultiArray > ("debug/raw_control_command", 100);[m
 [m
   debug_sensors_publisher = nh.advertise < xbot_msgs::DebugSensor> ("debug/sensors_data",100);[m
[36m@@ -308,8 +292,6 @@[m [mvoid XbotRos::subscribeTopics(ros::NodeHandle& nh)[m
 {[m
   velocity_command_subscriber = nh.subscribe(std::string("commands/velocity"), 10, &XbotRos::subscribeVelocityCommand, this);[m
   reset_odometry_subscriber = nh.subscribe("commands/reset_odometry", 10, &XbotRos::subscribeResetOdometry, this);[m
[31m-  motor_power_subscriber = nh.subscribe("commands/motor_power", 10, &XbotRos::subscribeMotorPower, this);[m
[31m-  controller_info_command_subscriber =  nh.subscribe(std::string("commands/controller_info"), 10, &XbotRos::subscribeControllerInfoCommand, this);[m
   motor_control_subscriber = nh.subscribe("commands/other_motors", 10, &XbotRos::subscribeMotorControlCommand, this);[m
 }[m
 [m
[1mdiff --git a/xbot_node/src/nodelet/xbot_nodelet.cpp b/xbot_node/src/nodelet/xbot_nodelet.cpp[m
[1mindex 17a4dff..37ff15c 100644[m
[1m--- a/xbot_node/src/nodelet/xbot_nodelet.cpp[m
[1m+++ b/xbot_node/src/nodelet/xbot_nodelet.cpp[m
[36m@@ -74,7 +74,7 @@[m [mpublic:[m
 private:[m
   void update()[m
   {[m
[31m-    ros::Rate spin_rate(10);[m
[32m+[m[32m    ros::Rate spin_rate(10);//check the state every 0.1sec[m
     while (!shutdown_requested_ && ros::ok() && xbot_->update())[m
     {[m
       spin_rate.sleep();[m
