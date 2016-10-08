/**
 * @file /include/xbot_driver/macros.hpp
 *
 * @brief Macros for xbot_driver.
 *
 * @date April, 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef XBOT_DRIVER_MACROS_HPP_
#define XBOT_DRIVER_MACROS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <ecl/config/ecl.hpp>

/*****************************************************************************
** Declspecs
*****************************************************************************/

/*
 * Import/exports symbols for the library
 */
#ifdef ECL_HAS_SHARED_LIBS // ecl is being built around shared libraries
  #ifdef xbot_EXPORTS // we are building a shared lib/dll
    #define xbot_PUBLIC ECL_HELPER_EXPORT
    #define EXP_TEMPLATE
  #else // we are using shared lib/dll
    #define xbot_PUBLIC ECL_HELPER_IMPORT
    #define EXP_TEMPLATE extern
  #endif
  #define xbot_LOCAL ECL_HELPERS_LOCAL
#else // ecl is being built around static libraries
  #define xbot_PUBLIC
  #define xbot_LOCAL
  #define EXP_TEMPLATE
#endif

#endif /* XBOT_DRIVER_MACROS_HPP_ */
