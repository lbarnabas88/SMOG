/*
 * log.hpp
 *
 *  Created on: 2013.06.25.
 *      Author: Baarnus
 */

#ifndef LOG_HPP_
#define LOG_HPP_

// Debug output
#ifndef NDEBUG
#include <iostream>
#define DBOUT(x) std::cout << x << std::endl
#define DBERR(x) std::cerr << x << std::endl
#else
#define DBOUT(x) void(0)
#define DBERR(x) void(0)
#endif

#endif /* LOG_HPP_ */
