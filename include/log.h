/*
Minimal logging shim replacing rosconsole (ROS_INFO / ROS_WARN / ROS_ERROR).
printf-style macros keep the exact message formats of the original call sites;
_STREAM variants mirror the ROS_*_STREAM macros.
*/

#ifndef FAST_CALIB_LOG_H
#define FAST_CALIB_LOG_H

#include <cstdio>
#include <iostream>

#include "color.h"

#define LOG_INFO(...)                    \
  do {                                   \
    std::printf("[ INFO] " __VA_ARGS__); \
    std::printf("\n");                   \
  } while (0)

#define LOG_WARN(...)                       \
  do {                                      \
    std::printf(YELLOW "[ WARN] ");         \
    std::printf(__VA_ARGS__);               \
    std::printf(RESET "\n");                \
  } while (0)

#define LOG_ERROR(...)                          \
  do {                                          \
    std::fprintf(stderr, RED "[ERROR] ");       \
    std::fprintf(stderr, __VA_ARGS__);          \
    std::fprintf(stderr, RESET "\n");           \
  } while (0)

#define LOG_INFO_STREAM(args) \
  (std::cout << "[ INFO] " << args << std::endl)
#define LOG_WARN_STREAM(args) \
  (std::cout << YELLOW << "[ WARN] " << args << RESET << std::endl)
#define LOG_ERROR_STREAM(args) \
  (std::cerr << RED << "[ERROR] " << args << RESET << std::endl)

#endif  // FAST_CALIB_LOG_H
