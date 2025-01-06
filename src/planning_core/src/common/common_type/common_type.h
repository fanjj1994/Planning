#ifndef COMMON_TYPE_H
#define COMMON_TYPE_H

#include <cstdint>

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;

typedef float float32;
typedef double float64;
#ifdef __cplusplus
typedef long double float128;
#endif

typedef char char8;

typedef bool boolean;

typedef std::string std_string;

#endif // !COMMON_TYPE_H