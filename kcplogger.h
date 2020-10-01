//
// UnitX Edgeware AB 2020
//

#ifndef KCP_CPP_LOGGER_H
#define KCP_CPP_LOGGER_H

#include <iostream>
#include <sstream>

#define LOGG_NOTIFY (unsigned)1
#define LOGG_WARN (unsigned)2
#define LOGG_ERROR (unsigned)4
#define LOGG_FATAL (unsigned)8
#define LOGG_MASK  LOGG_NOTIFY | LOGG_WARN | LOGG_ERROR | LOGG_FATAL //What to logg?

//#define DEBUG

#ifdef DEBUG
#define KCP_LOGGER(l,g,f) \
{ \
std::ostringstream a; \
if (g == (LOGG_NOTIFY & (LOGG_MASK))) {a << "Notification: ";} \
else if (g == (LOGG_WARN & (LOGG_MASK))) {a << "Warning: ";} \
else if (g == (LOGG_ERROR & (LOGG_MASK))) {a << "Error: ";} \
else if (g == (LOGG_FATAL & (LOGG_MASK))) {a << "Fatal: ";} \
if (a.str().length()) { \
if (l) {a << __FILE__ << " " << __LINE__ << " ";} \
a << f << std::endl; \
std::cout << a.str(); \
} \
}
#else
#define KCP_LOGGER(l,g,f)
#endif

#endif //KCP_CPP_LOGGER_H
