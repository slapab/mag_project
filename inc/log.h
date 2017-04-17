#ifndef LOG_H_
#define LOG_H_
#include <stdint.h>
#include <stdbool.h>
#include "log_config.h"





// This is always public function - used to initialize logging module
void initLog(void);

#ifdef DEBUG
bool logLock(void);
void logUnlock(void);
void logWriteMsg(const char* const msg);
void logWriteInt(int32_t val, uint_fast8_t base);
void logWriteUInt(uint32_t val, uint_fast8_t base);
#endif

// APP log
#if defined (DEBUG) && (1 == USE_APP_LOG)

#define APP_LOG_HEADER() logWriteMsg("APP:  ")
#define APP_LOG_NL() logWriteMsg("\r\n")
#define APP_LOG_MSG(msg) logWriteMsg((msg))
#define APP_LOG_INT(val, base) logWriteInt((val), (base))
#define APP_LOG_UINT(val, base) logWriteUInt((val), (base))
#define APP_LOG_LOCK() if (true == logLock()) { APP_LOG_HEADER()
#define APP_LOG_UNLOCK() APP_LOG_NL(); logUnlock(); }

#define APP_LOG_LU_MSG(msg) APP_LOG_LOCK(); APP_LOG_MSG((msg)); APP_LOG_UNLOCK();

#else

#define APP_LOG_LOCK()
#define APP_LOG_UNLOCK()
#define APP_LOG_HEADER()
#define APP_LOG_NL()
#define APP_LOG_MSG(msg)
#define APP_LOG_INT(val, base)
#define APP_LOG_UINT(val, base)
#define APP_LOG_LU_MSG(msg)

#endif //DEBUG



// Module specific logging:

// SD module log
#if defined (DEBUG) && (1 == USE_SD_LOG)
#define SD_LOG_HEADER() logWriteMsg("SD:   ")
#define SD_LOG_NL() logWriteMsg("\r\n")
#define SD_LOG_MSG(msg) logWriteMsg((msg))
#define SD_LOG_INT(val, base) logWriteInt((val), (base))
#define SD_LOG_UINT(val, base) logWriteUInt((val), (base))
#define SD_LOG_LOCK() if (true == logLock()) { SD_LOG_HEADER()
#define SD_LOG_UNLOCK() SD_LOG_NL(); logUnlock(); }

#define SD_LOG_LU_MSG(msg) SD_LOG_LOCK(); SD_LOG_MSG((msg)); SD_LOG_UNLOCK();

#else

#define SD_LOG_LOCK()
#define SD_LOG_UNLOCK()
#define SD_LOG_HEADER()
#define SD_LOG_NL()
#define SD_LOG_MSG(msg)
#define SD_LOG_INT(val, base)
#define SD_LOG_UINT(val, base)
#define SD_LOG_LU_MSG(msg)

#endif

// Display module log
#if defined (DEBUG) && (1 == USE_DISPLAY_LOG)

#define DISP_LOG_HEADER() logWriteMsg("DISP: ")
#define DISP_LOG_NL() logWriteMsg("\r\n")
#define DISP_LOG_MSG(msg) logWriteMsg((msg))
#define DISP_LOG_INT(val, base) logWriteInt((val), (base))
#define DISP_LOG_UINT(val, base) logWriteUInt((val), (base))
#define DISP_LOG_LOCK() if (true == logLock()) { DISP_LOG_HEADER()
#define DISP_LOG_UNLOCK() DISP_LOG_NL(); logUnlock(); }

#define DISP_LOG_LU_MSG(msg) DISP_LOG_LOCK(); DISP_LOG_MSG((msg)); DISP_LOG_UNLOCK();

#else

#define DISP_LOG_LOCK()
#define DISP_LOG_UNLOCK()
#define DISP_LOG_HEADER()
#define DISP_LOG_NL()
#define DISP_LOG_MSG(msg)
#define DISP_LOG_INT(val, base)
#define DISP_LOG_UINT(val, base)
#define DISP_LOG_LU_MSG(msg)

#endif

// MAG3110 module log
#if defined (DEBUG) && (1 == USE_MAG3110_LOG)

#define MAG_LOG_HEADER() logWriteMsg("MAG:  ")
#define MAG_LOG_NL() logWriteMsg("\r\n")
#define MAG_LOG_MSG(msg) logWriteMsg((msg))
#define MAG_LOG_INT(val, base) logWriteInt((val), (base))
#define MAG_LOG_UINT(val, base) logWriteUInt((val), (base))
#define MAG_LOG_LOCK() if (true == logLock()) { MAG_LOG_HEADER()
#define MAG_LOG_UNLOCK() MAG_LOG_NL(); logUnlock(); }

#define MAG_LOG_LU_MSG(msg) MAG_LOG_LOCK(); MAG_LOG_MSG((msg)); MAG_LOG_UNLOCK();

#else

#define MAG_LOG_LOCK()
#define MAG_LOG_UNLOCK()
#define MAG_LOG_HEADER()
#define MAG_LOG_NL()
#define MAG_LOG_MSG(msg)
#define MAG_LOG_INT(val, base)
#define MAG_LOG_UINT(val, base)
#define MAG_LOG_LU_MSG(msg)

#endif

#endif //LOG_H_
