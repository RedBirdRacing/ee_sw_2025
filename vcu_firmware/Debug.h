#ifndef DEBUG_H
#define DEBUG_H

// === Debug Flags ===
#define DEBUG true // Oveall debug functionality
#define DEBUG_PEDAL true && DEBUG
#define DEBUG_SIGNAL_PROC false && DEBUG
#define DEBUG_GENERAL true && DEBUG
#define DEBUG_PEDAL true && DEBUG
#define DEBUG_CAN true && DEBUG
#define DEBUG_STATUS true && DEBUG

#if DEBUG_PEDAL
#define DBG_PEDAL(x) Serial.print(x)
#define DBGLN_PEDAL(x) Serial.println(x)
#else
#define DBG_PEDAL(x)
#define DBGLN_PEDAL(x)
#endif

#if DEBUG_SIGNAL_PROC
#define DBG_SIG(x) Serial.print(x)
#define DBGLN_SIG(x) Serial.println(x)
#else
#define DBG_SIG(x)
#define DBGLN_SIG(x)
#endif

#if DEBUG_GENERAL
#define DBG_GENERAL(x) Serial.print(x)
#define DBGLN_GENERAL(x) Serial.println(x)
#else
#define DBG_GENERAL(x)
#define DBGLN_GENERAL(x)
#endif

#if DEBUG_PEDAL
#define DBG_PEDAL(x) Serial.print(x)
#define DBGLN_PEDAL(x) Serial.println(x)
#else
#define DBG_PEDAL(x)
#define DBGLN_PEDAL(x)
#endif

#if DEBUG_CAN
#define DBG_CAN(x) Serial.print(x)
#define DBGLN_CAN(x) Serial.println(x)
#else
#define DBG_CAN(x)
#define DBGLN_CAN(x)
#endif

#if DEBUG_STATUS
#define DBG_STATUS(x) Serial.print(x)
#define DBGLN_STATUS(x) Serial.println(x)
#else
#define DBG_STATUS(x)
#define DBGLN_STATUS(x)
#endif


#endif // DEBUG_H