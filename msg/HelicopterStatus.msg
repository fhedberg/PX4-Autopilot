uint64 timestamp        # time since system start (microseconds)

# Spool state
uint8 SPOOL_STATE_SHUT_DOWN = 0
uint8 SPOOL_STATE_GROUND_IDLE = 1
uint8 SPOOL_STATE_SPOOLING_UP = 2
uint8 SPOOL_STATE_THROTTLE_UNLIMITED = 3
uint8 SPOOL_STATE_SPOOLING_DOWN = 4

uint8 spool_state       # current spool state
float32 throttle_output # current throttle output [0, 1]
float32 spoolup_progress # spoolup progress [0, 1]
