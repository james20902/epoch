typedef enum {
    UNDEFINED,
    INITIALIZE,
    WAITING_DATA,
    WARMUP,
    GROUND, // this is the true "waiting" state
    LAUNCH_DETECTION, // same as ground, but with a faster loop rate to detect sudden acceleration/high jerk
    BOOST,
    COAST_ASCENT,
    COAST_DESCENT,
    PARACHUTE_DESCENT
} system_state;