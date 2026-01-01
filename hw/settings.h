// System PWM frequency (ideally higher than audible range)
#define PWM_FREQUENCY_HZ    20000

// Initial PWM level (starting above 1 means there will be a period of full on
// before PWM starts kicking in)
#define CONTACTOR_PWM_INITIAL   3.0f // out of 1.0
// Minimum PWM level (needs to be high enough that the contactors don't drop out)
#define CONTACTOR_PWM_MIN       0.125f // out of 1.0
// How much the level is decremented each timestep
#define CONTACTOR_PWM_DECREMENT 1.0f //((CONTACTOR_PWM_INITIAL - CONTACTOR_PWM_MIN) / 1.0f) //ramp down over 160 timesteps
