#include "hw/time.h"
#include "state_machines/contactors.h"

#include <stdint.h>

typedef struct {
    int32_t current_mA;
    millis_t current_millis;

    contactors_sm_t contactor_sm;
    contactors_requests_t contactor_req;
    
  
} bms_model_t;

extern bms_model_t model;