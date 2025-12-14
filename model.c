typedef struct {
    int32_t current_mA;
    millis_t current_millis;

    contactors_sm_t contactor_sm;
    contactors_requests contactor_req;
    
  
} bms_model_t;


