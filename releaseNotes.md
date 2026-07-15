# 0.0.1
* Ethercat specific control built

# 0.0.2
* adding routine lib
* safe conversion betwween rad and pulses
* encoder setting added to dataobjcts
* motor config -> pos, vel, acl, dir 
* Joint specific motor configs allowed 
* mover app added on top of controller 
* app now control the main loop
* motor activate and deactivate functions added controlled through app
* motor control cycle now based on state machine conditions
* position write only when enable is truly on in control loop 
* pdo value buffer is atomic 

# 0.0.3
* 





### The manual's required sequence

| Step | Write to Control Word (6040h) | Then mask status word & check                                             |
| ---- | ----------------------------- | ------------------------------------------------------------------------- |
| 1    | `0x0000`                      | `status_word & 0x250 == 0x250`                                            |
| 2    | `0x0006`                      | `status_word & 0x231 == 0x231`                                            |
| 3    | `0x0007`                      | `status_word & 0x233 == 0x233`                                            |
| 4    | `0x000F`                      | `status_word & 0x237 == 0x237` (→ **Operation Enabled**, motor is now on) |


it must transition through the states
   EC_AL_STATE_INIT = 1, /**< Init. */
    EC_AL_STATE_PREOP = 2, /**< Pre-operational. */
    EC_AL_STATE_SAFEOP = 4, /**< Safe-operational. */
    EC_AL_STATE_OP = 8, /**< Operational. */