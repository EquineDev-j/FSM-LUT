/*
 * fsm.h
 *
 *  Created on: Feb 1, 2023
 *      Author: bennettjj
 *  Finite state machine lookup table. States, events, logic
 */

#ifndef FSM_H_
#define FSM_H_

// Function Prototypes
void Fan_I (void);
void Fan_0 (void);
void Comp_I(void);
void Comp_0(void);

// create sets of values with meaningful name
// only valid in scope where declared
typedef enum {
    S0, // on
    S1, // off
    S2, // defrost
}state; // enum type 'state'

// create sets of values with meaningful name
// only valid in scope where declared
typedef enum {
    sp_H, // setpoint high
    sp_L, // setpoint low
    Ice
}event; // enum type 'event'

// creating new Type 'fp'. A pointer to a function.
// takes not input, outputs nothing
typedef void (*fp)(void);

// storing related values, of possibly different types
// can mix structures and arrays
typedef struct{
    state nextstate;
    fp action1; // action is a pointer type 'fp'
    fp action2;
}stateElement; // structure type 'stateElement'

// table array stateElement structure. Initialize state table
stateElement stateTable [3][3] = {
                                  {{S0, Fan_0, Comp_0},{S1, Fan_I, Comp_I},{S2, Fan_I, Comp_0}},
                                  {{S0, Fan_0, Comp_0},{S1, Fan_I, Comp_I},{S2, Fan_I, Comp_0}},
                                  {{S0, Fan_0, Comp_0},{S1, Fan_I, Comp_I},{S2, Fan_I, Comp_0}},
                                  // S0 = Off           // S1= on           // ICE

};

//state machine update function handler
state stateUpdate (state current, event input){
    stateElement currentstate = stateTable[current][input]; // Use LUT

    // run proper action function
    (*currentstate.action1)();
    (*currentstate.action2)();

    // return next state info
    return currentstate.nextstate;
}

// OUTPUTS
void Fan_I (void){P2->OUT|=BIT2;} // B_LED on
void Fan_0 (void){P2->OUT&=~BIT2;} // B_LED off
void Comp_I(void){P2->OUT|=BIT0;} // R_LED on
void Comp_0(void){P2->OUT&=~BIT0;} // R_LED off

#endif /* FSM_H_ */
