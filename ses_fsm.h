#ifndef SES_FSM_H_
#define SES_FSM_H_

#include <stdbool.h>
#include "ses_common.h"
#include "ses_scheduler.h"

typedef uint32_t systemTime_t;

struct time_t {
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint16_t milli;
};

enum {
	RET_HANDLED,
	RET_IGNORED,
	RET_TRANSITION,
};

enum {

	ENTRY,EXIT,JOYSTICK_PRESSED, ROTARY_PRESSED
};




typedef struct fsm_s Fsm; //< typedef for alarm clock state machine
typedef struct event_s Event; //< event type for alarm clock fsm
/** return values */






typedef uint8_t fsmReturnStatus; //< typedef to be used with above enum

/** typedef for state event handler functions */
typedef fsmReturnStatus (*State)(Fsm *, const Event*);

struct fsm_s {
	State state; //< current state, pointer to event handler
	bool isAlarmEnabled; //< flag for the alarm status
	struct time_t timeSet; //< multi-purpose var for system time and alarm time
};

typedef struct event_s {
	uint8_t signal; //< identifies the type of event
} Event;

inline static void fsm_dispatch(Fsm* fsm, const Event* event) {
	static Event entryEvent = { .signal = ENTRY };
	static Event exitEvent = { .signal = EXIT };
	State s = fsm->state;
	fsmReturnStatus r = fsm->state(fsm, event);
	if (r == RET_TRANSITION) {
		s(fsm, &exitEvent); //< call exit action of last state
		fsm->state(fsm, &entryEvent); //< call entry action of new state
	}
}

/* sets and calls initial state of state machine */
inline static void fsm_init(Fsm* fsm, State init) {
	//... other initialization
	Event entryEvent = { .signal = ENTRY };
	fsm->state = init;
	fsm->state(fsm, &entryEvent);
}

#endif /* SES_FSM_H_ */
