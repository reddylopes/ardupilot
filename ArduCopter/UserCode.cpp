#include "Copter.h"
#include "GCS_Mavlink.h"

/*
    case MAVLINK_MSG_ID_MULTIUAV_TASKEND:
    {
        mavlink_multiuav_taskend_t packet;
        mavlink_multiuav_taskend_decode(msg, &packet);
        break;
    }
 */

SCAutomaton::Event SCAutomaton::answer("ANSWER", true);
SCAutomaton::Event SCAutomaton::requestCharge("REQUEST_CHARGE", true);
SCAutomaton::Event SCAutomaton::taskEnd("TASK_END", false);
SCAutomaton::Event SCAutomaton::returnToBase("RETURN_TO_BASE", false);
SCAutomaton::Event SCAutomaton::assignment("ASSIGNMENT", false);
#define CRITICAL_SOC_VALUE 20
SCAutomaton::Event SCAutomaton::criticalSoc("CRITICAL_SOC", false);
#define ACCEPTABLE_SOC_VALUE 99
SCAutomaton::Event SCAutomaton::acceptableSoc("ACCEPTABLE_SOC", false);

std::vector<SCAutomaton::Event> SCAutomaton::go(20);
std::vector<SCAutomaton::Event> SCAutomaton::arrive(20);

void Copter::create_automata(uint64_t number_of_bases, uint64_t initial_base) {

	for (uint64_t b = 1; b <= number_of_bases; b++)
	{
		SCAutomaton::Event goI("GO" + SSTR(b), true);
		SCAutomaton::Event arriveI("ARRIVE" + SSTR(b), false);
		SCAutomaton::go.push_back(goI);
		SCAutomaton::arrive.push_back(arriveI);
	}

	// Geographic Automaton

	SCAutomaton::Language geographicAutomatonLanguage;
	SCAutomaton::State geographicAutomatonInitialState;
	SCAutomaton::StateVector geographicAutomatonStates;
	SCAutomaton::StateVector geographicAutomatonMarkedStates;
	SCAutomaton::TransitionVector geographicAutomatonTransitionFunction;

	geographicAutomatonLanguage.push_back(SCAutomaton::answer);

	SCAutomaton::State away("AWAY", false);
	geographicAutomatonStates.push_back(away);

	SCAutomaton::Transition transition(SCAutomaton::answer, away, away);
	geographicAutomatonTransitionFunction.push_back(transition);

	for (uint64_t b = 1; b <= number_of_bases; b++)
	{
		SCAutomaton::State baseState("BASE" + SSTR(b), true);
		geographicAutomatonStates.push_back(baseState);
		geographicAutomatonMarkedStates.push_back(baseState);

		if (b == initial_base)
		{
			baseState.setInitial();
			geographicAutomatonInitialState = baseState;
		}

		geographicAutomatonLanguage.push_back(SCAutomaton::arrive[b-1]);

		SCAutomaton::Transition transition2(SCAutomaton::answer, baseState, away);
		geographicAutomatonTransitionFunction.push_back(transition2);
		SCAutomaton::Transition transition3(SCAutomaton::arrive[b-1], away, baseState);
		geographicAutomatonTransitionFunction.push_back(transition3);
	}

	geographicAutomaton.setFields(
			"Geographic Automaton",
			geographicAutomatonStates,
			geographicAutomatonMarkedStates,
			geographicAutomatonInitialState,
			geographicAutomatonTransitionFunction,
			geographicAutomatonLanguage
	);

	// Occupational Automaton

	SCAutomaton::Language occupationalAutomatonLanguage;
	SCAutomaton::State occupationalAutomatonInitialState;
	SCAutomaton::StateVector occupationalAutomatonStates;
	SCAutomaton::StateVector occupationalAutomatonMarkedStates;
	SCAutomaton::TransitionVector occupationalAutomatonTransitionFunction;

	occupationalAutomatonLanguage.push_back(SCAutomaton::answer);
	occupationalAutomatonLanguage.push_back(SCAutomaton::taskEnd);
	occupationalAutomatonLanguage.push_back(SCAutomaton::criticalSoc);
	occupationalAutomatonLanguage.push_back(SCAutomaton::acceptableSoc);
	occupationalAutomatonLanguage.push_back(SCAutomaton::requestCharge);
	occupationalAutomatonLanguage.push_back(SCAutomaton::assignment);
	occupationalAutomatonLanguage.push_back(SCAutomaton::returnToBase);

	SCAutomaton::State idle("IDLE", true);
	SCAutomaton::State busyT("BUSY_ASSIGNMENT", false);
	SCAutomaton::State busyR("BUSY_RECHARGING", false);

	occupationalAutomatonStates.push_back(idle);
	occupationalAutomatonStates.push_back(busyT);
	occupationalAutomatonStates.push_back(busyR);

	occupationalAutomatonMarkedStates.push_back(idle);

	occupationalAutomatonInitialState = idle;

	SCAutomaton::Transition transition4(SCAutomaton::answer, idle, busyT);
	occupationalAutomatonTransitionFunction.push_back(transition4);
	SCAutomaton::Transition transition5(SCAutomaton::taskEnd, busyT, idle);
	occupationalAutomatonTransitionFunction.push_back(transition5);
	SCAutomaton::Transition transition6(SCAutomaton::criticalSoc, idle, busyR);
	occupationalAutomatonTransitionFunction.push_back(transition6);
	SCAutomaton::Transition transition7(SCAutomaton::acceptableSoc, busyR, idle);
	occupationalAutomatonTransitionFunction.push_back(transition7);
	SCAutomaton::Transition transition8(SCAutomaton::requestCharge, busyR, busyR);
	occupationalAutomatonTransitionFunction.push_back(transition8);
	SCAutomaton::Transition transition9(SCAutomaton::assignment, idle, idle);
	occupationalAutomatonTransitionFunction.push_back(transition9);
	SCAutomaton::Transition transition10(SCAutomaton::returnToBase, idle, idle);
	occupationalAutomatonTransitionFunction.push_back(transition10);

	occupationalAutomaton.setFields(
		"Occupational Automaton",
		occupationalAutomatonStates,
		occupationalAutomatonMarkedStates,
		occupationalAutomatonInitialState,
		occupationalAutomatonTransitionFunction,
		occupationalAutomatonLanguage
	);

	// Dynamics Automaton

	SCAutomaton::Language dynamicAutomatonLanguage;
	SCAutomaton::State dynamicAutomatonInitialState;
	SCAutomaton::StateVector dynamicAutomatonStates;
	SCAutomaton::StateVector dynamicAutomatonMarkedStates;
	SCAutomaton::TransitionVector dynamicAutomatonTransitionFunction;

	dynamicAutomatonLanguage.push_back(SCAutomaton::answer);

	SCAutomaton::State stopped("STATIONARY", true);
	stopped.setInitial();
	dynamicAutomatonStates.push_back(stopped);
	dynamicAutomatonMarkedStates.push_back(stopped);
	dynamicAutomatonInitialState = stopped;

	SCAutomaton::State assigned("FLYING_ASSIGNMENT", false);
	dynamicAutomatonStates.push_back(assigned);

	SCAutomaton::Transition transition11(SCAutomaton::answer, stopped, assigned);
	dynamicAutomatonTransitionFunction.push_back(transition11);
	SCAutomaton::Transition transition12(SCAutomaton::answer, assigned, assigned);
	dynamicAutomatonTransitionFunction.push_back(transition12);

	for (uint64_t b = 1; b <= number_of_bases; b++)
	{
		SCAutomaton::State baseState("FLYING_BASE" + SSTR(b), true);
		geographicAutomatonStates.push_back(baseState);

		dynamicAutomatonLanguage.push_back(SCAutomaton::go[b-1]);
		dynamicAutomatonLanguage.push_back(SCAutomaton::arrive[b-1]);

		SCAutomaton::Transition transition13(SCAutomaton::go[b-1], assigned, baseState);
		dynamicAutomatonTransitionFunction.push_back(transition13);
		SCAutomaton::Transition transition14(SCAutomaton::arrive[b-1], baseState, stopped);
		dynamicAutomatonTransitionFunction.push_back(transition14);
		SCAutomaton::Transition transition15(SCAutomaton::answer, baseState, assigned);
		dynamicAutomatonTransitionFunction.push_back(transition15);
	}

	dynamicAutomaton.setFields(
		"Dynamics Automaton",
		dynamicAutomatonStates,
		dynamicAutomatonMarkedStates,
		dynamicAutomatonInitialState,
		dynamicAutomatonTransitionFunction,
		dynamicAutomatonLanguage
	);

	for (unsigned char i = 0; i < copter.gcs().num_gcs(); i++) {
		if (copter.gcs().chan(i).initialised) {
			copter.gcs().chan(i).send_current_states(
				geographicAutomaton.getCurrentGeographicState(),
				occupationalAutomaton.getCurrentOccupationalState(),
				dynamicAutomaton.getCurrentDynamicState(),
				base_occupied);
		}
	}

	copter.gcs().send_text(MAV_SEVERITY_INFO, "REDDY - Sending GEOSTATE %s", geographicAutomaton.getCurrentState().getLabel().c_str());
	copter.gcs().send_text(MAV_SEVERITY_INFO, "REDDY - Sending OCCSTATE %s", occupationalAutomaton.getCurrentState().getLabel().c_str());
	copter.gcs().send_text(MAV_SEVERITY_INFO, "REDDY - Sending DYNSTATE %s", dynamicAutomaton.getCurrentState().getLabel().c_str());
	copter.gcs().send_text(MAV_SEVERITY_INFO, "REDDY - Sending BASE_OCCUPIED %i", copter.base_occupied);
}

void Copter::triggerTransitions(SCAutomaton::Event e) {
	copter.gcs().send_text(MAV_SEVERITY_INFO, "REDDY - Triggering event %s", e.getLabel().c_str());

	geographicAutomaton.triggerTransition(e);
	occupationalAutomaton.triggerTransition(e);
	dynamicAutomaton.triggerTransition(e);

	for (unsigned char i = 0; i < copter.gcs().num_gcs(); i++) {
		if (copter.gcs().chan(i).initialised) {
			copter.gcs().chan(i).send_current_states(
					geographicAutomaton.getCurrentGeographicState(),
					occupationalAutomaton.getCurrentOccupationalState(),
					dynamicAutomaton.getCurrentDynamicState(),
					base_occupied);
		}
	}

	copter.gcs().send_text(MAV_SEVERITY_INFO, "REDDY - Sending GEOSTATE %s", geographicAutomaton.getCurrentState().getLabel().c_str());
	copter.gcs().send_text(MAV_SEVERITY_INFO, "REDDY - Sending OCCSTATE %s", occupationalAutomaton.getCurrentState().getLabel().c_str());
	copter.gcs().send_text(MAV_SEVERITY_INFO, "REDDY - Sending DYNSTATE %s", dynamicAutomaton.getCurrentState().getLabel().c_str());
	copter.gcs().send_text(MAV_SEVERITY_INFO, "REDDY - Sending BASE_OCCUPIED %i", copter.base_occupied);
}

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
	// Se os autômatos já foram inicializados

	if(geographicAutomaton.isInitialized()) {
		// Verifica ocorrência de eventos não controláveis

		// Evento ARRIVE_I
		if(dynamicAutomaton.isAtState("FLYING_BASE") &&
				mission.state() == AP_Mission::MISSION_COMPLETE) {
			triggerTransitions(SCAutomaton::arrive.at(copter.base_occupied - 1));
		}

		// Evento TASK_END
		if(occupationalAutomaton.isAtState("BUSY_ASSIGNMENT") &&
				mission.state() == AP_Mission::MISSION_COMPLETE) {

			copter.set_mode(LOITER, MODE_REASON_COORDINATOR_COMMAND);
			triggerTransitions(SCAutomaton::taskEnd);

			// Envia a mensagem de TASKEND para a GCS
			for (unsigned char i = 0; i < copter.gcs().num_gcs(); i++) {
				if (copter.gcs().chan(i).initialised) {
					copter.gcs().chan(i).send_task_end();
				}
			}
		}

		// Evento CRITICAL_SOC
		if(occupationalAutomaton.isAtState("IDLE") &&
				battery.capacity_remaining_pct() < CRITICAL_SOC_VALUE) {
			triggerTransitions(SCAutomaton::criticalSoc);

			// Executa o evento REQUEST_CHARGE
			for (unsigned char i = 0; i < copter.gcs().num_gcs(); i++) {
				if (copter.gcs().chan(i).initialised) {
					copter.gcs().chan(i).send_request_charge();
				}
			}
			triggerTransitions(SCAutomaton::requestCharge);
		}

		// Evento ACCEPTABLE_SOC
		if(occupationalAutomaton.isAtState("BUSY_RECHARGING") &&
				battery.capacity_remaining_pct() > ACCEPTABLE_SOC_VALUE) {
			triggerTransitions(SCAutomaton::acceptableSoc);
		}
	}

}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
