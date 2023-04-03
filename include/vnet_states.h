#ifndef PINTO_V1_FIRMWARE_VNET_STATES_H
#define PINTO_V1_FIRMWARE_VNET_STATES_H

#include "vnet_definitions.h"

enum StateInput : uint16_t {
    MAIN_LOOP = ('0' << 1) + '1',
    DFU_LOOP = ('F' << 1) + 'E'
};

constexpr uint8_t STATE_MAIN_LOOP = StateInput::MAIN_LOOP;
constexpr uint8_t STATE_DFU_LOOP = StateInput::DFU_LOOP;

class State {
protected:
    bool m_has_arg = false;
    State *m_prev;
    State *m_next;
    StateID_e m_id;
    HandlerFunc_Arg_t m_func;
public:
    State(HandlerFunc_t func, StateID_e id, State *prev = nullptr, State *next = nullptr) :
            State((HandlerFunc_Arg_t) func, id, prev, next) {
        m_has_arg = false;
    }

    State(HandlerFunc_Arg_t func, StateID_e id, State *prev = nullptr, State *next = nullptr) {
        m_id = id;
        m_func = func;
        m_prev = prev;
        m_next = next;
        m_has_arg = true;
    }

    StateID_e id() const {
        return m_id;
    }

    void run(void *arg = nullptr) {
        if (m_func != nullptr) {
            if (m_has_arg)
                m_func(arg);
            else
                m_func(nullptr);
        }
    }

    State *next() {
        return m_next;
    }

    State *prev() {
        return m_prev;
    }

    bool has_next() {
        return m_next != nullptr;
    }

    bool has_prev() {
        return m_prev != nullptr;
    }
};

class OSState : public State {
public:
    OSState(HandlerFunc_t func, StateID_e id, State *prev = nullptr, State *next = nullptr) :
            OSState((HandlerFunc_Arg_t) func, id, prev, next) {}

    OSState(HandlerFunc_Arg_t func, StateID_e id, State *prev = nullptr, State *next = nullptr) :
            State(func, id, prev, next) {}
};

#endif //PINTO_V1_FIRMWARE_VNET_STATES_H
