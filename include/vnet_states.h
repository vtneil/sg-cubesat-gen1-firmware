#ifndef PINTO_V1_FIRMWARE_VNET_STATES_H
#define PINTO_V1_FIRMWARE_VNET_STATES_H

#include "vnet_definitions.h"

class State {
public:
    static constexpr uint8_t MAIN_LOOP = BYTES_2('0', '1');
    static constexpr uint8_t SD_READ = BYTES_2('F', 'D');
    static constexpr uint8_t DFU_LOOP = BYTES_2('F', 'E');
    static constexpr uint8_t FORCE_RESET = BYTES_2('F', 'F');

protected:
    bool m_has_arg = false;
    State *m_prev;
    State *m_next;
    State_ID m_id;
    HandlerFunc_Arg_t m_func;

public:
    State(HandlerFunc_t func, State_ID id, State *prev = nullptr, State *next = nullptr) :
            State((HandlerFunc_Arg_t) func, id, prev, next) {
        m_has_arg = false;
    }

    State(HandlerFunc_Arg_t func, State_ID id, State *prev = nullptr, State *next = nullptr) {
        m_id = id;
        m_func = func;
        m_prev = prev;
        m_next = next;
        m_has_arg = true;
    }

    State_ID id() const {
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
    explicit OSState(HandlerFunc_t func,
                     State_ID id = SYS_DISABLED,
                     State *prev = nullptr,
                     State *next = nullptr) : State(func, id, prev, next) {}

    explicit OSState(HandlerFunc_Arg_t func,
                     State_ID id = SYS_DISABLED,
                     State *prev = nullptr,
                     State *next = nullptr) : State(func, id, prev, next) {}
};

#endif //PINTO_V1_FIRMWARE_VNET_STATES_H
