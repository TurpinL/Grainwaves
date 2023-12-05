#ifndef GRAINWAVES_GATE_IN_ENHANCED
#define GRAINWAVES_GATE_IN_ENHANCED

#include "daisysp.h"

using namespace daisy;

class GateInEnhanced
{
  public:
    GateInEnhanced() {}
    ~GateInEnhanced() {}

    void Init(GateIn gate_in) {
        gate_in_ = gate_in;
    }

    void Update() {
        prev_state_ = state_;
        state_ = gate_in_.State();
    }

    bool RisingEdge() { return state_ && !prev_state_; }
    bool FallingEdge() { return prev_state_ && !state_; }
    inline bool State() { return state_; }

  private:
    GateIn gate_in_;
    bool prev_state_, state_;
};

#endif
