#pragma once

#include "esphome/core/automation.h"
#include "pd.h"

namespace esphome {
namespace power_delivery {

template<typename... Ts>
class PowerDeliveryRequestVoltage : public Action<Ts...>, public Parented<PowerDelivery> {
public:
  TEMPLATABLE_VALUE(int, voltage)
  void play(Ts... x) override {this->parent_->request_voltage(this->voltage_.value(x...));}
};


class StateTrigger : public Trigger<> {
 public:
  explicit StateTrigger(PowerDelivery *pd) {
    pd->add_on_state_callback([this]() { this->trigger(); });
  }
};

template<PowerDeliveryState State> class PDStateTrigger : public Trigger<> {
 public:
  explicit PDStateTrigger(PowerDelivery *pd) {
    pd->add_on_state_callback([this, pd]() {
      if (pd->state == State)
        this->trigger();
    });
  }
};

class PowerReadyTrigger : public Trigger<>{
public:
  explicit PowerReadyTrigger(PowerDelivery *pd) {
    pd->add_on_state_callback([this, pd]() {
      if (    pd->state == PD_STATE_EXPLICIT_SPR_CONTRACT 
           || pd->state == PD_STATE_EXPLICIT_EPR_CONTRACT 
           || pd->state == PD_STATE_PD_TIMEOUT)
        this->trigger();
    });
  }
};

class ConnectedTrigger : public Trigger<>{
public:
  explicit ConnectedTrigger(PowerDelivery *pd) {
    pd->add_on_state_callback([this, pd]() {
      if( pd->prev_state_ == PD_STATE_DISCONNECTED && pd->state == PD_STATE_DEFAULT_CONTRACT)
        this->trigger();
    });
  }
};



using DisconnectedTrigger = PDStateTrigger<PowerDeliveryState::PD_STATE_DISCONNECTED>;
using ErrorTrigger = PDStateTrigger<PowerDeliveryState::PD_STATE_ERROR>;
using TransitionTrigger = PDStateTrigger<PowerDeliveryState::PD_STATE_TRANSITION>;

template<typename... Ts> class IsConnectedCondition : public Condition<Ts...>, public Parented<PowerDelivery> {
 public:
  bool check(Ts... x) override { return this->parent_->state == PowerDeliveryState::PD_STATE_DEFAULT_CONTRACT || this->parent_->state == PowerDeliveryState::PD_STATE_EXPLICIT_SPR_CONTRACT || PowerDeliveryState::PD_STATE_EXPLICIT_EPR_CONTRACT; }
};




}
}
