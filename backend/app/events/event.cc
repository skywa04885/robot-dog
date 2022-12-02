//
// Created by luke on 2-12-22.
//

#include "event.h"

namespace app::events
{
  event::event(void):
    m_consumptions_left(0)
  {}

  event_test::event_test(std::string initial_message):
    m_message(initial_message)
  {}
}