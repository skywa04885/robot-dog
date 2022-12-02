//
// Created by luke on 2-12-22.
//

#include "event_bus_consumer.h"

namespace app::events
{
  /// @brief creates a new event bus consumer.
  /// @param initial_event_bus the event bus for which this consumer is.
  /// @param initial_read_events the initial event index.
  event_bus_consumer::event_bus_consumer(event_bus *initial_event_bus, size_t initial_read_events) :
    m_event_bus(initial_event_bus),
    m_event_buffer_read(initial_read_events)
  {}

  /// @brief consumes an event if there.
  /// @return the event if there otherwise none.
  boost::optional<boost::shared_ptr<event>> event_bus_consumer::consume(void)
  {
    boost::optional<boost::shared_ptr<event>> optional_event =
      this->m_event_bus->consume(this->m_event_buffer_read);

    if (not optional_event.has_value())
      return boost::none;

    ++this->m_event_buffer_read;
    return optional_event.value();
  }
}