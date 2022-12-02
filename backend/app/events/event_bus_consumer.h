//
// Created by luke on 2-12-22.
//

#ifndef BACKEND_EVENT_BUS_CONSUMER_H
#define BACKEND_EVENT_BUS_CONSUMER_H

#include <cstddef>
#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>

#include "event_bus.h"
#include "event.h"

namespace app::events
{
  class event_bus_consumer
  {
  protected:
    event_bus *m_event_bus;
    size_t m_event_buffer_read;

  protected:
    /// @brief creates a new event bus consumer.
    /// @param initial_event_bus the event bus for which this consumer is.
    /// @param initial_read_events the initial event index.
    event_bus_consumer(event_bus *initial_event_bus, size_t initial_read_events);

  public:
    /// @brief gets the total number of events left to consume.
    /// @return the total number of events left to consume.
    inline size_t unconsumed_event_count(void) const
    {

      return this->m_event_bus->m_event_buffer_write - this->m_event_buffer_read;
    }

    /// @brief checks if there are any events to consume.
    /// @return true if there are events to consume.
    inline bool can_consume(void) const
    {
      return this->unconsumed_event_count() != 0;
    }

    /// @brief consumes an event if there.
    /// @return the event if there otherwise none.
    boost::optional<boost::shared_ptr<event>> consume(void);
  };
}

#endif //BACKEND_EVENT_BUS_CONSUMER_H
