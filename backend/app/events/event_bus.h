//
// Created by luke on 2-12-22.
//

#ifndef BACKEND_EVENT_BUS_H
#define BACKEND_EVENT_BUS_H

#include <boost/log/trivial.hpp>
#include <boost/container/list.hpp>
#include <boost/container/vector.hpp>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "event.h"

namespace app::events
{
  class event_bus_consumer;

  class event_bus
  {
    friend class event_bus_consumer;
  protected:
    boost::mutex m_mutex;
    boost::container::list<event_bus_consumer> m_consumers;
    boost::container::vector<boost::optional<boost::shared_ptr<event>>> m_event_buffer;
    size_t m_event_buffer_read;
    size_t m_event_buffer_write;
    size_t m_event_buffer_size;
  public:
    /// @brief constructs a new event bus with the given capacity.
    /// @param capacity the capacity for the event bus.
    event_bus(const size_t capacity);

    /// @brief constructs a new event bus with default size.
    event_bus(void);
  protected:
    /// @brief consumes the event at the given index.
    /// @param event_buffer_read index of the event to consume.
    /// @return boost::none if there is nothing to consume, otherwise the consumed event.
    boost::optional<boost::shared_ptr<event>> consume(const size_t event_buffer_read);

    /// @brief removes all the completely consumed events.
    void prune(void);
  public:
    /// @brief creates a new consumer.
    /// @return the pointer to the newly created consumer.
    event_bus_consumer *create_consumer(void);

    /// @brief emits a new event.
    /// @param event the event to emit.
    /// @return the reference to the current bus instance.
    event_bus &emit(boost::shared_ptr<event> event);
  public:
    /// @brief gets the number of events which can be added before the buffer is full.
    /// @return the available size.
    inline size_t get_event_buffer_available_size(void) const noexcept
    {
      return this->m_event_buffer.capacity() - this->m_event_buffer_size;
    }
  };
}

#endif //BACKEND_EVENT_BUS_H
