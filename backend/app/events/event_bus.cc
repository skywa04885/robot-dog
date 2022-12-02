//
// Created by luke on 2-12-22.
//

#include "event_bus.h"
#include "event_bus_consumer.h"

namespace app::events
{
  /// @brief constructs a new event bus with the given capacity.
  /// @param capacity the capacity for the event bus.
  event_bus::event_bus(const size_t capacity):
    m_mutex(),
    m_consumers(),
    m_event_buffer(capacity, boost::none),
    m_event_buffer_read(0),
    m_event_buffer_write(0),
    m_event_buffer_size(0)
  {
  }

  /// @brief constructs a new event bus with default size.
  event_bus::event_bus(void):
    event_bus(1024)
  {
  }

  /// @brief removes all the completely consumed events.
  void event_bus::prune(void)
  {
    if (this->get_event_buffer_available_size() == 0)
      return;

    while (this->m_event_buffer_read != this->m_event_buffer_write)
    {
      const size_t idx = this->m_event_buffer_read % this->m_event_buffer.capacity();

      boost::optional<boost::shared_ptr<event>> optional_event = this->m_event_buffer[idx];
      BOOST_ASSERT(optional_event.has_value());

      boost::shared_ptr<event> ev = optional_event.value();
      if (ev->m_consumptions_left > 0)
        break;

      BOOST_LOG_TRIVIAL(info) << "Prune";

      this->m_event_buffer[idx] = boost::none;
      ++this->m_event_buffer_read;
    }
  }

  /// @brief consumes the event at the given index.
  /// @param event_buffer_read index of the event to consume.
  /// @return boost::none if there is nothing to consume, otherwise the consumed event.
  boost::optional<boost::shared_ptr<event>> event_bus::consume(const size_t event_buffer_read)
  {
    boost::lock_guard lock(this->m_mutex);

    if (event_buffer_read == this->m_event_buffer_write)
      return boost::none;

    if (event_buffer_read < this->m_event_buffer_read)
      throw std::runtime_error("Attempting to read pruned event.");

    boost::optional<boost::shared_ptr<event>> optional_event =
      this->m_event_buffer[event_buffer_read % this->m_event_buffer.capacity()];
    BOOST_ASSERT(optional_event.has_value());

    boost::shared_ptr<event> ev = optional_event.value();
    --ev->m_consumptions_left;

    this->prune();

    return ev;
  }

  /// @brief creates a new consumer.
  /// @return the pointer to the newly created consumer.
  event_bus_consumer *event_bus::create_consumer(void)
  {
    boost::lock_guard lock(this->m_mutex);
    return &this->m_consumers.emplace_back(this, this->m_event_buffer_write);
  }

  /// @brief emits a new event.
  /// @param event the event to emit.
  /// @return the reference to the current bus instance.
  event_bus &event_bus::emit(boost::shared_ptr<event> event)
  {
    boost::lock_guard lock(this->m_mutex);

    if (this->get_event_buffer_available_size() == 0)
      throw std::overflow_error("Event buffer is full.");

    if (this->m_consumers.empty())
      return *this;

    event->m_consumptions_left = this->m_consumers.size();

    this->m_event_buffer[this->m_event_buffer_write % this->m_event_buffer.capacity()] = event;

    ++this->m_event_buffer_write;
    ++this->m_event_buffer_size;

    return *this;
  }
}
