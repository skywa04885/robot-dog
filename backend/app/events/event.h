//
// Created by luke on 2-12-22.
//

#ifndef BACKEND_EVENT_H
#define BACKEND_EVENT_H

#include <string>

namespace app::events
{

    class event
    {
      friend class event_bus;
      friend class event_bus_consumer;
    protected:
      size_t m_consumptions_left;
    public:
      event(void);

    };

    class event_test: public event
    {
    protected:
      std::string m_message;
    public:
      event_test(std::string initial_message);
    public:
      inline const std::string &get_message(void) const
      {
        return this->m_message;
      }
    };

} // events

#endif //BACKEND_EVENT_H
