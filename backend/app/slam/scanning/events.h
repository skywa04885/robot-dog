//
// Created by luke on 2-12-22.
//

#ifndef BACKEND_EVENTS_H
#define BACKEND_EVENTS_H

#include "../../events/event.h"

namespace app
{
  namespace slam
  {
    namespace scanning
    {

      class event_scan_completed: public events::event
      {
      protected:
      public:
      };
    } // app
  } // slam
} // scanning

#endif //BACKEND_EVENTS_H
