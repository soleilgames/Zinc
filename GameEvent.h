
#include "EventManager.h"

#include "SceneManager.h" // For ObjectID
#include "stringutils.h"

namespace Soleil {

  class EventDestructObject : public Event
  {
  public:
    static constexpr EventType Type(void)
    {
      return ConstHash("EventDestructObject");
    }

  public:
    EventDestructObject(ObjectID objectId)
      : Event(Type())
      , objectId(objectId)
    {
    }

    ObjectID objectId;
  };

  class EventGameOver : public Event
  {
  public:
    static constexpr EventType Type(void) { return ConstHash("EventGameOver"); }

  public:
    EventGameOver()
      : Event(Type())
    {
    }
  };

  class EventLoadLevel : public Event
  {
  public:
    static constexpr EventType Type(void)
    {
      return ConstHash("EventLoadLevel");
    }

  public:
    EventLoadLevel(std::string name)
      : Event(Type())
      , level(name)
    {
    }

    std::string level;
  };

} // Soleil
