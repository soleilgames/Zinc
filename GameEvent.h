
#include "EventManager.h"

#include "stringutils.h"
#include <osg/Node>

namespace Soleil {

  class EventDestructObject : public Event
  {
  public:
    static constexpr EventType Type(void)
    {
      return ConstHash("EventDestructObject");
    }

  public:
    EventDestructObject(osg::NodePath pathToObject)
      : Event(Type())
      , object(pathToObject)
    {
    }

    osg::NodePath object;
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

  class EventPlayerDestroyed : public Event
  {
  public:
    static constexpr EventType Type(void)
    {
      return ConstHash("EventPlayerDestroyed");
    }

  public:
    EventPlayerDestroyed()
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
