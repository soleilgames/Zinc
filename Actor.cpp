

#include "Actor.h"

namespace Soleil {

  Actor::Actor(int lifePoints)
    : lifePoints(lifePoints)
    , inRemoveQueue(false)
  {
  }
} // Soleil
