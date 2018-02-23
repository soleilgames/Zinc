/*
 * Copyright (C) 2017  Florian GOLESTIN
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef SOLEIL__EVENTMANAGER_H_
#define SOLEIL__EVENTMANAGER_H_

#include <array>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace Soleil {

  typedef unsigned long EventType;

  class Event
  {
  public:
    EventType getType(void) const noexcept { return type; }
    virtual ~Event() {}

  protected:
    Event(const EventType type)
      : type(type)
    {
    }

    EventType type;
  };

  typedef std::shared_ptr<Event> EventPtr;

  // class EventListener
  // {
  // public:
  //   virtual void operator()(Event& data) = 0;
  // };

  // typedef std::shared_ptr<EventListener> EventListenerPtr;

  typedef std::function<void(Event& event)> EventListener;

  class EventManager
  {
  public:
    void enroll(const EventType& eventType, EventListener listener);

    void emit(EventPtr event);

    void processEvents(double deltaTime);

    void delay(double time, EventPtr event);

  private:
    typedef std::vector<EventListener> EventListenerList;
    std::map<EventType, EventListenerList> registeredListeners;

    typedef std::deque<EventPtr> EventQueue;
    std::array<EventQueue, 2> queues;
    unsigned int activeQueue;

  private: // Delayed
    std::vector<std::pair<double, EventPtr>> delayed;

  public:
    static void Init();
    static void Enroll(const EventType& eventType, EventListener listener);
    static void Emit(EventPtr event);
    static void ProcessEvents(double deltaTime);
    static void Delay(double time, EventPtr event);
  };

} // Soleil

#endif /* SOLEIL__EVENTMANAGER_H_ */
