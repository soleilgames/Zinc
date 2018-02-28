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

#ifndef SOLEIL__ALIENCRAFT_H_
#define SOLEIL__ALIENCRAFT_H_

#include <osg/Callback>
#include <osg/MatrixTransform>

namespace Soleil {

  class AlienCraft : public osg::Callback
  {
  public:
    AlienCraft();
    bool run(osg::Object* object, osg::Object* data);

  public:
    osg::Vec3 velocity;
    osg::Vec3 force;
    float     friction;
    float     mass;
    float     maxSpeed;
    float     maxForce;
    float     maxRotation;
    float     fireRate;

  private:
    double previousTime;
    double lastShootTime;
    double remainingActionTime;
    int    behavior;

    enum
    {
      AlienCraftNoBehavior,
      AlienCraftChasePlayer,
      AlienCraftFlee,
      AlienCraftGoto
    };

  private:
    osg::Vec3 chasePlayer(const osg::Vec3& targetToCraft,
                          const osg::Vec3& position, const float deltaTime,
                          osg::Node* node);
    osg::Vec3 flee(const osg::Vec3& playerDirection);
    osg::Vec3 goTo(const osg::Vec3& targetToCraft);

  public:
    static constexpr float ChaseRange =
      200.0f * 200.0f; // (we are using lenght2)
    static constexpr float FireRange = 50.0f * 50.0f;
    static constexpr float Avoidance =
      20.0f * 20.0f; // TODO: Use Avoidance behavior
    static constexpr float Facing = 0.98f;
  };

} // Soleil

#endif /* SOLEIL__ALIENCRAFT_H_ */
