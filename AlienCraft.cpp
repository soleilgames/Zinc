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

#include <cassert>

#include "AlienCraft.h"

#include "EventManager.h"
#include "GameEvent.h"
#include "Logger.h"
#include "ParticleObjects.h"
#include "SceneManager.h"
#include "Utils.h"

namespace Soleil {

  AlienCraft::AlienCraft()
    : velocity(0.0f, 0.0f, 0.0f)
    , force(0.0f, 0.0f, 0.0f)
    , friction(0.00f) // No friction
    , mass(1.0f)
    , maxSpeed(4.0f)
    , maxForce(1.0f)
    , fireRate(1.0f / 2.0f) // Two fire per second
    , previousTime(0.0f)
    , lastShootTime(0.0f)
  {
  }

  bool AlienCraft::run(osg::Object* object, osg::Object* data)
  {
    osg::NodeVisitor* visitor = data->asNodeVisitor();
    assert(visitor && "Class must be attached as a Callback visitor");
    osg::MatrixTransform* node =
      object->asNode()->asTransform()->asMatrixTransform();
    assert(node && "Callback should be bound to MatrixTransform");

    if (previousTime == 0.0f) {
      previousTime = visitor->getFrameStamp()->getSimulationTime();
    }

    // --- Compute behavior --------------------------------------
    osg::NodePath playerPath = SceneManager::GetNodePath(ConstHash("Player"));
    osg::Vec3     target     = playerPath.back()->getBound().center();
    osg::Vec3     current    = node->getMatrix().getTrans();
    osg::Vec3     desired    = normalize(target - current) * maxSpeed;
    osg::Vec3     steering   = limit(desired - velocity, maxForce);

    force = steering;

    // Compute Physics -------------------------------------------
    const float deltaTime =
      visitor->getFrameStamp()->getSimulationTime() - previousTime;

    const osg::Vec3 acceleration = force / mass;

    velocity += acceleration;

    if (friction > 0.0f) {
      velocity.x() *= std::pow(friction * mass, deltaTime);
      velocity.y() *= std::pow(friction * mass, deltaTime);
      velocity.z() *= std::pow(friction * mass, deltaTime);
    }

    osg::Quat q;
    q.makeRotate(osg::Vec3(0, 1, 0), velocity);
    osg::Matrix m = node->getMatrix();
    m.setRotate(q);
    // node->setMatrix(m * osg::Matrix::translate(velocity * deltaTime));
    m *= osg::Matrix::translate(velocity * deltaTime);

    const osg::Vec3 nextPosition = m.getTrans();
    osg::Vec3       collisionNormal;

    if (Soleil::SceneManager::SegmentCollision(
          node->getMatrix().getTrans(), nextPosition, node, &collisionNormal)) {

      osg::NodePath p = visitor->getNodePath();
      Soleil::EventManager::Emit(
        std::make_shared<Soleil::EventDestructObject>(p));
    } else {
      node->setMatrix(m);
    }

    // Shoot the player if close enough ----------------------------------------
    lastShootTime += deltaTime;
    if (lastShootTime >= fireRate) {
      const osg::Vec3 targetToCraft = target - current;
      const float     facing = normalize(targetToCraft) * normalize(velocity);
      // SOLEIL__LOGGER_DEBUG("FACING: ", facing);
      if (targetToCraft.length() < 50.0f && facing > 0.98f) {
        // TODO: Limit the number of shoot

        const osg::Vec3 normalizedVelocity = normalize(velocity);
        const osg::Vec3 gun =
          current + normalizedVelocity * node->getBound().radius() * 1.1f;
        // To avoid the alien craft destruct itself when shooting;

        ShootEmitter::EmitAlienShoot(gun, normalizedVelocity * 100.0f);
      }
      lastShootTime = 0.0f;
    }

    previousTime = visitor->getFrameStamp()->getSimulationTime();
    return traverse(object, data);
  }

} // Soleil
