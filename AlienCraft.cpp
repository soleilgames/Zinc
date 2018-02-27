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
#include <osg/PositionAttitudeTransform>

namespace Soleil {

  AlienCraft::AlienCraft()
    : velocity(0.0f, 0.0f, 0.0f)
    , force(0.0f, 0.0f, 0.0f)
    , friction(0.00f) // No friction
    , mass(1.0f)
    , maxSpeed(10.0f) // 10 is a bit below player speed
    , maxForce(.1f)
    , maxRotation(0.5f)
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

    constexpr float ChaseRange = 200.0f * 200.0f; // (we are using lenght2)
    constexpr float FireRange  = 50.0f * 50.0f;
    constexpr float Avoidance  = 10.0f * 10.0f; // TODO: Use Avoidance behavior
    constexpr float Facing     = 0.98f;

    // --- Compute behavior --------------------------------------
    const osg::ref_ptr<osg::PositionAttitudeTransform> playerNode =
      SceneManager::GetNodePath(ConstHash("Player"))
        .back()
        ->asTransform()
        ->asPositionAttitudeTransform();
    const osg::Vec3 target  = playerNode->getPosition();
    const osg::Vec3 current = node->getMatrix().getTrans();
    const osg::Vec3 playerDirection =
      playerNode->getAttitude() * osg::Vec3(0, 1, 0);

    const bool isPlayerInRange    = (target - current).length2() < ChaseRange;
    const bool isPlayerInTooClose = (target - current).length2() < Avoidance;
    const bool isPlayerFacingMe =
      facing(target, playerDirection, current) > Facing;

    const float deltaTime =
      visitor->getFrameStamp()->getSimulationTime() - previousTime;

    osg::Vec3 desired(0, 0, 0);
    if ((isPlayerInRange && isPlayerFacingMe) || isPlayerInTooClose) {
      desired =
        normalize(playerDirection) * maxSpeed; // TODO: Add Some randomeness

    } else if (isPlayerInRange) {
      desired = normalize(target - current) * maxSpeed;

      // Shoot the player if close enough --------------------------------------
      lastShootTime += deltaTime;
      if (lastShootTime >= fireRate) {
        // TODO: Optimize already computed
        const osg::Vec3 targetToCraft = target - current;
        const float     facing = normalize(targetToCraft) * normalize(velocity);
        // SOLEIL__LOGGER_DEBUG("FACING: ", facing);
        if (targetToCraft.length() < FireRange && facing > Facing) {

          const osg::Vec3 normalizedVelocity = normalize(velocity);
          const osg::Vec3 gun =
            current + normalizedVelocity * node->getBound().radius() * 1.1f;
          // To avoid the alien craft destruct itself when shooting;

          ShootEmitter::EmitAlienShoot(gun, normalizedVelocity * 100.0f);
        }
        lastShootTime = 0.0f;
      }

    } else {
      desired = normalize(target - current) * maxSpeed;
    }

    SOLEIL__LOGGER_DEBUG("Desired (before rotation): ", desired);
    if (desired.length2() > 0.0f) {
      const float dot = (velocity * desired);
      SOLEIL__LOGGER_DEBUG("Dot: ", desired,
                           ". Map: ", map(-1.0f, 1.0f, osg::PIf, 0.0f, dot));
      // If the vehicle has no velocity, allows an abrupt turn
      if (velocity.length2() > 0.0f &&
          map(-1.0f, 1.0f, osg::PIf, 0.0f, dot) > maxRotation) {
        const float length = desired.length();

        // TODO: Not always turn left
        desired = normalize(velocity) * length;
        desired = osg::Quat(maxRotation, osg::Vec3(0, 1, 0)) * desired;
        SOLEIL__LOGGER_DEBUG("Desired (after rotation): ", desired);
      }

      osg::Vec3 steering = limit(desired - velocity, maxForce);

      SOLEIL__LOGGER_DEBUG("Steering: ", steering);
      force = steering;
    }

    // Compute Physics -------------------------------------------
    const osg::Vec3 acceleration = force / mass;

    velocity += acceleration;

    if (friction > 0.0f) {
      velocity.x() *= std::pow(friction * mass, deltaTime);
      velocity.y() *= std::pow(friction * mass, deltaTime);
      velocity.z() *= std::pow(friction * mass, deltaTime);
    }

    if (velocity.length2() > 0.0f) {
// TODO: Orient the craft event if velocity == 0
#if 0
      osg::Quat q;
      q.makeRotate(osg::Vec3(0, 1, 0), velocity);
#else
      osg::Vec3 new_forward = normalize(velocity);
      osg::Vec3 approximate_up(0, 0, 1);
      osg::Vec3 new_side = new_forward ^ approximate_up; // cross product
      osg::Vec3 new_up   = new_forward ^ new_side;       // cross product
      osg::Quat q;
      q.makeRotate(osg::Vec3(0, 1, 0), new_forward);
#endif
      osg::Matrix m = node->getMatrix();
      m.setRotate(q); //   * deltaTime
      m *= osg::Matrix::translate(velocity * deltaTime);

// TODO: Restore collistion
#if 0
      const osg::Vec3 nextPosition = m.getTrans();
      osg::Vec3       collisionNormal;

      if (Soleil::SceneManager::SegmentCollision(node->getMatrix().getTrans(),
                                                 nextPosition, node,
                                                 &collisionNormal)) {

        osg::NodePath p = visitor->getNodePath();
        Soleil::EventManager::Emit(
          std::make_shared<Soleil::EventDestructObject>(p));
      } else {
        node->setMatrix(m);
      }
#else
      node->setMatrix(m);
#endif
    }

    previousTime = visitor->getFrameStamp()->getSimulationTime();
    return traverse(object, data);
  }

} // Soleil
