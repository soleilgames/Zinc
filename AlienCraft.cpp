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

// // TODO: remove Debug:
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
//#define SOLEIL__DEBUG_STEERING 1

namespace Soleil {

  // TODO: Remove debug:
  static osg::ref_ptr<osg::Node> taxis;

  static std::size_t                   currenId = 0;
  static std::vector<AlienCraft::Boid> boids;

  AlienCraft::AlienCraft()
    : velocity(0.0f, 0.0f, 0.0f)
    , force(0.0f, 0.0f, 0.0f)
    , friction(0.00f) // No friction
    , mass(1.0f)
    , maxSpeed(10.0f) // 10 is a bit below player speed
    , maxForce(.1f)
    , maxRotation(.6f)      // 0.5 osg::PI_2f
    , fireRate(1.0f / 2.0f) // Two fire per second
    , previousTime(0.0f)
    , lastShootTime(0.0f)
    , remainingActionTime(0.0f)
    , behavior(AlienCraftNoBehavior)
  {
    if (!taxis) {
      taxis = osgDB::readNodeFile("../media/taxis.osgt");
    }

    myBoidId = currenId++;
    boids.push_back({myBoidId, osg::Vec3(), osg::Vec3()});
  }

  AlienCraft::~AlienCraft()
  {
    // Remove ourself from the boids list
    for (auto it = boids.begin(); it != boids.end(); ++it) {
      if (it->id == myBoidId) {
        boids.erase(it);
        return;
      }
    }
  }

  bool AlienCraft::run(osg::Object* object, osg::Object* data)
  {
    osg::NodeVisitor* visitor = data->asNodeVisitor();
    assert(visitor && "Class must be attached as a Callback visitor");
    osg::PositionAttitudeTransform* node =
      object->asNode()->asTransform()->asPositionAttitudeTransform();
    assert(node && "Callback should be bound to a PositionAttitudeTransform");

    if (previousTime == 0.0f) {
      previousTime = visitor->getFrameStamp()->getSimulationTime();
    }

    // --- Compute behavior --------------------------------------
    const osg::ref_ptr<osg::PositionAttitudeTransform> playerNode =
      SceneManager::GetNodePath(ConstHash("Player"))
        .back()
        ->asTransform()
        ->asPositionAttitudeTransform();
    const osg::Vec3 target  = playerNode->getPosition();
    const osg::Vec3 current = node->getPosition();
    const osg::Vec3 playerDirection =
      playerNode->getAttitude() * osg::Vec3(0, 1, 0);

    const osg::Vec3 targetToCraft      = target - current;
    const float     lengthToTarget2    = targetToCraft.length2();
    const bool      isPlayerInRange    = lengthToTarget2 < ChaseRange;
    const bool      isPlayerInTooClose = lengthToTarget2 < Avoidance;
    const bool      isPlayerFacingMe =
      facing(target, playerDirection, current) > Facing;

    const float deltaTime =
      visitor->getFrameStamp()->getSimulationTime() - previousTime;

    osg::Vec3 desired(0, 0, 0);

    if (behavior == AlienCraftNoBehavior) {
      if (
#if not defined(SOLEIL__DEBUG_STEERING)
        (isPlayerInRange && isPlayerFacingMe) ||
#endif
        isPlayerInTooClose) {
        behavior            = AlienCraftFlee;
        remainingActionTime = Random(0.5f, 2.0f);
      } else if (isPlayerInRange) {
        behavior            = AlienCraftChasePlayer;
        remainingActionTime = Random(0.5f, 1.0f);
      } else {
        behavior            = AlienCraftGoto;
        remainingActionTime = 0;
      }
    }
    switch (behavior) {
      case AlienCraftChasePlayer:
        desired = chasePlayer(targetToCraft, current, deltaTime, node);
        break;
      // case AlienCraftFlee: desired = flee(playerDirection); break;
      case AlienCraftFlee:
        desired = normalize(current - target) * maxSpeed;
        break;

      case AlienCraftGoto: desired = goTo(targetToCraft); break;
    };
    remainingActionTime -= deltaTime;
    if (remainingActionTime <= 0.0f) behavior = AlienCraftNoBehavior;

    // this->applyForceSeparate(current);
    // this->applyForceAlignment(current);
    // this->applyForceCohesion(current);

    if (desired.length2() > 0.0f) {
      // If the vehicle has no velocity, allows an abrupt turn
      if (velocity.length2() > 0.0f) {
        const float dot   = (normalize(velocity) * normalize(desired));
        const float angle = std::acos(
          dot / (velocity.length(), desired.length())); // TODO: use map?
        if (std::abs(angle) > maxRotation) {
          const float     length = desired.length();
          const osg::Vec3 axis =
            normalize(normalize(desired) ^
                      normalize(velocity)); // TODO: Normalize the normalization

          desired = normalize(velocity) * length;
          desired = osg::Quat(-maxRotation, axis) * desired;
        }
      }

      osg::Vec3 steering = limit(desired - velocity, maxForce);
      force              = steering;
    }

    this->applyForceSeparate(current);
    this->applyForceAlignment(current);
    this->applyForceCohesion(current);

// TODO: remove test:
#ifdef SOLEIL__DEBUG_STEERING
    {
      assert(osg::Quat().zeroRotation());
      static osg::ref_ptr<osg::MatrixTransform> desiredDir;
      static osg::ref_ptr<osg::MatrixTransform> velocityDir;
      static osg::ref_ptr<osg::MatrixTransform> steeringDir;
      if (!desiredDir) {
        // Desired ---
        desiredDir = new osg::MatrixTransform;
        desiredDir->setNodeMask(desiredDir->getNodeMask() &
                                ~SceneManager::Mask::Collision);
        desiredDir->addChild(Soleil::GetNodeByName(*taxis, "Direction"));
        // Velocity ---
        velocityDir = new osg::MatrixTransform;
        velocityDir->setNodeMask(velocityDir->getNodeMask() &
                                 ~SceneManager::Mask::Collision);
        velocityDir->addChild(Soleil::GetNodeByName(*taxis, "Normal"));
        // Steering ---
        steeringDir = new osg::MatrixTransform;
        steeringDir->setNodeMask(steeringDir->getNodeMask() &
                                 ~SceneManager::Mask::Collision);
        steeringDir->addChild(Soleil::GetNodeByName(*taxis, "Reflect"));
        // Update ---
        EventManager::Delay(0.0f, [](Event& /*e*/) {
          SceneManager::GetRoot()->addChild(desiredDir);
          SceneManager::GetRoot()->addChild(velocityDir);
          SceneManager::GetRoot()->addChild(steeringDir);
        });
      } else {
        desiredDir->setMatrix(osg::Matrix::rotate(osg::Vec3(0, 1, 0), desired) *
                              osg::Matrix::translate(current));
        if (velocity.length2() > 0.0f) {
          velocityDir->setMatrix(
            osg::Matrix::rotate(osg::Vec3(0, 1, 0), velocity) *
            osg::Matrix::translate(current));
        }
        if (force.length2() > 0.0f) {
          steeringDir->setMatrix(
            osg::Matrix::rotate(osg::Vec3(0, 1, 0), force) *
            osg::Matrix::translate(current));
        }
      }
    }
#endif

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
      osg::Vec3 new_forward = normalize(velocity);
#if 0
      // TODO: For the roll operation if needed
      osg::Vec3 approximate_up(0, 0, 1);
      osg::Vec3 new_side = new_forward ^ approximate_up; // cross product
      osg::Vec3 new_up   = new_forward ^ new_side;       // cross product
#endif
      const osg::Quat q = quatLookAt(new_forward, VectorUp());
      // osg::Matrix     m = node->getMatrix();
      // m.setRotate(q); //   * deltaTime
      // m *= osg::Matrix::translate(velocity * deltaTime);
      // const osg::Vec3 nextPosition = m.getTrans();

      const osg::Vec3 nextPosition = current + (velocity * deltaTime);
      osg::Vec3       collisionNormal;
      if (Soleil::SceneManager::SegmentCollision(current, nextPosition, node,
                                                 &collisionNormal)) {

        osg::NodePath p = visitor->getNodePath();
        Soleil::EventManager::Emit(
          std::make_shared<Soleil::EventDestructObject>(p));
      } else {
        // node->setMatrix(m);
        node->setAttitude(q);
        node->setPosition(nextPosition);
      }
    }

    // Update velocity for other boids: -------------------
    Boid* me = nullptr;
    for (auto it = boids.begin(); it != boids.end(); ++it) {
      if (it->id == myBoidId) {
        me = it.base();
        break;
      }
    }
    assert(me != nullptr && "Boid not found in list");
    me->velocity = velocity;
    me->position = current;
    // TODO: Use my own map or one callback for every boids

    previousTime = visitor->getFrameStamp()->getSimulationTime();
    return traverse(object, data);
  }

  osg::Vec3 AlienCraft::chasePlayer(const osg::Vec3& targetToCraft,
                                    const osg::Vec3& position,
                                    const float deltaTime, osg::Node* node)
  {
    // Shoot the player if close enough --------------------------------------
    lastShootTime += deltaTime;
    if (lastShootTime >= fireRate) {
      const float facing = normalize(targetToCraft) * normalize(velocity);
      // SOLEIL__LOGGER_DEBUG("FACING: ", facing);
      if (targetToCraft.length() < FireRange && facing > Facing) {

        const osg::Vec3 normalizedVelocity = normalize(velocity);
        const osg::Vec3 gun =
          position + normalizedVelocity * node->getBound().radius() * 1.1f;
        // To avoid the alien craft destruct itself when shooting;

        ShootEmitter::EmitAlienShoot(gun, normalizedVelocity * 100.0f);
      }
      lastShootTime = 0.0f;
    }

    return normalize(targetToCraft) * maxSpeed;
  }

  osg::Vec3 AlienCraft::flee(const osg::Vec3& playerDirection)
  {
    return normalize(playerDirection) * maxSpeed; // TODO: Add Some randomeness
  }

  osg::Vec3 AlienCraft::goTo(const osg::Vec3& targetToCraft)
  {
    return normalize(targetToCraft) * maxSpeed;
  }

  void AlienCraft::applyForceSeparate(const osg::Vec3& currentPosition)
  {
    int       count = 0;
    osg::Vec3 sum;

    for (auto it = boids.begin(); it != boids.end(); ++it) {
      if (it->id == myBoidId) continue;

      const osg::Vec3 diff     = currentPosition - it->position;
      const float     distance = diff.length();
      if (distance > SeparationRange) {
        continue;
      }

      sum += (diff / distance * distance); // weight by distance
      // SOLEIL__LOGGER_DEBUG((diff / distance * distance),
      //                      "==", normalize(diff) / distance);

      count++;
    }

    if (count > 0) {
      sum /= count;
      sum.normalize();
      sum *= maxSpeed;
      osg::Vec3 steer = sum - velocity;
      steer           = limit(steer, maxForce);

      steer *= 1.5f; // // TODO: configurable

      force += steer;
    }
  }

  void AlienCraft::applyForceAlignment(const osg::Vec3& currentPosition)
  {
    int       count = 0;
    osg::Vec3 sum;

    for (auto it = boids.begin(); it != boids.end(); ++it) {
      if (it->id == myBoidId) continue;

      const osg::Vec3 diff     = currentPosition - it->position;
      const float     distance = diff.length();
      if (distance > SeparationRange) {
        continue;
      }

      sum += it->velocity;
      // SOLEIL__LOGGER_DEBUG((diff / distance * distance),
      //                      "==", normalize(diff) / distance);

      count++;
    }

    if (count > 0) {
      sum /= count;
      sum.normalize();
      sum *= maxSpeed;
      osg::Vec3 steer = sum - velocity;
      steer           = limit(steer, maxForce);

      steer *= .40f; // // TODO: configurable

      force += steer;
    }
  }

  void AlienCraft::applyForceCohesion(const osg::Vec3& currentPosition)
  {
    int       count = 0;
    osg::Vec3 sum;

    for (auto it = boids.begin(); it != boids.end(); ++it) {
      if (it->id == myBoidId) continue;

      const osg::Vec3 diff     = currentPosition - it->position;
      const float     distance = diff.length();
      if (distance > SeparationRange) {
        continue;
      }

      sum += it->position;
      // SOLEIL__LOGGER_DEBUG((diff / distance * distance),
      //                      "==", normalize(diff) / distance);

      count++;
    }

    if (count > 0) {
      sum /= count;

      const osg::Vec3 desired = normalize(sum - currentPosition) * maxSpeed;
      const osg::Vec3 steer =
        (desired - velocity) * .010f; // // TODO: configurable

      force += steer;
    }
  }

} // Soleil
