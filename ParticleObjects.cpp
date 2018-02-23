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

#include "ParticleObjects.h"

#include "EventManager.h"
#include "GameEvent.h"
#include "SceneManager.h"
#include <cassert>

namespace Soleil {

  void ShootEmitter::emitParticles(double /*dt*/) {}

  void ShootEmitter::shoot(const osg::Vec3& position, const osg::Vec3& velocity)
  {
    const osgParticle::Particle& particleTemplate =
      getUseDefaultTemplate()
        ? getParticleSystem()->getDefaultParticleTemplate()
        : getParticleTemplate();

    osgParticle::Particle* P =
      getParticleSystem()->createParticle(&particleTemplate);
    if (P) {
      P->setPosition(position);
      P->setVelocity(velocity);
    } else {
      OSG_NOTICE << "run out of particle" << std::endl;
    }
  }

  static osg::ref_ptr<ShootEmitter> alienShootEmitter;

  void ShootEmitter::EmitAlienShoot(const osg::Vec3& position,
                                    const osg::Vec3& velocity)
  {
    assert(alienShootEmitter &&
           "Call ShootEmitter::CreateAlienShootEmitter before this method");

    alienShootEmitter->shoot(position, velocity);
  }

  osg::ref_ptr<osgParticle::Emitter> ShootEmitter::CreateAlienShootEmitter()
  {
    constexpr float scale = 1.0f;

    alienShootEmitter = new ShootEmitter;

    // osg::ref_ptr<osgParticle::RandomRateCounter> rrc =
    //   new osgParticle::RandomRateCounter;
    // rrc->setRateRange(800, 1000);

    // osg::ref_ptr<osgParticle::ModularEmitter> emitter =
    //   new osgParticle::ModularEmitter;
    // // emitter->setParticleSystem(ps);
    // emitter->setCounter(rrc);
    // emitter->setEndless(false);
    // emitter->setLifeTime(.10f);

    // osg::ref_ptr<osgParticle::RadialShooter> shooter =
    //   new osgParticle::RadialShooter;
    // osg::ref_ptr<osgParticle::SectorPlacer> placer =
    //   new osgParticle::SectorPlacer;

    // emitter->setPlacer(placer);
    // emitter->setShooter(shooter);

    // placer->setCenter(osg::Vec3(0, 0, 60));
    // placer->setRadiusRange(0.0f * scale, 0.25f * scale);

    // shooter->setThetaRange(0.0f, osg::PI * 2.0f);
    // shooter->setInitialSpeedRange(1.0f * scale, 10.0f * scale);

    return alienShootEmitter;
  }

  void CollisionOperator::operate(osgParticle::Particle* P, double dt)
  {
    osg::Vec3 start = P->getPosition();
    osg::Vec3 end   = start + (P->getVelocity() * dt);

    osg::NodePath path;
    if (SceneManager::SegmentCollision(start, end, nullptr, nullptr, nullptr,
                                       SceneManager::Mask::Shootable, &path)) {
      P->kill();
      EventManager::Emit(std::make_shared<EventDestructObject>(path));
    }
  }

  osg::Object* CollisionOperator::cloneType() const
  {
    assert(false && "// TODO: cloneType");
  }

  osg::Object* CollisionOperator::clone(const osg::CopyOp&) const
  {
    assert(false && "// TODO: clone");
  }

} // Soleil
