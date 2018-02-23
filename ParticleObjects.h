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

#ifndef SOLEIL__PARTICLEOBJECTS_H_
#define SOLEIL__PARTICLEOBJECTS_H_

#include <osgParticle/Emitter>
#include <osgParticle/Operator>

namespace Soleil {

  class ShootEmitter : public osgParticle::Emitter
  {
  public:
    void emitParticles(double dt) override;
    void shoot(const osg::Vec3& position, const osg::Vec3& velocity);

  public:
    static osg::ref_ptr<osgParticle::Emitter> CreateAlienShootEmitter();
    static void EmitAlienShoot(const osg::Vec3& position,
                               const osg::Vec3& velocity);
  };

  class CollisionOperator : public osgParticle::Operator
  {
  public:
    void operate(osgParticle::Particle* P, double dt) override;

  public: // TODO:
    osg::Object* cloneType() const override;
    osg::Object* clone(const osg::CopyOp&) const override;
  };

} // Soleil

#endif /* SOLEIL__PARTICLEOBJECTS_H_ */
