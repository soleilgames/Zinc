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

#ifndef SOLEIL__VOLUMETRICFOG_H_
#define SOLEIL__VOLUMETRICFOG_H_

#include <functional>
#include <osg/Camera>
#include <osg/Drawable>
#include <osg/Program>
#include <osg/Texture2D>

namespace Soleil {

  osg::ref_ptr<osg::Program> CreateVolumetricFogProgram();

  class CopyTextureOnCondition : public osg::Drawable
  {
  public:
    typedef std::function<bool(const osg::Vec3& eye)> Condition;

    class CopyCallBack : public osg::Drawable::DrawCallback
    {
    public:
      CopyCallBack(Condition condition, osg::ref_ptr<osg::Texture2D> texture);
      void drawImplementation(osg::RenderInfo&,
                              const osg::Drawable*) const override;

    protected:
      Condition                    condition;
      osg::ref_ptr<osg::Texture2D> texture;
    };

    CopyTextureOnCondition(Condition                    condition,
                           osg::ref_ptr<osg::Texture2D> texture);
  };

  class ClearScreenOnCondition : public osg::Camera::DrawCallback
  {
  public:
    typedef std::function<bool(const osg::Vec3& eye)> Condition;

  public:
    ClearScreenOnCondition(Condition condition);
    void operator()(osg::RenderInfo& renderInfo) const override;

  private:
    Condition condition;
  };

} // Soleil

#endif /* SOLEIL__VOLUMETRICFOG_H_ */
