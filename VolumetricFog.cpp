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

#include "VolumetricFog.h"

#include "Logger.h"
#include "SceneManager.h"
#include "Utils.h"
#include <osg/CullFace>
#include <osg/Program>
#include <osg/Shader>
#include <osgDB/ReadFile>
#include <osgUtil/DelaunayTriangulator>

namespace Soleil {

  // The vertex shader will be used after the main scene is rendered to texture.
  // It simply transfers the positions and texture coordinates.
  static const char* vertSource = {"void main(void)\n"
                                   "{\n"
                                   "gl_Position = ftransform();\n"
                                   "gl_TexCoord[0] = gl_MultiTexCoord0;\n"
                                   "}\n"};
  // static const char* fragSource = {
  //   "uniform sampler2D sceneTex;\n"
  //   "uniform sampler2D bufferA;\n"
  //   "uniform sampler2D bufferB;\n"
  //   "vec4 fogColor = vec4(0.776, 0.839, 0.851, 1.0);\n"
  //   "void main(void)\n"
  //   "{\n"
  //   "float alpha = texture2D(bufferA, gl_TexCoord[0].st).r - "
  //   "texture2D(bufferB, gl_TexCoord[0].st).r;"
  //   "if (alpha < 0.0) alpha = 0.0;" // TODO:
  //   // Get scene and compute greyscale intensity
  //   "vec4 c = mix(texture2D(sceneTex, gl_TexCoord[0].st), fogColor, alpha);
  //   \n "
  //   "gl_FragColor = c;\n"
  //   "}\n"};

  osg::ref_ptr<osg::Program> CreateVolumetricFogProgram()
  {
    auto frag = osgDB::readShaderFile("../media/shaders/volumetricFog.frag");
    osg::ref_ptr<osg::Program> program = new osg::Program;
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
    // program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
    program->addShader(frag);
    return program;
  }

  CopyTextureOnCondition::CopyCallBack::CopyCallBack(
    Condition condition, osg::ref_ptr<osg::Texture2D> texture)
    : condition(condition)
    , texture(texture)
  {
  }

  void CopyTextureOnCondition::CopyCallBack::drawImplementation(
    osg::RenderInfo& renderInfo, const osg::Drawable*) const
  {
    // TODO: Just get Trans
    osg::Vec3d eye;
    osg::Vec3d center;
    osg::Vec3d up;
    renderInfo.getView()->getCamera()->getViewMatrixAsLookAt(eye, center, up);
    // SOLEIL__LOGGER_DEBUG("Camera position: ", eye);

    if (condition(eye) /*Camera is not in fog*/) {
      texture->copyTexImage2D(*renderInfo.getState(), 0, 0,
                              texture->getTextureWidth(),
                              texture->getTextureHeight());
    } // else {
    //   // texture->getC
    //   texture->apply(*renderInfo.getState());
    //   //GLuint clearColor[4] = {0, 0, 0, 0};
    //   // glClearTexImage(GL_TEXTURE_2D, 0, GL_BGRA, GL_UNSIGNED_BYTE,
    //   // &clearColor);
    // }
  }

  CopyTextureOnCondition::CopyTextureOnCondition(
    Condition condition, osg::ref_ptr<osg::Texture2D> texture)
  {
    this->setDrawCallback(new CopyCallBack(condition, texture));
  }

  ClearScreenOnCondition::ClearScreenOnCondition(Condition condition)
    : condition(condition)
  {
  }

  void ClearScreenOnCondition::operator()(osg::RenderInfo& renderInfo) const
  {
    // TODO: Just get Trans
    osg::Vec3d eye;
    osg::Vec3d center;
    osg::Vec3d up;
    renderInfo.getView()->getCamera()->getViewMatrixAsLookAt(eye, center, up);

    if (condition(eye) /*Camera is not in fog*/) {
      // Keep the copied texture
      renderInfo.getCurrentCamera()->setClearMask(0);
    } else {
      // Camera is in the fog, clear texture
      renderInfo.getCurrentCamera()->setClearColor(
        osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
      renderInfo.getCurrentCamera()->setClearDepth(0.0f);
      renderInfo.getCurrentCamera()->setClearMask(GL_COLOR_BUFFER_BIT |
                                                  GL_DEPTH_BUFFER_BIT);
      // renderInfo.getCurrentCamera()->getGraphicsContext()->clear();
      // renderInfo.getState()->getGraphicsContext()->clear();
      // SOLEIL__LOGGER_DEBUG("CLEAR");
      // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
  }

  void ApplyVolumetricFog(osgViewer::Viewer&       viewer,
                          osg::ref_ptr<osg::Group> root,
                          osg::ref_ptr<osg::Node>  scene,
                          osg::ref_ptr<osg::Node>  fogModel)
  {
    // // MUST: Be rendered after the scene
    // osg::ref_ptr<osg::Texture2D> sceneBuffer = new osg::Texture2D;
    // sceneBuffer->setTextureSize(1024, 1024); // TODO: Real Size
    // sceneBuffer->setInternalFormat(GL_RGBA);
    // auto drawable = new Soleil::CopyTextureOnCondition(
    //   [](const osg::Vec3& /*eye*/) { return true; }, sceneBuffer);
    // // drawable->getOrCreateStateSet()->setTextureAttributeAndModes(0,
    // tex2D);
    // root->addChild(drawable);
    assert(viewer.getCamera() && viewer.getCamera()->getViewport());
    const float width  = viewer.getCamera()->getViewport()->width();
    const float height = viewer.getCamera()->getViewport()->height();

    osg::ref_ptr<osg::Camera> hudCamera =
      Soleil::createHUDCamera(0.0, 1.0, 0.0, 1.0);
    root->addChild(hudCamera);
    hudCamera->setNodeMask(Soleil::SceneManager::Mask::Render);
    // hudCamera->setClearColor(viewer.getCamera()->getClearColor());

    // Render the scene into an off-screen buffer A, encoding each pixel's w
    // depth
    // as its alpha value- Z Buffering enabled.
    osg::ref_ptr<osg::Texture2D> tex2D = new osg::Texture2D;
    osg::ref_ptr<osg::Camera>    rttCameraFront;
    {
      tex2D->setTextureSize(width, height);
      tex2D->setInternalFormat(GL_DEPTH_COMPONENT24);
      tex2D->setSourceFormat(GL_DEPTH_COMPONENT);
      tex2D->setSourceType(GL_FLOAT);

      rttCameraFront =
        Soleil::createRTTCamera(osg::Camera::DEPTH_BUFFER, tex2D);
      rttCameraFront->addChild(scene);

      auto quad = Soleil::createScreenQuad(.10f, .10f);
      hudCamera->addChild(quad);
      quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex2D);

      root->addChild(rttCameraFront);
    }

    // Render the backside of the fog into off-screen buffer A, encoding each
    // pixel's w depth.
    // osg::ref_ptr<osg::Node> fogModel =
    //   osgDB::readNodeFile("../media/ZincFog.osgt");
    osg::ref_ptr<osg::Camera> rttCameraFogBack;
    {
      rttCameraFogBack =
        Soleil::createRTTCamera(osg::Camera::DEPTH_BUFFER, tex2D);
      rttCameraFogBack->setClearMask(0);
      // cull front faces
      osg::ref_ptr<osg::Group>    g        = new osg::Group;
      osg::ref_ptr<osg::StateSet> stateset = g->getOrCreateStateSet();
      osg::CullFace* cullFace = new osg::CullFace(osg::CullFace::Mode::FRONT);
      stateset->setAttributeAndModes(cullFace,
                                     osg::StateAttribute::ON |
                                       osg::StateAttribute::OVERRIDE |
                                       osg::StateAttribute::PROTECTED);

      g->addChild(fogModel);
      rttCameraFogBack->addChild(g);
      // auto quad =
      //   Soleil::createScreenQuad(.10f, .10f, osg::Vec3(0.0f, 0.1f, 0.0f));
      // hudCamera->addChild(quad);
      // quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex2D);

      root->addChild(rttCameraFogBack);
      // root->addChild(g);
    }

    // If the camera is not inside fog, render the scene into an off-screen
    // buffer
    // B (or copy it from buffer A before step 2 takes place), using the w depth
    // alpha encoding. Otherwise, skip this step.
    osg::ref_ptr<osg::Texture2D> bufferB = new osg::Texture2D;
    //{
    bufferB->setTextureSize(width, height);
    bufferB->setInternalFormat(GL_DEPTH_COMPONENT24);
    bufferB->setSourceFormat(GL_DEPTH_COMPONENT);
    bufferB->setSourceType(GL_FLOAT);

#if 0
    osg::ref_ptr<osg::Camera>    rttCameraFrontAgain;
    rttCameraFrontAgain =
      Soleil::createRTTCamera(osg::Camera::DEPTH_BUFFER, bufferB);
    rttCameraFrontAgain->addChild(scene);
    // auto quad =
    //   Soleil::createScreenQuad(1.0f, 1.0f);
    // rttCameraFrontAgain->addChild(quad);

    //If used camera has to render only if the main camera is not inside the fog
    //root->addChild(rttCameraFrontAgain);
#elif 1
    // TODO: Get the geometry
    assert(Soleil::GetNodeByName(*fogModel, "Cube_0"));
    osg::ref_ptr<osg::Geometry> cube =
      Soleil::GetNodeByName(*fogModel, "Cube_0")->asGeometry();
    assert(cube);
    osg::Vec3Array* points =
      dynamic_cast<osg::Vec3Array*>(cube->getVertexArray());
    assert(points);

    osg::ref_ptr<osgUtil::DelaunayConstraint> c =
      new osgUtil::DelaunayConstraint;
    c->getPoints(points);

    auto box = cube->computeBoundingBox();

    auto isNotInsideFog = [fogModel, c, box](const osg::Vec3& eye) {
      // return fogModel->getBound().contains(eye) == false;
      // return c->contains(eye) == false;
      return box.contains(eye) == false;
      // TODO: HULL
    };

    auto copyTexture =
      new Soleil::CopyTextureOnCondition(isNotInsideFog, bufferB);
    copyTexture->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex2D);
    rttCameraFogBack->addChild(copyTexture);
#endif
    //}

    // Render the front side of the fog volume into off-screen buffer B with w
    // alpha encoding. Since the fog volume should be in front of parts of the
    // object that are obscured by fog, it will replace them at those pixels.
    osg::ref_ptr<osg::Camera> rttCameraFogFront;
    {
      rttCameraFogFront =
        Soleil::createRTTCamera(osg::Camera::DEPTH_BUFFER, bufferB);
      rttCameraFogFront->setClearMask(0);
      rttCameraFogFront->setPreDrawCallback(
        new Soleil::ClearScreenOnCondition(isNotInsideFog));
      // cull front faces
      osg::ref_ptr<osg::Group>    g        = new osg::Group;
      osg::ref_ptr<osg::StateSet> stateset = g->getOrCreateStateSet();
      osg::CullFace* cullFace = new osg::CullFace(osg::CullFace::Mode::BACK);
      stateset->setAttributeAndModes(cullFace,
                                     osg::StateAttribute::ON |
                                       osg::StateAttribute::OVERRIDE |
                                       osg::StateAttribute::PROTECTED);

      g->addChild(fogModel);
      rttCameraFogFront->addChild(g);
      auto quad =
        Soleil::createScreenQuad(.10f, .10f, osg::Vec3(0.0f, 0.1f, 0.0f));
      hudCamera->addChild(quad);
      quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, bufferB);

      root->addChild(rttCameraFogFront);
    }

    // Subtract the two buffers in screen space using the alpha value to blend
    // on
    // a fog mask.
    osg::ref_ptr<osg::Texture2D> sceneBuffer = new osg::Texture2D;
    sceneBuffer->setTextureSize(width, height);
    sceneBuffer->setInternalFormat(GL_RGBA);
    osg::ref_ptr<osg::Camera> rttCamera =
      Soleil::createRTTCamera(osg::Camera::COLOR_BUFFER, sceneBuffer);
    // TODO: So that I can render thing below it
    // rttCamera->setClearColor(viewer.getCamera()->getClearColor());
    rttCamera->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
    rttCamera->addChild(scene);
    root->addChild(rttCamera);

    auto quad = Soleil::createScreenQuad(1.0f, 1.0f);
    quad->setName("FogSceneResult");
    hudCamera->addChild(quad);
    osg::StateSet* stateset = quad->getOrCreateStateSet();
    stateset->setTextureAttribute(0, sceneBuffer);
    stateset->setTextureAttribute(1, tex2D);
    stateset->setTextureAttribute(2, bufferB);
    stateset->setAttribute(Soleil::CreateVolumetricFogProgram());
    stateset->addUniform(new osg::Uniform("sceneTex", 0));
    stateset->addUniform(new osg::Uniform("bufferA", 1));
    stateset->addUniform(new osg::Uniform("bufferB", 2));
    // TODO: Removed for the video (added to root)
    // stateset->addUniform(
    //   new osg::Uniform("fogColor", osg::Vec4(0.776f, 0.839f, 0.851f, 1.0f)));
    // stateset->addUniform(new osg::Uniform("density", 5.0f));
    // stateset->addUniform(new osg::Uniform("doSquared", 0));
  }

} // Soleil
