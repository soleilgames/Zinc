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

#include "Actor.h"
#include "AlienCraft.h"
#include "EntityGroup.h"
#include "EventManager.h"
#include "ImGuiHandler.h"
#include "ParticleObjects.h"
#include "SceneManager.h"
#include "Utils.h"
#include "VolumetricFog.h"
#include <functional>
#include <osg/BlendColor>
#include <osg/BlendEquation>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Switch>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osgParticle/ModularProgram>
#include <osgParticle/ParticleSystemUpdater>
#include <osgViewer/Viewer>

class InfinitePlane : public osg::Transform
{
public:
  InfinitePlane();
  // META_Node(Soleil, InfinitePlane);

  virtual bool computeLocalToWorldMatrix(osg::Matrix&      matrix,
                                         osg::NodeVisitor* nv) const;
  virtual bool computeWorldToLocalMatrix(osg::Matrix&      matrix,
                                         osg::NodeVisitor* nv) const;

protected:
  virtual ~InfinitePlane() {}
};

InfinitePlane::InfinitePlane()
{
  // fogPlane->getOrCreateStateSet()->setRenderingHint(
  //   osg::StateSet::TRANSPARENT_BIN);
  // fogPlane->getOrCreateStateSet()->setMode(GL_LIGHTING,
  //                                          osg::StateAttribute::OFF);
  // osg::ref_ptr<osg::StateAttribute> blend =
  //   new osg::BlendFunc(osg::BlendFunc::BlendFuncMode::SRC_ALPHA,
  //                      osg::BlendFunc::BlendFuncMode::ONE_MINUS_SRC_ALPHA);
  // {
  //   osg::StateSet* stateset = fogPlane->getOrCreateStateSet();
  //   stateset->setAttributeAndModes(blend, osg::StateAttribute::ON);
  //   auto mask = fogPlane->getNodeMask();
  //   mask &= ~Soleil::SceneManager::Mask::Collision;
  //   mask &= ~Soleil::SceneManager::Mask::Shootable;
  //   fogPlane->setNodeMask(mask);
  // }
  setReferenceFrame(osg::Transform::ABSOLUTE_RF);
  setCullingActive(false);
  osg::StateSet* ss = getOrCreateStateSet();
  // ss->setAttributeAndModes(new osg::Depth(osg::Depth::LEQUAL, 1.0f, 1.0f));
  ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
  ss->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
  ss->setRenderBinDetails(5, "RenderBin");
}

bool
InfinitePlane::computeLocalToWorldMatrix(osg::Matrix&      matrix,
                                         osg::NodeVisitor* nv) const
{

  // n, f	= distances to near, far planes
  // e		= focal length = 1 / tan(FOV / 2)
  // a		= viewport height / width
  // ---
  // e	0	0	0
  // 0	e/a	0	0
  // 0	0	-1	-2n
  // 0	0	-1	0

  if (nv && nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR) {
    osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);

    double fovy = 0.8;
    double a    = 1080.0 / 1920.0;
    double n    = 0.01;
    double zFar = 1000.0;
    // const bool isPersp =
    //   cv->getCurrentCamera()->getProjectionMatrixAsPerspective(fovy, a, n,
    //                                                            zFar);
    float e = 1.0f / tan(fovy / 2.0f);
    a       = cv->getCurrentCamera()->getViewport()->height() /
        cv->getCurrentCamera()->getViewport()->width();

    // clang-format off
    osg::Matrix infinite(e	, 0	, 0	, 0		,
			 0	, e/a	, 0	, 0		,
			 0	, 0	, -1.0f	, -2.0f * n	,
			 0	, 0	, -1.0f	, 0
			 );
    // clang-format on
    SOLEIL__LOGGER_DEBUG(
      "fovy=", fovy, ", a=", a, ", n=", n, ", zFar=", zFar,
      "\nINFINITY = ", infinite,
      "\ninfinite * osg::Vec3(0, 1, 0) = ", infinite * osg::Vec3(0, 1, 0));

    matrix.preMult(infinite);
    // matrix = infinite;
    return true;
  } else {
    return osg::Transform::computeLocalToWorldMatrix(matrix, nv);
  }
}

bool
InfinitePlane::computeWorldToLocalMatrix(osg::Matrix&      matrix,
                                         osg::NodeVisitor* nv) const
{
  if (nv && nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR) {
    assert(false);
    // TODO:
    osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
    matrix.postMult(osg::Matrix::translate(-cv->getEyeLocal()));
    return true;
  } else
    return osg::Transform::computeWorldToLocalMatrix(matrix, nv);
}

/* --- Infinite Plane --- */

static void
drawUI(void)
{
  ImGui::ShowDemoWindow(nullptr);
}

class DebugTrackball : public osgGA::TrackballManipulator
{
public:
  virtual bool handle(const osgGA::GUIEventAdapter& ea,
                      osgGA::GUIActionAdapter&      us) override;
  osg::ref_ptr<osg::Node> root;
};

class CamCB : public osg::Callback
{
public:
  float angle;

  CamCB()
    : angle(0.0f)
  {
  }

  bool run(osg::Object* object, osg::Object* data)
  {
#if 0
    const float TwoPi = osg::PIf * 2.0f;
    angle += 0.01f;
    if (angle > TwoPi) angle = angle - TwoPi;
    
    osg::Vec3 direction =
      osg::Matrix::rotate(angle, 0, 0, 1) * osg::Vec3(0, 1, 0);

    object->asCamera()->setViewMatrix(
      osg::Matrix::lookAt(osg::Vec3(0, 0, 0), direction, VectorUp()));

    SOLEIL__LOGGER_DEBUG("Current angle: ", osg::RadiansToDegrees(angle));
#else
    angle += 0.1f;
    // angle = 10.0f;
    osg::Vec3 direction(0, 0, 50.0f);
    object->asCamera()->setViewMatrix(osg::Matrix::lookAt(
      direction, direction + osg::Vec3(0, 1.0f, 0), VectorUp()));

    SOLEIL__LOGGER_DEBUG("Current angle: ", angle);
#endif

    return traverse(object, data);
  }
};

bool
DebugTrackball::handle(const osgGA::GUIEventAdapter& event,
                       osgGA::GUIActionAdapter&      us)
{
  using KEY  = osgGA::GUIEventAdapter::KeySymbol;
  using TYPE = osgGA::GUIEventAdapter::EventType;

  if (event.getEventType() == TYPE::KEYUP) {
    switch (event.getKey()) {
      case KEY::KEY_P: {
        int mask = root->getNodeMask();
        if (mask == 1) {
          root->setNodeMask(0xffffffff);
        } else {
          root->setNodeMask(1);
        };
        break;
      }
    }
  }
  return TrackballManipulator::handle(event, us);
}

int
main(int // argc
     ,
     char* // argv
     [])
{
  osgViewer::Viewer        viewer;
  osg::ref_ptr<osg::Group> scene = new osg::Group;
  viewer.setSceneData(scene);

  Soleil::EventManager::Init();
  Soleil::SceneManager::Init(scene);

  // Add the Player -------------------------------------
  osg::ref_ptr<osg::Node> playerModel =
    osgDB::readNodeFile("../media/ZincAF100.osgt");
  osg::ref_ptr<osg::PositionAttitudeTransform> player =
    new osg::PositionAttitudeTransform;
  player->setName("Player");
  player->addChild(playerModel);
  player->setPosition(osg::Vec3(0, 0, 10));
  osg::ref_ptr<Soleil::EntityGroup> playerEntityGroup = new Soleil::EntityGroup;
  playerEntityGroup->addChild(player);
  scene->addChild(playerEntityGroup);

  //scene->addChild(osgDB::readNodeFile("../media/ZincMoutonRose.osgt"));

  viewer.getCamera()->setComputeNearFarMode(
    osg::CullSettings::COMPUTE_NEAR_FAR_USING_PRIMITIVES);
  viewer.realize();

  // -------------------------------------
  {
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    // camera->setAllowEventFocus(false);
    camera->setClearColor(osg::Vec4(1, 0, 0, 0));
    //camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    //camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
    // camera->setClearMask(GL_DEPTH_BUFFER_BIT);
    camera->setClearMask(0);

    // osg::ref_ptr<InfinitePlane> inf  = new InfinitePlane();
    osg::ref_ptr<osg::Node> fogPlane =
      osgDB::readNodeFile("../media/ZincFogSurface.osgt");
    fogPlane->getOrCreateStateSet()->setRenderingHint(
      osg::StateSet::TRANSPARENT_BIN);
    fogPlane->getOrCreateStateSet()->setMode(GL_LIGHTING,
                                             osg::StateAttribute::OFF);
    osg::ref_ptr<osg::StateAttribute> blend =
      new osg::BlendFunc(osg::BlendFunc::BlendFuncMode::SRC_ALPHA,
                         osg::BlendFunc::BlendFuncMode::ONE_MINUS_SRC_ALPHA);
    {
      osg::StateSet* stateset = fogPlane->getOrCreateStateSet();
      stateset->setAttributeAndModes(blend, osg::StateAttribute::ON);
      auto mask = fogPlane->getNodeMask();
      mask &= ~Soleil::SceneManager::Mask::Collision;
      mask &= ~Soleil::SceneManager::Mask::Shootable;
      fogPlane->setNodeMask(mask);
    }
    // fogPlane->setCullingActive(false);
    // inf->addChild(fogPlane);
    camera->addChild(fogPlane);
    ///////////////////////////////
    double     fovy = 0.8;
    double     a    = 1080.0 / 1920.0;
    double     n    = 1000.01;
    double     zFar = 1000.0;
    const bool isPersp =
      viewer.getCamera()->getProjectionMatrixAsPerspective(fovy, a, n, zFar);
    assert(isPersp);
    float e = 1.0f / tan(fovy / 2.0f);

    // a       = viewer.getCamera()->getViewport()->height() /
    //     viewer.getCamera()->getViewport()->width();
#if 1
    // clang-format off
    osg::Matrix infinite(e	, 0	, 0	, 0		,
			 0	, e/a	, 0	, 0		,
			 0	, 0	, -1.0f	, -2.0f * n	,
			 0	, 0	, -1.0f	, 0
			 );
    // clang-format on
    osg::Matrix infiniteT;
    const bool transposed = infiniteT.transpose(infinite);
    infiniteT = osg::Matrix::inverse(infiniteT);
    assert(transposed);
    SOLEIL__LOGGER_DEBUG("fovy=", fovy, ", a=", a, ", n=", n, ", zFar=", zFar,
                         "\nINFINITY = ", infiniteT,
                         "\ninfiniteT * osg::Vec4(0, 1, 21, 0) = ",
                         infiniteT * osg::Vec4(0, 1, 0, 0));

    camera->setProjectionMatrix(infiniteT);
#else
    osg::Vec4d frustum;
    bool       isFrustum = viewer.getCamera()->getProjectionMatrixAsFrustum(
      frustum.x(), frustum.y(), frustum.z(), frustum.w(), n, zFar);
    assert(isFrustum);
    SOLEIL__LOGGER_DEBUG("frustum=", frustum);
    const float r = frustum.y();
    const float t = frustum.w();
    // clang-format off
    osg::Matrix infinite(n / r	, 0	, 0	, 0		,
			 0	, n / t	, 0	, 0		,
			 0	, 0	, -1.0f	, -2.0f * n	,
			 0	, 0	, -1.0f	, 0
			 );
    // clang-format on
    SOLEIL__LOGGER_DEBUG("fovy=", fovy, ", a=", a, ", n=", n, ", zFar=", zFar,
                         "\nINFINITY = ", infinite,
                         "\ninfinite * osg::Vec4(0, 1, 21, 0) = ",
                         infinite * osg::Vec4(0, 1, 0, 0));

    camera->setProjectionMatrix(infinite);
#endif
    // camera->setViewMatrix(osg::Matrix::lookAt(osg::Vec3(), VectorFront(),
    // VectorUp()));
    // camera->setViewMatrix(
    //   osg::Matrix::lookAt(osg::Vec3(0, 0, 0), osg::Vec3(0, 1, 0), VectorUp()));
    //camera->setEventCallback(new CamCB);

    scene->addChild(camera);
  }
  ///////////////////////////////
  // ------------------------------

  //----------------------------------------------------
  // Setup of the ImGUI
  osg::Camera* camera = viewer.getCamera();
  assert(camera);
  osg::ref_ptr<ImGUIEventHandler> imguiHandler = new ImGUIEventHandler{drawUI};
  viewer.addEventHandler(imguiHandler);
  camera->setPreDrawCallback(new ImGUINewFrame{*imguiHandler});
  camera->setPostDrawCallback(new ImGUIRender{*imguiHandler});

  osg::ref_ptr<DebugTrackball> cam = new DebugTrackball;
  cam->root                        = scene;
  viewer.getCamera()->setCullMask(1);
  viewer.getUpdateVisitor()->setTraversalMask(2);
  viewer.setCameraManipulator(cam);
  viewer.setLightingMode(osg::View::SKY_LIGHT);
  double previousTime = viewer.getFrameStamp()->getSimulationTime();
  while (!viewer.done()) {
    viewer.frame();

    // Update events
    double deltaTime =
      viewer.getFrameStamp()->getSimulationTime() - previousTime;
    Soleil::EventManager::ProcessEvents(deltaTime);
    previousTime = viewer.getFrameStamp()->getSimulationTime();
  }
  return 0;
}
