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

static bool computed = false;

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
      case KEY::KEY_O: {
        computed = true;
      }
    }
  }
  return TrackballManipulator::handle(event, us);
}

/* --- Infinite Plane --- */

class InfinitePlane : public osg::Geometry
{
public:
  InfinitePlane(osg::Plane plane);

  void update(const osg::Matrix& invertedViewProjectionMatrix);

protected:
  ~InfinitePlane() {}

protected:
  osg::Plane plane;

protected:
  osg::ref_ptr<osg::Vec3Array> vertices;
};

static constexpr int NumVertices = 12;

InfinitePlane::InfinitePlane(osg::Plane plane)
  : plane(plane)
  , vertices(
      new osg::Vec3Array(osg::Array::Binding::BIND_PER_VERTEX, NumVertices))
{
  // (*vertices)[0] = osg::Vec3();
  // (*vertices)[1] = osg::Vec3();
  // (*vertices)[2] = osg::Vec3();
  // (*vertices)[3] = osg::Vec3();

  float a        = 100.0f;
  (*vertices)[0] = osg::Vec3(-1, -1, 0) * a;
  (*vertices)[1] = osg::Vec3(1, -1, 0) * a;
  (*vertices)[2] = osg::Vec3(1, 1, 0) * a;
  (*vertices)[3] = osg::Vec3(-1, 1, 0) * a;

  osg::ref_ptr<osg::Vec3Array> normals =
    new osg::Vec3Array(osg::Array::Binding::BIND_PER_VERTEX, NumVertices);
  osg::ref_ptr<osg::Vec4Array> colors =
    new osg::Vec4Array(osg::Array::Binding::BIND_PER_VERTEX, NumVertices);

  for (int i = 0; i < NumVertices; ++i) {
    //(*normals)[i] = plane.getNormal();
    (*normals)[i] = VectorUp();
    if (i < 4) {
      (*colors)[i] = osg::Vec4(1, 0, 0, 0.1);
    } else if (i < 8) {
      (*colors)[i] = osg::Vec4(0, 0, 1, 0.1);
    } else {
      (*colors)[i] = osg::Vec4(0, 1, 0, 1);
    }
  }

  setDataVariance(osg::Object::DataVariance::DYNAMIC);
  setUseDisplayList(false);
  setUseVertexBufferObjects(true);
  setVertexArray(vertices);
  setNormalArray(normals);
  setColorArray(colors);
  // addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));
  // addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 4, 4));
  addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 8, 4));
  this->setCullingActive(false);
  dirtyGLObjects();
  // TODO: Do not cull
}

static bool
IntersectSegmentPlane(const osg::Vec3& src, const osg::Vec3& dest,
                      const osg::Plane& p, float& t, osg::Vec3& q)
{
  const osg::Vec3 direction = dest - src;
  t = (p.asVec4().w() - (p.getNormal() * src)) / (p.getNormal() * direction);

  SOLEIL__LOGGER_DEBUG("T=", t);
  // If t in [0..1] compute and return intersection point
  if (t >= 0.0f && t <= 1.0f) {
    q = src + direction * t;
    return true;
  }

  return false;
}

void
InfinitePlane::update(const osg::Matrix& invertedViewProjectionMatrix)
{
  if (computed) return;

// clang-format off
  #if 1
  // OpenGL coorindates (render front and back planes)
  osg::Vec3 CubeCoordinates[8]{
      osg::Vec3(-1.0f, -1.0f, 1.0f),
      osg::Vec3( 1.0f, -1.0f, 1.0f),
      osg::Vec3( 1.0f,  1.0f, 1.0f),
      osg::Vec3(-1.0f,  1.0f, 1.0f),
      
      osg::Vec3(-1.0f, -1.0f, -1.0f),
      osg::Vec3( 1.0f, -1.0f, -1.0f),
      osg::Vec3( 1.0f,  1.0f, -1.0f),
      osg::Vec3(-1.0f,  1.0f, -1.0f)};
  #elif 0
  // OSG coorindates (render up and down plane)
  osg::Vec3 CubeCoordinates[8]{
    osg::Vec3(  -1.0f, 1.0f, -1.0f),
      osg::Vec3( 1.0f, 1.0f, -1.0f),
      osg::Vec3( 1.0f, 1.0f,  1.0f),
      osg::Vec3(-1.0f, 1.0f,  1.0f),
      
      osg::Vec3(-1.0f, -1.0f, -1.0f),
      osg::Vec3( 1.0f, -1.0f, -1.0f),
      osg::Vec3( 1.0f, -1.0f,  1.0f),
      osg::Vec3(-1.0f, -1.0f,  1.0f)};

  #elif 0
  float ar = 1920.0f / 1080.0f;
  float fov = 60.f;
  float near = 0.1f;
  float far = 1000.f;
  float halfHeight = tanf(osg::DegreesToRadians(fov / 2.f));
  float halfWidth = halfHeight * ar;

  float xn = halfWidth * near;
  float xf = halfWidth * far;
  float yn = halfHeight * near;
  float yf = halfHeight * far;

  osg::Vec3 CubeCoordinates[] 
  {
    // near face
    {xn,  near, yn   },
    {-xn, near,  yn },
    {xn,  near, -yn },
    {-xn, near,  -yn},

    // far face
    {xf  , far, yf  },
    {-xf, far , yf  },
    {xf  , far, -yf },
    {-xf, far , -yf },
};
  #endif
// clang-format on

#if 0
  for (int i = 0; i < 4 /*8*/; ++i) {
    (*vertices)[i] = invertedViewProjectionMatrix * CubeCoordinates[i];
    SOLEIL__LOGGER_DEBUG("(*vertices)[", i, "]=", (*vertices)[i]);
  }
#else
  // Draw Frustrum
  for (int i = 0; i < 8; ++i) {
    osg::Vec4 r =
      osg::Vec4(CubeCoordinates[i], 1.0f) * invertedViewProjectionMatrix;
    r = r / r.w();
    // (*vertices)[i].x() = r.x() * (1.0f / r.w());
    // (*vertices)[i].y() = r.y() * (1.0f / r.w());
    // (*vertices)[i].z() = r.z() * (1.0f / r.w());
    (*vertices)[i].x() = r.x();
    (*vertices)[i].y() = r.y();
    (*vertices)[i].z() = r.z();

    SOLEIL__LOGGER_DEBUG("(*vertices)[", i, "]=", (*vertices)[i]);
  }

  // Render Plane
  for (int i = 0; i < 4; ++i) {
    osg::Vec3 intersection{};
    float     t = 0.0f;

    if (IntersectSegmentPlane(vertices->at(i), vertices->at(i + 4), plane, t,
                              intersection)) {
      (*vertices)[i + 8] = intersection;
      SOLEIL__LOGGER_DEBUG("Intersection: (", i, "): ", intersection);
    } else {
      // TODO:
      //(*vertices)[i + 8] = intersection;
      SOLEIL__LOGGER_DEBUG(i, "  :(");
    }
  }
#endif
  vertices->dirty();

  // dirtyBound();
}

class InfinitePlaneCallback : public osg::Callback
{
  bool run(osg::Object* object, osg::Object* data);
};

bool
InfinitePlaneCallback::run(osg::Object* object, osg::Object* data)
{
  osgUtil::CullVisitor* c = data->asNodeVisitor()->asCullVisitor();
  assert(c && "Not a cull visitor");

  int u = 0;
  SOLEIL__LOGGER_DEBUG("----------------------------------------");
  for (auto const& v : c->getState()->getViewFrustum().getPlaneList()) {
    SOLEIL__LOGGER_DEBUG(u++, ": ", v);
  }

// c->getCurrentCamera()->setProjectionMatrixAsPerspective(0.8f, 1920.0f /
// 1080.0f,
//                                                      0.1, 1000);

#if 1
  osg::Matrix invertedViewProjectionMatrix =
    osg::Matrix::inverse(c->getCurrentCamera()->getViewMatrix() *
                         c->getCurrentCamera()->getProjectionMatrix());
#elif 0
  // clang-format off
  osg::Matrix invertedViewProjectionMatrix;
  // invertedViewProjectionMatrix.transpose(osg::Matrix(
  // 					   1, 0, 0, 0,
  // 					   0, 1, 0, 0,
  // 					   0, 0, 1, 0,
  // 					   0, 0, 1, 0
  // 						     ));

  // invertedViewProjectionMatrix.preMult(osg::Matrix::inverse(
  // 			 c->getCurrentCamera()->getViewMatrix() *
  //                        c->getCurrentCamera()->getProjectionMatrix()));
// clang-format on
#elif 0
  osg::Matrix invertedViewProjectionMatrix =
    osg::Matrix::inverse(c->getCurrentCamera()->getProjectionMatrix() *
                         c->getCurrentCamera()->getViewMatrix());

#endif
  SOLEIL__LOGGER_DEBUG((void*)c->getCurrentCamera(), ">>> ",
                       c->getCurrentCamera()->getProjectionMatrix(), "\n====\n",
                       c->getState()->getProjectionMatrix());

  static_cast<InfinitePlane*>(object)->update(invertedViewProjectionMatrix);

  return traverse(object, data);
}

/* --- Infinite Plane --- */

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
  // scene->addChild(playerEntityGroup);

  // scene->addChild(osgDB::readNodeFile("../media/ZincMoutonRose.osgt"));

  // ------------------------------
  osg::ref_ptr<InfinitePlane> p =
    new InfinitePlane(osg::Plane(osg::Vec3(0, 0, 1), osg::Vec3()));
  p->addCullCallback(new InfinitePlaneCallback);
  osg::ref_ptr<osg::Geode> g = new osg::Geode;
  g->addDrawable(p);
  scene->addChild(g);

  osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
  vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
  vertices->push_back(osg::Vec3(1.0f, 0.0f, 0.0f));
  vertices->push_back(osg::Vec3(1.0f, 0.0f, 1.0f));
  vertices->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));
  osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
  normals->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));
  osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
  colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
  colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
  colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
  colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
  osg::ref_ptr<osg::Geometry> quad = new osg::Geometry;
  quad->setVertexArray(vertices.get());
  quad->setNormalArray(normals.get());
  quad->setNormalBinding(osg::Geometry::BIND_OVERALL);
  quad->setColorArray(colors.get());
  quad->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
  quad->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));
  g->addDrawable(quad);
  // ------------------------------

  // viewer.getCamera()->setComputeNearFarMode(
  //   osg::CullSettings::COMPUTE_NEAR_FAR_USING_PRIMITIVES);
  // viewer.getCamera()->setComputeNearFarMode(
  //   osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
  viewer.realize();

  viewer.getCamera()->setProjectionMatrixAsPerspective(0.8f, 1920.0f / 1080.0f,
                                                       0.1, 1000);

  //----------------------------------------------------
  // Setup of the ImGUI
  // osg::Camera* camera = viewer.getCamera();
  // assert(camera);
  // osg::ref_ptr<ImGUIEventHandler> imguiHandler = new
  // ImGUIEventHandler{drawUI};
  // viewer.addEventHandler(imguiHandler);
  // camera->setPreDrawCallback(new ImGUINewFrame{*imguiHandler});
  // camera->setPostDrawCallback(new ImGUIRender{*imguiHandler});

  osg::ref_ptr<DebugTrackball> cam = new DebugTrackball;
  cam->root                        = scene;
  viewer.getCamera()->setCullMask(1);
  viewer.getUpdateVisitor()->setTraversalMask(2);
  viewer.setCameraManipulator(cam);
  viewer.setLightingMode(osg::View::SKY_LIGHT);
  return viewer.run();

  double previousTime = viewer.getFrameStamp()->getSimulationTime();
  while (!viewer.done()) {
    viewer.frame();

    // Update events
    // double deltaTime =
    //   viewer.getFrameStamp()->getSimulationTime() - previousTime;
    // Soleil::EventManager::ProcessEvents(deltaTime);
    // previousTime = viewer.getFrameStamp()->getSimulationTime();
  }
  return 0;
}
