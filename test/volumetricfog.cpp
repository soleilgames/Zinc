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

osg::Vec4 fogColor(0.776f, 0.839f, 0.851f, 1.0f);
float     fogDensity     = 5.0f;
bool      fogDoSquared   = 0;
bool      renderFogModel = false;

static void
drawUI(void)
{
  ImGui::ShowDemoWindow(nullptr);

  ImGui::Begin("Volumetrix fog");
  // ImVec4 color(fogColor.r(), fogColor.g(), fogColor.b(), fogColor.a());
  // ImGui::ColorPicker4("Color", (float*)&color);
  ImGui::ColorPicker4("Color", (float*)fogColor.ptr());
  ImGui::SliderFloat("Density", &fogDensity, 0.0f, 10.0f);
  ImGui::Checkbox("Squared", &fogDoSquared);
  ImGui::Separator();
  ImGui::Checkbox("Render Fog model", &renderFogModel);
  ImGui::End();
}

template <typename T> class UniCallBack : public osg::Uniform::Callback
{
public:
  UniCallBack(T& value)
    : value(value)
  {
  }

  void operator()(osg::Uniform* uniform, osg::NodeVisitor* /*nv*/) override
  {
    // SOLEIL__LOGGER_DEBUG("---------------------------- SET:", value);
    uniform->set(value);
  }

public:
  T& value;
};

class SwitchCB : public osg::Callback
{
  bool run(osg::Object* object, osg::Object* data) override
  {
    if (renderFogModel) {
      object->asNode()->asSwitch()->setAllChildrenOn();
    } else {
      object->asNode()->asSwitch()->setAllChildrenOff();
    }
    return this->traverse(object, data);
  }
};

class DebugTrackball : public osgGA::TrackballManipulator
{
public:
  virtual bool handle(const osgGA::GUIEventAdapter& ea,
                      osgGA::GUIActionAdapter&      us) override;
  osg::ref_ptr<osg::Node> root;
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
  osg::ref_ptr<osg::Group> root      = new osg::Group;
  osg::ref_ptr<osg::Group> superroot = new osg::Group;
  osg::ref_ptr<osg::Group> scene     = new osg::Group;
  root->addChild(scene);
  superroot->addChild(root);
  viewer.setSceneData(superroot);

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

  scene->addChild(osgDB::readNodeFile("../media/ZincMoutonRose.osgt"));

  viewer.getCamera()->setComputeNearFarMode(
    osg::CullSettings::COMPUTE_NEAR_FAR_USING_PRIMITIVES);

  viewer.realize();

  osg::ref_ptr<osg::StateSet> s =
    new osg::StateSet(*viewer.getCamera()->getStateSet(),
                      osg::CopyOp(osg::CopyOp::Options::DEEP_COPY_ALL));

  //----------------------------------------------------
  // Setup of the Fog
  osg::ref_ptr<osg::Node> fogModel =
    osgDB::readNodeFile("../media/ZincVolumeFogTest.osgt");
  Soleil::ApplyVolumetricFog(viewer, root, scene, fogModel);
  //
  osg::ref_ptr<osg::Node> n = Soleil::GetNodeByName(*root, "FogSceneResult");
  // n->getStateSet()
  //   ->getUniform("fogColor")
  //   ->setUpdateCallback(new UniCallBack<osg::Vec4>(fogColor));
  // n->getStateSet()->getUniform("density")->setUpdateCallback(
  //   new UniCallBack<float>(fogDensity));
  // n->getStateSet()
  //   ->getUniform("doSquared")
  //   ->setUpdateCallback(new UniCallBack<bool>(fogDoSquared));
  // TODO: Uniform has to be removed from the node and added to root in order to
  // have the UniformCallback in action
  {
    osg::ref_ptr<osg::Uniform> u = new osg::Uniform("fogColor", osg::Vec4());
    u->setUpdateCallback(new UniCallBack<osg::Vec4>(fogColor));
    root->getOrCreateStateSet()->addUniform(u);
  }
  {
    osg::ref_ptr<osg::Uniform> u = new osg::Uniform("density", 0.0f);
    u->setUpdateCallback(new UniCallBack<float>(fogDensity));
    root->getOrCreateStateSet()->addUniform(u);
  }
  {
    osg::ref_ptr<osg::Uniform> u = new osg::Uniform("doSquared", 0);
    u->setUpdateCallback(new UniCallBack<bool>(fogDoSquared));
    root->getOrCreateStateSet()->addUniform(u);
  }

  osg::ref_ptr<osg::Switch> swi = new osg::Switch;
  swi->addChild(fogModel);
  swi->addEventCallback(new SwitchCB);
  scene->addChild(swi);

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

    // assert(*s == *viewer.getCamera()->getStateSet());

    // Update events
    double deltaTime =
      viewer.getFrameStamp()->getSimulationTime() - previousTime;
    Soleil::EventManager::ProcessEvents(deltaTime);
    previousTime = viewer.getFrameStamp()->getSimulationTime();
  }
  return 0;
}
