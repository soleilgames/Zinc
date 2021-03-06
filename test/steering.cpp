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
#include "ParticleObjects.h"
#include "SceneManager.h"
#include "Utils.h"
#include <functional>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osgParticle/ModularProgram>
#include <osgParticle/ParticleSystemUpdater>
#include <osgViewer/Viewer>

constexpr float Range = 50;
std::function<void(const osg::Vec3& position)> CreateAlienCraft;

// ------- Alien Shoot
// TODO: Set this method in ParticleObjects
static osg::ref_ptr<osgParticle::ParticleSystem>
CreateAlienShootPS(osg::Group& parent)
{
  constexpr float                           scale = 1.0f;
  osg::ref_ptr<osgParticle::ParticleSystem> ps =
    new osgParticle::ParticleSystem;

  ps->setDefaultAttributes("../media/textures/alienshoot.png", false, false);

  float radius = 0.4f * scale;
  // float density = 1.2f; // 1.0kg/m^3

  auto& defaultParticleTemplate = ps->getDefaultParticleTemplate();
  defaultParticleTemplate.setLifeTime(1.0);
  // defaultParticleTemplate.setSizeRange(osgParticle::rangef(0.75f, 3.0f));
  // defaultParticleTemplate.setAlphaRange(osgParticle::rangef(0.1f, 1.0f));
  // defaultParticleTemplate.setColorRange(osgParticle::rangev4(
  //   osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f), osg::Vec4(1.0f, 1.0f, 1.0f, 0.0f)));
  defaultParticleTemplate.setRadius(radius);
  // defaultParticleTemplate.setMass(density * radius * radius * radius *
  // osg::PI *
  //                                 4.0f / 3.0f);

  // TODO: Try         _program = new osgParticle::FluidProgram;

  osg::ref_ptr<osgParticle::ModularProgram> program =
    new osgParticle::ModularProgram;
  program->setParticleSystem(ps);
  program->addOperator(new Soleil::CollisionOperator);

  osg::ref_ptr<osg::Geode> geode = new osg::Geode;
  geode->addDrawable(ps);
  parent.addChild(program);
  parent.addChild(geode);

  // parent->addChild(parent2);

  osg::ref_ptr<osgParticle::ParticleSystemUpdater> updater =
    new osgParticle::ParticleSystemUpdater;
  updater->addParticleSystem(ps);
  parent.addChild(updater);

  // MeshSetLineMode(geometry);

  return ps;
}

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
      case KEY::KEY_O: {
        CreateAlienCraft(osg::Vec3(Random(-0.0f, Range), Random(-0.0f, Range),
                                   Random(-1.0f, 3.0f)));
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
  osg::ref_ptr<osg::Group> root = new osg::Group;
  viewer.setSceneData(root);

  Soleil::EventManager::Init();
  Soleil::SceneManager::Init(root);

  // Add the Player -------------------------------------
  osg::ref_ptr<osg::Node> playerModel =
    osgDB::readNodeFile("../media/ZincAF100.osgt");
  osg::ref_ptr<osg::PositionAttitudeTransform> player =
    new osg::PositionAttitudeTransform;
  player->setName("Player");
  player->addChild(playerModel);
  osg::ref_ptr<Soleil::EntityGroup> playerEntityGroup = new Soleil::EntityGroup;
  playerEntityGroup->addChild(player);
  root->addChild(playerEntityGroup);

  assert(Soleil::GetPathByNodeName(*root, "Player").empty() == false);
  Soleil::SceneManager::RegisterNodePath(
    Soleil::ConstHash("Player"), Soleil::GetPathByNodeName(*root, "Player"));

  // Add Ennemies --------------------------------------
  osg::ref_ptr<osgParticle::ParticleSystem> alienShootPS =
    CreateAlienShootPS(*root);
  Soleil::SceneManager::RegisterParticleSystem(3, alienShootPS);
  Soleil::SceneManager::AddParticleEmitter(
    3, Soleil::ShootEmitter::CreateAlienShootEmitter());
  osg::ref_ptr<Soleil::EntityGroup> ennemies = new Soleil::EntityGroup;
  osg::ref_ptr<osg::Node>           templateEnnemy =
    osgDB::readNodeFile("../media/ZincEnnemyOne.osgt");

  CreateAlienCraft =
    [&ennemies, &templateEnnemy, num = 0 ](const osg::Vec3& position)
  {
    osg::ref_ptr<Soleil::Actor> first = new Soleil::Actor(1);
    first->setPosition(position);
    // first->setMatrix(osg::Matrix::translate(30, 00, -30));
    first->addChild(templateEnnemy);

    osg::ref_ptr<Soleil::AlienCraft> ac = new Soleil::AlienCraft;
    first->addUpdateCallback(ac);
    first->setName(Soleil::toString("FirstEnnemy_N", num));
    ennemies->addChild(first);
  };
// First:
#if 1
  for (int i = 0; i < 0; ++i) {
    CreateAlienCraft(osg::Vec3(Random(-Range, Range), Random(-Range, Range),
                               Random(-Range, Range)));
  }
#else
  CreateAlienCraft(osg::Vec3(30, 0, 0));
  CreateAlienCraft(osg::Vec3(30, 2, 0));
  CreateAlienCraft(osg::Vec3(30, -2, 0));
  CreateAlienCraft(osg::Vec3(34, 0, 0));
#endif

  root->addChild(ennemies);

  viewer.realize();
  osg::ref_ptr<DebugTrackball> cam = new DebugTrackball;
  cam->root                        = root;
  viewer.getCamera()->setCullMask(1);
  viewer.getUpdateVisitor()->setTraversalMask(2);
  viewer.setCameraManipulator(cam);
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
