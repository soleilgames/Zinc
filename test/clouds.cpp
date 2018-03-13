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
#include "CloudBlock"
#include "EntityGroup.h"
#include "EventManager.h"
#include "ParticleObjects.h"
#include "SceneManager.h"
#include "Utils.h"
#include <functional>
#include <osg/BlendColor>
#include <osg/BlendEquation>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osgParticle/ModularProgram>
#include <osgParticle/ParticleSystemUpdater>
#include <osgViewer/Viewer>

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
  osg::ref_ptr<osg::Group> root = new osg::Group;
  viewer.setSceneData(root);

  Soleil::EventManager::Init();
  Soleil::SceneManager::Init(root);

  // -----------------------------
  // osg::Image* makeGlow( int width, int height, float expose, float sizeDisc )
  auto makeGlow = [](int width, int height, float expose, float sizeDisc) {
    osg::ref_ptr<osg::Image> image = new osg::Image;
    image->allocateImage(width, height, 1, GL_LUMINANCE_ALPHA,
                         GL_UNSIGNED_BYTE);

    unsigned char* data = image->data();
    for (int y = 0; y < height; ++y) {
      float dy = (y + 0.5f) / height - 0.5f;
      for (int x = 0; x < width; ++x) {
        float dx   = (x + 0.5f) / width - 0.5f;
        float dist = sqrtf(dx * dx + dy * dy);
        float intensity =
          2 - osg::minimum(
                2.0f, powf(2.0f, osg::maximum(dist - sizeDisc, 0.0f) * expose));
        float noise = 1.0f;

        unsigned char color =
          (unsigned char)(noise * intensity * 255.0f + 0.5f);
        *(data++) = color;
        *(data++) = color;
      }
    }
    return image.release();
  };

  // void readCloudCells( CloudBlock::CloudCells& cells, const std::string& file
  // )
  auto readCloudCells = [](CloudBlock::CloudCells& cells,
                           const std::string&      file) {
    std::ifstream is(file.c_str());
    assert(is);
    if (!is) return;

    double       x, y, z;
    unsigned int density, brightness;
    while (!is.eof()) {
      is >> x >> y >> z >> density >> brightness;

      CloudBlock::CloudCell cell;
      cell._pos.set(x, y, z);
      cell._density    = (float)density;
      cell._brightness = (float)brightness;
      cells.push_back(cell);
    }
  };

  CloudBlock::CloudCells cells;
  readCloudCells(cells, "../media/data.txt");

  osg::ref_ptr<CloudBlock> clouds = new CloudBlock;
  clouds->setCloudCells(cells);

  {
    osg::StateSet* ss = clouds->getOrCreateStateSet();
    ss->setAttributeAndModes(
      new osg::BlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA));
    ss->setAttributeAndModes(new osg::Depth(osg::Depth::LESS, 0.0, 1.0, false));
    ss->setTextureAttributeAndModes(
      0, new osg::Texture2D(makeGlow(32, 32, 2.0f, 0.0f)));
  }
  osg::ref_ptr<osg::Geode> geode = new osg::Geode;
  geode->addDrawable(clouds.get());
  root->addChild(geode);
  // ------------------------

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

  viewer.realize();
  osg::ref_ptr<DebugTrackball> cam = new DebugTrackball;
  cam->root                        = root;
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
