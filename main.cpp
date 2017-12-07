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

#include <osg/BlendEquation>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/TexGen>
#include <osg/Texture2D>
#include <osg/Texture3D>
#include <osg/io_utils>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osgDB/WriteFile>
#include <osgGA/CameraManipulator>
#include <osgGA/NodeTrackerManipulator>
#include <osgViewer/Viewer>

#include <osg/Timer>

#include "Logger.h"
#include "SkyBox.h"
#include "Utils.h"
#include "easing.h"
#include "stringutils.h"

// USE_OSGPLUGIN(ive)
// USE_OSGPLUGIN(osg)
// USE_OSGPLUGIN(osg2)
// USE_OSGPLUGIN(rgb)
// USE_OSGPLUGIN(OpenFlight)

// USE_OSGPLUGIN(osg)

// USE_DOTOSGWRAPPER_LIBRARY(osg)
// // USE_DOTOSGWRAPPER_LIBRARY(osgViewer)

// USE_SERIALIZER_WRAPPER_LIBRARY(osg)

// USE_GRAPHICSWINDOW()

osg::Vec3 worldPosition;
osg::ref_ptr<osg::MatrixTransform> GroupPtr;
osg::ref_ptr<osg::MatrixTransform> PlayerNode;
osg::ref_ptr<osg::MatrixTransform> PlaneNode;
osg::ref_ptr<osg::PositionAttitudeTransform> CameraBase;

// TODO: In utils file
/* --- Utils ---*/
template <typename T> int Sign(T val) { return (T(0) < val) - (val < T(0)); }
template <typename T, typename U> U mix(T x, T y, U a) {
  return x * (1.0 - a) + y * a;
}
/* --- Utils ---*/

/* --- PlayerFlightCameraManipulator */
class PlayerFlightCameraManipulator : public osgGA::CameraManipulator {
public:
  PlayerFlightCameraManipulator(osg::MatrixTransform *target);

  bool handle(const osgGA::GUIEventAdapter &event,
              osgGA::GUIActionAdapter &action) override;

public:
  void setByMatrix(const osg::Matrixd &matrix);
  void setByInverseMatrix(const osg::Matrixd &matrix);
  osg::Matrixd getMatrix() const;
  osg::Matrixd getInverseMatrix() const;

private:
  osg::MatrixTransform *target_;
};

PlayerFlightCameraManipulator::PlayerFlightCameraManipulator(
    osg::MatrixTransform *target)
    : target_(target) {
  osg::Vec3 eye, center, up;
  osg::Matrix camMat = osg::Matrix::inverse(target_->getMatrix());
  camMat.getLookAt(eye, center, up);

  center.y() = 0.0; // TODO: Issue, if center is too close to eye, origin is set
                    // to 0. Must be centered on the subject
  SOLEIL__LOGGER_DEBUG("E Y E        =  ", eye);
  SOLEIL__LOGGER_DEBUG("C E N T E R  =  ", center);
  SOLEIL__LOGGER_DEBUG("U P          =  ", up);

  setByMatrix(camMat);
}

void PlayerFlightCameraManipulator::setByMatrix(const osg::Matrixd &matrix) {
  target_->setMatrix(matrix);
}

void PlayerFlightCameraManipulator::setByInverseMatrix(
    const osg::Matrixd & /*matrix*/) {
  assert(false && "// TODO: ");
  // target_->setInitialBound()
}

osg::Matrixd PlayerFlightCameraManipulator::getMatrix() const {
  osg::Matrix cameraArcBall;
  CameraBase->getAttitude().get(cameraArcBall);

  return osg::Matrix::inverse(target_->getMatrix()) * cameraArcBall *
         PlayerNode->getMatrix();
}

osg::Matrixd PlayerFlightCameraManipulator::getInverseMatrix() const {
  return osg::Matrix::inverse(getMatrix());
}

bool PlayerFlightCameraManipulator::handle(
    const osgGA::GUIEventAdapter &event, osgGA::GUIActionAdapter & /*action*/) {
  const float x = event.getXnormalized();
  const float y = event.getYnormalized();

  ///////////////////////////////////////////////////
  // // TODO: Work in Progress			   //
  // 1. Make sure there is no static but members   //
  // 2. Smooth the Movement with quaternion slerp  //
  // 3. Smooth rotations with Animation&frame rate //
  // 4. Make the camera follow the plane rotation? //
  ///////////////////////////////////////////////////

  const float Min = 0.3f;
  const float speed = 0.0125f;

  /////////////////////////////////////////
  // Move the ROOT node (Camera + Plane) //
  /////////////////////////////////////////
  // TODO: Statics will be members
  static osg::ref_ptr<osg::PositionAttitudeTransform> pat =
      new osg::PositionAttitudeTransform;
  static float accumulatedPitch = 0.0f;

  float yaw = 0;
  float pitch = 0;

  /* Ease the movements for smoother experience */
  const float absoluteX = osg::absolute(x);
  if (absoluteX >= Min) {
    yaw = -Sign(x) * ExponentialEaseIn(absoluteX) * speed;
  }
  const float absoluteY = osg::absolute(y);
  if (absoluteY >= Min) {
    pitch = Sign(y) * ExponentialEaseIn(osg::absolute(y)) * speed;
  }

  /* Make sure to cap the pitch to avoid the plane to fligh on its top. This
   * forbid looping but should make the game easier */
  constexpr float MaxPitch = 1.256637061f; // 80% of PI/2
  if ((accumulatedPitch + pitch) > MaxPitch) {
    pitch = accumulatedPitch - MaxPitch;
  } else if ((accumulatedPitch + pitch) < -MaxPitch) {
    pitch = accumulatedPitch + MaxPitch;
  }
  accumulatedPitch += pitch;

  osg::Quat Yaw;
  Yaw.makeRotate(yaw, 0, 0, 1);
  osg::Quat Pitch;
  Pitch.makeRotate(pitch, 1, 0, 0);

  /* Order matter: This combination avoid the Roll when Pitching and Yawing at
   * the same time.*/
  osg::Quat tr = Pitch * pat->getAttitude() * Yaw;

  pat->setAttitude(tr);
  pat->setPosition(pat->getPosition() + (tr * osg::Vec3(0, 0.06f, 0)));

  osg::Matrix matrix;
  pat->computeLocalToWorldMatrix(matrix, nullptr);
  PlayerNode->setMatrix(matrix); // TODO: Use PAT for player node

  /////////////////////////////////////////////////
  // Orient the plane to the targetted direction //
  /////////////////////////////////////////////////

  osg::Matrix planeOrientation;
  float planePitch = 0.0f;
  if (osg::absolute(y) > Min || osg::absolute(x) > Min) {
    planePitch = (osg::absolute(y) > Min) ? y : 0.0f;
    const float planeRoll = atan2(x, osg::absolute(y));

#if 1
    // Plane movement detached from the main group
    planePitch = (osg::absolute(y) > Min)
                     ? Sign(y) * ExponentialEaseIn(osg::absolute(y))
                     : 0.0f;
#else
    planePitch = pitch;
#endif
    
    planeOrientation = osg::Matrix::rotate(planeRoll, 0, 1, 0) *
                       osg::Matrix::rotate(planePitch, 1, 0, 0);
  }
  PlaneNode->setMatrix(planeOrientation);
#if 1
  {
    static osg::Quat arcBall;
    osg::Quat arcBallFinal;
    arcBallFinal.makeRotate(planePitch, 1, 0, 0);

    arcBall.slerp(0.025f, arcBall, arcBallFinal); // TODO: Configurable lerp
    // TODO: Frame rate fixed lerp

    CameraBase->setAttitude(arcBall);
  }
#endif

  return true;
}

/* --- PlayerFlightCameraManipulator --- */

/* --- Create texture 3D from a list of texture 2D --- */

osg::ref_ptr<osg::StateSet>
convertImageListTo3Dstate(std::vector<std::string> files) {
  assert(files.size() > 0);

  std::vector<osg::ref_ptr<osg::Image>> imageList;

  osg::ref_ptr<osg::Image> StandardImage;
  for (const auto &file : files) {
    osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile(file);
    assert(image);

    if (StandardImage) {
      // Make  sure all the images are the same
      assert(StandardImage->getPixelFormat() == image->getPixelFormat());
      assert(StandardImage->s() == image->s());
      assert(StandardImage->t() == image->t());
    } else {
      // The first one will be the rule
      StandardImage = image;
    }

    imageList.push_back(image);
  }

  std::cout << "--->" << imageList.size() << "\n";

  // then allocated a 3d image to use for texturing.
  osg::ref_ptr<osg::Image> image_3d = new osg::Image;
  image_3d->allocateImage(StandardImage->s(), StandardImage->t(),
                          imageList.size(), StandardImage->getPixelFormat(),
                          StandardImage->getDataType());

  for (unsigned int i = 0; i < imageList.size(); ++i) {
    // copy the 2d images into the 3d image.
    image_3d->copySubImage(0, 0, i, imageList[i]);
  }

  image_3d->setInternalTextureFormat(StandardImage->getInternalTextureFormat());

  // set up the 3d texture itself,
  // note, well set the filtering up so that mip mapping is disabled,
  // gluBuild3DMipsmaps doesn't do a very good job of handled the
  // imbalanced dimensions of the 256x256x4 texture.
  osg::ref_ptr<osg::Texture3D> texture3D = new osg::Texture3D;
#if 1
  texture3D->setFilter(osg::Texture3D::MIN_FILTER, osg::Texture3D::LINEAR);
  texture3D->setFilter(osg::Texture3D::MAG_FILTER, osg::Texture3D::LINEAR);
#endif
  // texture3D->setWrap(osg::Texture3D::WRAP_R, osg::Texture3D::REPEAT);
  texture3D->setImage(image_3d);

  // create a texgen to generate a R texture coordinate, the geometry
  // itself will supply the S & T texture coordinates.
  // in the animateStateSet callback well alter this R value to
  // move the texture through the 3d texture, 3d texture filtering
  // will do the blending for us.
  osg::ref_ptr<osg::TexGen> texgen = new osg::TexGen;
  texgen->setMode(osg::TexGen::OBJECT_LINEAR);
  texgen->setPlane(osg::TexGen::R, osg::Plane(0.0f, 0.0f, 0.0f, 0.0f));

  // create the StateSet to store the texture data
  osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet;
  stateset->setTextureMode(0, GL_TEXTURE_GEN_R, osg::StateAttribute::ON);
  stateset->setTextureAttribute(0, texgen);
  stateset->setTextureAttributeAndModes(0, texture3D, osg::StateAttribute::ON);
  stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

  osg::ref_ptr<osg::StateAttribute> blend =
      new osg::BlendFunc(osg::BlendFunc::BlendFuncMode::SRC_ALPHA,
                         osg::BlendFunc::BlendFuncMode::ONE_MINUS_SRC_ALPHA);
  stateset->setAttributeAndModes(blend, osg::StateAttribute::ON);

  // stateset->setAttribute(
  //   new osg::BlendEquation(osg::BlendEquation::Equation::FUNC_ADD));

  return stateset;
}

class UpdateStateCallback : public osg::NodeCallback {
public:
  UpdateStateCallback(osg::StateSet *stateset) : stateset(stateset) {}

  virtual void operator()(osg::Node *node, osg::NodeVisitor *nv) {

    osg::StateAttribute *attribute =
        stateset->getTextureAttribute(0, osg::StateAttribute::TEXGEN);
    osg::TexGen *texgen = dynamic_cast<osg::TexGen *>(attribute);
    // if (texgen) {
    texgen->getPlane(osg::TexGen::R)[3] += 1.00f / 17.0f; // TODO: Framerate
    //}

    // note, callback is responsible for scenegraph traversal so
    // should always include call the traverse(node,nv) to ensure
    // that the rest of callbacks and the scene graph are traversed.
    traverse(node, nv);
  }

private:
  osg::StateSet *stateset;
};

/* --- Create texture 3D from a list of texture 2D --- */

/* --- Explosion Impostor --- */

osg::ref_ptr<osg::Geometry> createExplosionQuad() {
  osg::ref_ptr<osg::Geometry> quad = osg::createTexturedQuadGeometry(
      osg::Vec3(-0.5f, 0.0f, -0.5f), osg::Vec3(1.0f, 0.0f, 0.0f),
      osg::Vec3(0.0f, 0.0f, 1.0f));

#if 0
  osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
  osg::ref_ptr<osg::Image>     image =
    osgDB::readImageFile("../media/textures/0020.png");
  assert(image);
  texture->setImage(image);
  osg::StateSet* ss = quad->getOrCreateStateSet();
  ss->setTextureAttributeAndModes(0, texture);
#else

  std::vector<std::string> files;
  for (int i = 20; i < 71; i += 3) {
    const std::string s = "../media/textures/00" + std::to_string(i) + ".png";
    std::cout << s << "\n";
    files.push_back(s);
  }

  osg::ref_ptr<osg::StateSet> texture3d = convertImageListTo3Dstate(files);
  quad->setStateSet(texture3d);
#endif

  return quad;
}

/* --- Explosion Impostor --- */

static osg::ref_ptr<osg::MatrixTransform> createPlayerGraph() {
  osg::ref_ptr<osg::MatrixTransform> group = new osg::MatrixTransform;
  osg::ref_ptr<osg::Node> cessna = osgDB::readNodeFile("../media/Player.osgt");
  osg::ref_ptr<osg::Node> axes = osgDB::readNodeFile("../media/axes.osgt");
  assert(cessna.get() != nullptr && "Root is null");
  group->addChild(cessna);
  // group->addChild(axes);

  // TODO: FIXME the osgexport do not export camera node as a node alone
  osg::MatrixTransform *camNode = dynamic_cast<osg::MatrixTransform *>(
      Soleil::GetNodeByName(*cessna, "Camera"));
  assert(camNode); // TODO: Better error system

  CameraBase = new osg::PositionAttitudeTransform;
  osg::ref_ptr<osg::MatrixTransform> newCamNode = new osg::MatrixTransform;
  newCamNode->setMatrix(camNode->getMatrix());
  newCamNode->setName("NewCamera");
  // group->addChild(newCamNode);
  CameraBase->addChild(newCamNode);
  group->addChild(CameraBase);

  return group;
}

#if 0
static osg::ref_ptr<osg::MatrixTransform> createAxisPath(int size) {
  assert(size >= 0);

  osg::ref_ptr<osg::MatrixTransform> root = new osg::MatrixTransform;
  osg::ref_ptr<osg::Node> child = osgDB::readNodeFile("../media/axes.osgt");
  for (int i = 0; i < size; ++i) {
    osg::ref_ptr<osg::MatrixTransform> target = new osg::MatrixTransform;
    target->addChild(child);
    target->setMatrix(osg::Matrix::translate(osg::Vec3(
                          0.0f, static_cast<float>(i) * 5.0f - 20.0f, 0.0f)) *
                      osg::Matrix::scale(osg::Vec3(10.0f, 10.0f, 10.0f)));

    root->addChild(target);
  }
  return root;
}
#endif

int main(int // argc
         ,
         char *const // argv
             []) {

  osg::ref_ptr<osg::Group> root = new osg::Group();

  auto plane = createPlayerGraph();
  root->addChild(plane);

  ///////////////////////
  // Setting the Scene //
  ///////////////////////

  osg::ref_ptr<osg::Node> skycube =
      osgDB::readNodeFile("../media/ZincSkybox.osgt");
  assert(skycube);
  osg::ref_ptr<SkyBox> skybox = new SkyBox;
  skybox->addChild(skycube);
  root->addChild(skybox);

// root->addChild(createAxisPath(10));

#if 0
  root->addChild(osgDB::readNodeFile("../media/ZincIslands.obj"));
#elif 1
  root->addChild(osgDB::readNodeFile("../media/islands.3ds.osgt"));
#elif 0
  root->addChild(osgDB::readNodeFile("../media/islands.osgt"));

  osg::ref_ptr<osg::Node> testNode =
      osgDB::readNodeFile("../media/ZincTest.obj");
  assert(testNode);
  osg::ref_ptr<osg::MatrixTransform> testRoot = new osg::MatrixTransform;
  testRoot->addChild(testNode);

  root->addChild(testRoot);
  osgDB::writeNodeFile(*testRoot, "../media/ZincTest.osgt");
#endif

  /////////////////////////////
  // ----------------------- //
  /////////////////////////////

  osg::ref_ptr<osg::Billboard> explosion = new osg::Billboard;
  explosion->setMode(osg::Billboard::POINT_ROT_EYE);
  osg::StateSet *ss = explosion->getOrCreateStateSet();
  ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
  // osg::ref_ptr<osg::StateAttribute> blend =
  //   new osg::BlendFunc(osg::BlendFunc::BlendFuncMode::SRC_ALPHA,
  //                      osg::BlendFunc::BlendFuncMode::ONE_MINUS_SRC_ALPHA);
  // ss->setAttributeAndModes(blend, osg::StateAttribute::ON);

  osg::ref_ptr<osg::Geometry> ex = createExplosionQuad();
  explosion->addDrawable(ex, osg::Vec3(0.0, 1.0f, 1.0f));
  explosion->setUpdateCallback(new UpdateStateCallback(ex->getStateSet()));
  root->addChild(explosion);

  osgViewer::Viewer viewer;
  viewer.setSceneData(root);

  ///////////////////////////////////////////
  // Setting the player camera manipulator //
  ///////////////////////////////////////////
  PlayerNode = plane;
  osg::ref_ptr<osg::MatrixTransform> centerg = new osg::MatrixTransform;
  centerg->addChild(osgDB::readNodeFile("../media/axes.osgt"));
  centerg->setMatrix(
      osg::Matrix::translate(PlayerNode->computeBound().center()));
  PlayerNode->addChild(centerg);

  PlaneNode = dynamic_cast<osg::MatrixTransform *>(
      Soleil::GetNodeByName(*plane.get(), "Plane"));
  assert(PlaneNode);
  osg::MatrixTransform *camNode = dynamic_cast<osg::MatrixTransform *>(
      Soleil::GetNodeByName(*plane.get(), "NewCamera"));
  assert(camNode); // TODO: Better error system

  osg::ref_ptr<PlayerFlightCameraManipulator> playerManipulator =
      new PlayerFlightCameraManipulator(camNode);

  viewer.setCameraManipulator(playerManipulator);

  /////////////////////////////////////////////////
  // ------------------------------------------- //
  /////////////////////////////////////////////////

  viewer.realize();

  return viewer.run();
}
