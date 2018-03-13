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

#ifndef SOLEIL__UTILS_H_
#define SOLEIL__UTILS_H_

#include <osg/Camera>   // createRTTCamera
#include <osg/Geode>    // createRTTCamera
#include <osg/Geometry> // createRTTCamera
#include <osg/Group>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/PolygonMode> // createRTTCamera
#include <osg/Texture2D>   // createRTTCamera
#include <random>
#include <string>

#include <iostream>

#include "Logger.h"

namespace Soleil {

  struct NameVisitor : public osg::NodeVisitor
  {
    std::string   name;
    osg::Node*    found = nullptr;
    osg::NodePath path;

    NameVisitor(osg::NodeVisitor::TraversalMode mode)
      : osg::NodeVisitor(mode)
    {
    }

    void apply(osg::Node& node) override
    {
      // SOLEIL__LOGGER_DEBUG("Searching for '", name,
      //                      "' in node: ", node.getName());
      if (node.getName() == name) {
        path  = this->getNodePath();
        found = &node;
      } else {
        traverse(node);
      }
    }
  };

  inline osg::Node* GetNodeByName(osg::Node& root, const std::string& name)
  {
    if (root.getName() == name) return &root;
    NameVisitor v(osg::NodeVisitor::TraversalMode::TRAVERSE_ALL_CHILDREN);
    v.name = name;

    root.accept(v);
    return v.found;
  }

  inline osg::NodePath GetPathByNodeName(osg::Node&         root,
                                         const std::string& name,
                                         osg::NodePath*     path = nullptr)
  {
    NameVisitor v(osg::NodeVisitor::TraversalMode::TRAVERSE_ALL_CHILDREN);
    v.name = name;

    root.accept(v);
    return v.path;
  }

  inline std::string toName(const osg::Node& node)
  {
    std::string name =
      node.libraryName() + std::string("::") + node.className();
    if (not node.getName().empty()) {
      name += ": " + node.getName();
    }
    return name;
  }

  inline osg::ref_ptr<osg::Camera> createHUDCamera(double left, double right,
                                                   double bottom, double top)
  {
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);
    camera->setRenderOrder(osg::Camera::POST_RENDER);
    camera->setAllowEventFocus(false);
    camera->setProjectionMatrix(osg::Matrix::ortho2D(left, right, bottom, top));
    camera->getOrCreateStateSet()->setMode(GL_LIGHTING,
                                           osg::StateAttribute::OFF);
    return camera;
  }

  inline osg::ref_ptr<osg::Geode> createScreenQuad(
    float width, float height, const osg::Vec3& corner = osg::Vec3(),
    float scale = 1.0f)
  {
    osg::ref_ptr<osg::Geometry> geom = osg::createTexturedQuadGeometry(
      corner, osg::Vec3(width, 0.0f, 0.0f), osg::Vec3(0.0f, height, 0.0f), 0.0f,
      0.0f, 1.0f * scale, 1.0f * scale);
    osg::ref_ptr<osg::Geode> quad = new osg::Geode;
    quad->addDrawable(geom);
    const int values =
      osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED;
    quad->getOrCreateStateSet()->setAttribute(
      new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK,
                           osg::PolygonMode::FILL),
      values);
    quad->getOrCreateStateSet()->setMode(GL_LIGHTING, values);
    return quad;
  }

  inline osg::ref_ptr<osg::Camera> createRTTCamera(
    osg::Camera::BufferComponent buffer, osg::Texture* tex,
    bool isAbsolute = false)
  {
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setClearColor(osg::Vec4());
    camera->setClearMask(GL_COLOR_BUFFER_BIT |
                         GL_DEPTH_BUFFER_BIT); // TODO: Don't clear everything
    camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    camera->setRenderOrder(osg::Camera::PRE_RENDER);
    if (tex) {
      tex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
      tex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
      camera->setViewport(0, 0, tex->getTextureWidth(),
                          tex->getTextureHeight());
      camera->attach(buffer, tex);
    }
    if (isAbsolute) {
      camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
      camera->setProjectionMatrix(osg::Matrix::ortho2D(0.0, 1.0, 0.0, 1.0));
      camera->setViewMatrix(osg::Matrix::identity());
      camera->addChild(createScreenQuad(1.0f, 1.0f));
    }
    return camera;
  }

} // Soleil

template <typename T>
int
Sign(T val)
{
  return (T(0) < val) - (val < T(0));
}

template <typename T, typename U>
T
mix(T x, T y, U a)
{
  return x * (1.0 - a) + y * a;
}

template <typename T>
T
reflect(T incidence, T normal)
{
  return normal * (-2.0f * (incidence * normal)) + incidence;
}

template <typename T>
T
Random(T maxValue)
{
  return static_cast<T>(rand()) / (static_cast<T>(RAND_MAX / maxValue));
}
template <typename T>
T
Random(T minValue, T maxValue)
{
  std::random_device rd;        // TODO: Use same device everywhere
  std::mt19937       gen(rd()); // TODO: Allow pure true randomness
  return mix(minValue, maxValue, std::generate_canonical<double, 1>(gen));
}

inline osg::Vec3
VectorFront()
{
  return osg::Vec3(0, 1, 0);
}

inline osg::Vec3
VectorUp()
{
  return osg::Vec3(0, 0, 1);
}

inline osg::Vec3
VectorRight()
{
  return osg::Vec3(1, 0, 0);
}

inline osg::Vec3
normalize(const osg::Vec3& node)
{
  osg::Vec3 normalized = node;
  normalized.normalize();
  return normalized;
}

inline osg::Vec3
limit(const osg::Vec3& vec, const float maxSize)
{
  if (vec.length() <= maxSize) return vec;
  // return vec * (1 - maxSize / vec.length());
  return normalize(vec) * maxSize;
}

/**
 * Return a number close to 1.0f if A face B, less if not facing. A negative
 * number if A face away B
 */
inline float
facing(const osg::Vec3& A, const osg::Vec3& directionOfA, const osg::Vec3& B)
{
  const osg::Vec3 aToB   = normalize(B - A);
  const float     facing = aToB * normalize(directionOfA);
  return facing;
}

inline osg::Quat
quatLookAt(const osg::Vec3& center)
{
  const float dot = VectorFront() * center;
  if (osg::absolute(dot) - (-1.0f) < 0.000001f) {
    // Center is in our back we need to returns a full 180 degrees rotation
    return osg::Quat(osg::PIf, VectorUp());
  }
  if (osg::absolute(dot - (1.0f)) < 0.000001f) {
    // Center is front, no need to rotate
    return osg::Quat();
  }
  const float     angle = std::acos(dot);
  const osg::Vec3 axis  = normalize(VectorFront() ^ center);
  return osg::Quat(angle, axis);

  // Paramerters:
  // const osg::Vec3& position,
  // const osg::Vec3& up
  //
  // const float dot = position * center;
  // if (osg::absolute(dot) - (-1.0f) < 0.000001f) {
  //   SOLEIL__LOGGER_DEBUG("BACK");
  //   // Center is in our back we need to returns a full 180 degrees rotation
  //   return osg::Quat(osg::PIf, up);
  // }
  // if (osg::absolute(dot) - (1.0f) < 0.000001f) {
  //   SOLEIL__LOGGER_DEBUG("FRONT");
  //   // Center is front, no need to rotate
  //   return osg::Quat();
  // }

  // const float rotationAngle = std::acos(dot);
  // // const float     rotationAngle = dot;
  // const osg::Vec3 rotationAxis = normalize(osg::Vec3(0, 1, 0) ^ center);
  // osg::Quat       t;
  // t.makeRotate(rotationAngle, rotationAxis);
  // return t;
  // // return osg::Quat(rotationAngle, rotationAxis);
}

inline osg::Quat
quatLookAt(const osg::Vec3& dir, const osg::Vec3& up)
{
  if (dir.length2() == 0.0f) {
    assert(false && "Zero length direction in quatLookAt(const osg::Vec3& "
                    "dir, const osg::Vec3& up)");
    return osg::Quat();
  }

  if (up != dir) {
    const osg::Vec3 v = dir + up * -(up * dir);
    osg::Quat       q;
    q.makeRotate(VectorFront(), v);
    osg::Quat r;
    r.makeRotate(v, dir);
    return q * r; // * q;
  }

  osg::Quat q;
  q.makeRotate(VectorFront(), dir);
  return q;
}

/**
 * While X is in range [A, B], returns its corresponding value in range [C, D]
 */
inline float
map(const float A, const float B, const float C, const float D, const float X)
{
  return (X - A) / (B - A) * (D - C) + C;
}

#endif /* SOLEIL__UTILS_H_ */
