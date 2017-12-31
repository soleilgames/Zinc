
#include <cassert>

#include <osg/MatrixTransform>
#include <osg/Node>

#include <iostream> // TODO: temp

#include "ShootTracer.h"

namespace Soleil {

ShootTracer::ShootTracer(int numberOfPoints, float width,
                         const osg::Vec3 &color)
    : numberOfPoints(numberOfPoints), halfWidth(width / 2.0f), color(color),
      vertices(new osg::Vec3Array(osg::Array::Binding::BIND_PER_VERTEX,
                                  numberOfPoints)),
      normals(new osg::Vec3Array(osg::Array::Binding::BIND_PER_VERTEX,
                                 numberOfPoints)),
      colors(new osg::Vec4Array(osg::Array::Binding::BIND_PER_VERTEX,
                                numberOfPoints)) {
  const osg::Vec3 origin(halfWidth, 0.0f, 0.0f);
  const osg::Vec3 normal(0.0f, 0.0f, 1.0f);

  assert(numberOfPoints > 0 && "Cannot have less than one point");

  for (int i = 0; i < numberOfPoints - 1; i += 2) {
    (*vertices)[i] = origin;
    (*vertices)[i + 1] = -origin;

    (*normals)[i] = normal;
    (*normals)[i + 1] = normal;

    // TODO: Use easing function is necessary:
    const float alpha =
        1.0f - (numberOfPoints - i) / static_cast<float>(numberOfPoints);
    (*colors)[i] = osg::Vec4(color, alpha);
    (*colors)[i + 1] = osg::Vec4(color, alpha);
  }

  // TODO: use double-buffering
  setDataVariance(osg::Object::DataVariance::DYNAMIC);
  setUseDisplayList(false);
  setUseVertexBufferObjects(true);
  setVertexArray(vertices);
  setNormalArray(normals);
  setColorArray(colors);
  addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, numberOfPoints));
}

ShootTracer::~ShootTracer() {}

void ShootTracer::updateHead(const osg::Vec3 &position,
                             const osg::Vec3 &direction) {

  const osg::Vec3 min = position + osg::Vec3(halfWidth, 0.0f, 0.0f);
  const osg::Vec3 max = position - osg::Vec3(halfWidth, 0.0f, 0.0f);
  for (int i = 0; i < numberOfPoints - 1; i += 2) {
    (*vertices)[i] = min;
    (*vertices)[i + 1] = max;
  }
  // (*vertices)[numberOfPoints - 2] = position + osg::Vec3(halfWidth, 0.0f, 0.0f);
  // (*vertices)[numberOfPoints - 1] = position - osg::Vec3(halfWidth, 0.0f, 0.0f);
  this->direction = direction;
  this->direction.normalize();
}

void ShootTracer::updateTail() {
  for (int i = 0; i < numberOfPoints - 3; i += 2) {
    (*vertices)[i] = (*vertices)[i + 2];
    (*vertices)[i + 1] = (*vertices)[i + 3];
    (*normals)[i] = (*normals)[i + 2];
    (*normals)[i + 1] = (*normals)[i + 3];
  }
  // TODO: Conf speed
  // TODO: Frame rate
  constexpr float speed = 3.6f;
  (*vertices)[numberOfPoints - 2] += direction * speed;
  (*vertices)[numberOfPoints - 1] += direction * speed;

  vertices->dirty();

  osg::Vec3 normal = osg::Vec3(0.0f, 0.0f, 1.0f);
  normal.normalize();
  (*normals)[numberOfPoints - 2] = normal;
  (*normals)[numberOfPoints - 1] = normal;
  normals->dirty();

  dirtyBound();
}

ShootTracerCallback::ShootTracerCallback() {}

void ShootTracerCallback::operator()(osg::Node *node,
                                     osg::NodeVisitor *visitor) {
  // TODO: So not update if not fired
  // TODO: Cull if not fired
  for (std::size_t i = 0; i < tracers.size(); ++i) {
    osg::ref_ptr<Soleil::ShootTracer> shootTracer;
    tracers[i].lock(shootTracer);
    if (shootTracer == nullptr)
      continue;

    shootTracer->updateTail();
  }

  traverse(node, visitor);
}

} // Soleil
