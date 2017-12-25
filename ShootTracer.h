
#ifndef SOLEIL__SHOOTTRACER_H_
#define SOLEIL__SHOOTTRACER_H_

#include <osg/Geometry>
#include <osg/Vec3>

namespace Soleil {

  class ShootTracer : public osg::Geometry
  {
  public:
    ShootTracer(int numberOfPoints, float width, const osg::Vec3& color);

  protected:
    ~ShootTracer();

  public:
    void updateTail(const osg::Matrix& modelMatrix);

  private:
    int       numberOfPoints;
    float     halfWidth;
    osg::Vec3 color;

  private:
    osg::ref_ptr<osg::Vec3Array> vertices;
    osg::ref_ptr<osg::Vec3Array> normals;
    osg::ref_ptr<osg::Vec4Array> colors;
    // TODO: Use the one in geometry
  };

  class ShootTracerCallback : public osg::NodeCallback
  {
  public:
    ShootTracerCallback(ShootTracer* shootTracer);
    void operator()(osg::Node* node, osg::NodeVisitor* visitor);

  private:
    osg::observer_ptr<ShootTracer> shootTracer;
  };

} // Soleil

#endif /* SOLEIL__SHOOTTRACER_H_ */
