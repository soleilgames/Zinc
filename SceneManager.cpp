
#include <limits>
#include <memory>

#include "SceneManager.h"

#include "Logger.h"

#include "Utils.h"
#include <functional>
#include <osg/ComputeBoundsVisitor>
#include <osg/Drawable>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/RayIntersector>

namespace Soleil {

  static std::unique_ptr<SceneManager> sceneManager;

  void SceneManager::Init(osg::ref_ptr<osg::Group> sceneRoot)
  {
    sceneManager            = std::make_unique<SceneManager>();
    sceneManager->sceneRoot = sceneRoot;
  }

  // bool SceneManager::SegmentCollision(const osg::Vec3& start,
  //                                     const osg::Vec3& end, osg::Vec3*
  //                                     normal,
  //                                     float* distanceToObject)
  // {
  //   assert(sceneManager && "Call SceneManager::Init method");

  //   osg::ref_ptr<osgUtil::LineSegmentIntersector> lineSegment =
  //     new osgUtil::LineSegmentIntersector(
  //       osgUtil::Intersector::CoordinateFrame::MODEL, start, end, nullptr,
  //       osgUtil::Intersector::IntersectionLimit::LIMIT_NEAREST); // NO_LIMIT
  //   // LIMIT_NEAREST
  //   osgUtil::IntersectionVisitor visitor(lineSegment);
  //   visitor.setTraversalMask(Soleil::SceneManager::Mask::Collision);

  //   sceneManager->sceneRoot->accept(visitor);

  //   // TODO: Do polytope fetch. Collision is taken from the center of mass
  //   wich
  //   // allows wings to avoid colliding

  //   if (lineSegment->containsIntersections()) {
  //     float nearestDistance = std::numeric_limits<float>::max();
  //     SOLEIL__LOGGER_DEBUG("----");
  //     for (const auto& intersection : lineSegment->getIntersections()) {
  //       const float distance =
  //         (start - intersection.getWorldIntersectPoint()).length();

  //       if (nearestDistance > distance) {
  //         nearestDistance = distance;

  //         if (normal) {
  //           *normal = intersection.getWorldIntersectNormal();
  //         }
  //       }

  //       SOLEIL__LOGGER_DEBUG(
  //         "COLLISION between collider and ",
  //         intersection.nodePath.back()->getName(),
  //         ". Normal: ", intersection.getWorldIntersectNormal(),
  //         ". Distance=", distance);
  //     }

  //     if (distanceToObject) {
  //       *distanceToObject = nearestDistance;
  //     }
  //     if (normal) {
  //       normal->normalize();
  //     }

  //     return true;
  //   }
  //   return false;
  // }

  class IntersectionVisitor : public osgUtil::IntersectionVisitor
  {
  public:
    IntersectionVisitor(
      osg::Node* collider, osgUtil::Intersector* intersector = nullptr,
      osgUtil::IntersectionVisitor::ReadCallback* readCallback = nullptr);

    void apply(osg::Node& node) override;
    void apply(osg::Group& node) override;
    void apply(osg::Transform& node) override;

    osg::Node* collider;

    ~IntersectionVisitor() {}
  };

  IntersectionVisitor::IntersectionVisitor(
    osg::Node* collider, osgUtil::Intersector* intersector,
    osgUtil::IntersectionVisitor::ReadCallback* readCallback)
    : osgUtil::IntersectionVisitor(intersector, readCallback)
    , collider(collider)
  {
  }

  void IntersectionVisitor::apply(osg::Node& node)
  {
    if (&node == collider) {
      // SOLEIL__LOGGER_DEBUG("Avoiding self");
      return;
    }
    // SOLEIL__LOGGER_DEBUG("Descending on ", toName(node));

    osgUtil::IntersectionVisitor::apply(node);
  }

  void IntersectionVisitor::apply(osg::Group& node)
  {
    if (&node == collider) {
      // SOLEIL__LOGGER_DEBUG("Avoiding self");
      return;
    }
    // SOLEIL__LOGGER_DEBUG("Descending on ", toName(node));

    osgUtil::IntersectionVisitor::apply(node);
  }

  void IntersectionVisitor::apply(osg::Transform& node)
  {
    if (&node == collider) {
      // SOLEIL__LOGGER_DEBUG("Avoiding self");
      return;
    }
    // SOLEIL__LOGGER_DEBUG("Descending on ", toName(node));

    osgUtil::IntersectionVisitor::apply(node);
  }

  bool SceneManager::SegmentCollision(const osg::Vec3& start,
                                      const osg::Vec3& end, osg::Node* collider,
                                      osg::Vec3*          normal,
                                      float*              distanceToObject,
                                      osg::Node::NodeMask mask,
                                      osg::NodePath*      path)
  {
    assert(sceneManager && "Call SceneManager::Init method");

    osg::ref_ptr<osgUtil::LineSegmentIntersector> lineSegment =
      new osgUtil::LineSegmentIntersector(
        osgUtil::Intersector::CoordinateFrame::MODEL, start, end, nullptr,
        osgUtil::Intersector::IntersectionLimit::LIMIT_NEAREST); // NO_LIMIT
    // LIMIT_NEAREST
    IntersectionVisitor visitor(collider, lineSegment);
    visitor.setTraversalMask(mask);

    sceneManager->sceneRoot->accept(visitor);

    // TODO: Do polytope fetch. Collision is taken from the center of mass wich
    // allows wings to avoid colliding

    if (lineSegment->containsIntersections()) {
      float nearestDistance = std::numeric_limits<float>::max();
      SOLEIL__LOGGER_DEBUG("----");
      for (const auto& intersection : lineSegment->getIntersections()) {
        const float distance =
          (start - intersection.getWorldIntersectPoint()).length();

        if (nearestDistance > distance) {
          nearestDistance = distance;
          if (path) {
            *path = intersection.nodePath; // TODO: Avoid multi-copies
          }

          if (normal) {
            *normal = intersection.getWorldIntersectNormal();
          }
        }

        SOLEIL__LOGGER_DEBUG(
          "COLLISION between collider and ",
          toName(*intersection.nodePath.back()),
          ". Normal: ", intersection.getWorldIntersectNormal(),
          ". Distance=", distance);
      }

      if (distanceToObject) {
        *distanceToObject = nearestDistance;
      }
      if (normal) {
        normal->normalize();
      }

      return true;
    }
    return false;
  }

  osg::NodePath SceneManager::GetNodePath(ObjectID id)
  {
    assert(sceneManager && "Call SceneManager::Init method");

    assert(sceneManager->nodesPath.find(id) != sceneManager->nodesPath.end() &&
           "Node not found");
    return sceneManager->nodesPath[id];
  }

  void SceneManager::RegisterNodePath(ObjectID id, osg::NodePath nodePath)
  {
    assert(sceneManager && "Call SceneManager::Init method");

    sceneManager->nodesPath[id] = nodePath;
  }

  void SceneManager::RegisterParticleSystem(
    ObjectID id, osg::ref_ptr<osgParticle::ParticleSystem> system)
  {
    assert(sceneManager && "Call SceneManager::Init method");

    assert(sceneManager->particleSystems.find(id) ==
             sceneManager->particleSystems.end() &&
           "Update not supported yet on Particle systems");

    sceneManager->particleSystems.emplace(id, system);
    sceneManager->sceneRoot->addChild(system);
    // TODO: Only one ParticleSystemUpdater
    // TODO: A specific group for particle systems
  }

  void SceneManager::AddParticleEmitter(
    ObjectID particleSystemID, osg::ref_ptr<osgParticle::Emitter> emitter)
  {
    assert(sceneManager && "Call SceneManager::Init method");

    osg::ref_ptr<osgParticle::ParticleSystem> ps =
      sceneManager->particleSystems[particleSystemID];
    assert(ps && "No particle system found with this name");

    emitter->setParticleSystem(ps);
    sceneManager->sceneRoot->addChild(emitter);

    // TODO: A specific group for emitter
    // TODO: Remove all emitters that are finished (have a
    // SceneManager::frameUpdate method)
  }

  // TODO: Idea -> Each frame compute all the bounding box in specific position

} // Soleil
