
#ifndef SOLEIL__SCENEMANAGER_H_
#define SOLEIL__SCENEMANAGER_H_

#include <map>
#include <osg/BoundingBox>
#include <osg/Group>
#include <osgParticle/Emitter>
#include <osgParticle/ParticleSystem>
#include <vector>

namespace Soleil {

  typedef std::size_t ObjectID;

  class SceneManager
  {
  public:
    enum Mask
    {
      Render    = 1 << 0,
      Collision = 1 << 1,
      Shootable = 1 << 2,
    };

  public:
    static void Init(osg::ref_ptr<osg::Group> sceneRoot);

  public: // Collisions and test
    // static bool SegmentCollision(const osg::Vec3& start, const osg::Vec3&
    // end,
    //                              osg::Vec3* normal           = nullptr,
    //                              float*     distanceToObject = nullptr);
    static bool SegmentCollision(const osg::Vec3& start, const osg::Vec3& end,
                                 osg::Node*          collider,
                                 osg::Vec3*          normal           = nullptr,
                                 float*              distanceToObject = nullptr,
                                 osg::Node::NodeMask mask = Collision,
                                 osg::NodePath*      path = nullptr);

  public: // Scene lookup
    static osg::NodePath GetNodePath(ObjectID id);
    static void RegisterNodePath(ObjectID, osg::NodePath nodePath);
    static osg::ref_ptr<osg::Group> GetRoot();

  protected:
    std::map<ObjectID, osg::NodePath> nodesPath;

  public: // Particles ------------
    static void RegisterParticleSystem(
      ObjectID id, osg::ref_ptr<osgParticle::ParticleSystem> system);
    static void AddParticleEmitter(ObjectID particleSystemID,
                                   osg::ref_ptr<osgParticle::Emitter> emitter);

  protected:
    std::map<ObjectID, osg::ref_ptr<osgParticle::ParticleSystem>>
      particleSystems;

  public:
    osg::ref_ptr<osg::Group> sceneRoot;

    // Debug -------------------------------
  };

} // Soleil

#endif /* SOLEIL__SCENEMANAGER_H_ */
