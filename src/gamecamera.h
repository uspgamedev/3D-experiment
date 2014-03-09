
#ifndef GAMECAMERA_H_
#define GAMECAMERA_H_

#include <OgreVector3.h>
#include <gameobject.h>

namespace Ogre {
class Camera;
class SceneManager;
class Quaternion;
}
namespace OIS {
class MouseEvent;
}

namespace ShipProject {

class GameCamera {
public:
    GameCamera(Ogre::SceneManager* sceneMgr, const std::string& camName="DefaultCam");
    ~GameCamera();

    void AttachTo(GameObject* object);

    void SetParameters(const Ogre::Vector3& parent_origin_offset = Ogre::Vector3::ZERO, double max_dist=7.5);

    void SetDistance(double dist);
    double GetDistance() { return dist_; }
    void Rotate(double yaw, double pitch);

    Ogre::Camera* camera() { return camera_; }
    GameObject* parent() { return parent_; }
    Ogre::Quaternion orientation();

    void injectMouseMoved( const OIS::MouseEvent &arg );

protected:
    Ogre::Camera* camera_;
	GameObject* parent_;

    Ogre::Vector3 offset_; // offset to parent origin
    double dist_;          // distance to origin (parent origin+offset)
    double max_dist_;
    double cumulative_pitch_;

    void setupTransform();
};

}
#endif
