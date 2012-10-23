#ifndef _GAME_RENDER_ANIMATION_H
#define _GAME_RENDER_ANIMATION_H

#include <vector>

#include <components/nifogre/ogre_nif_loader.hpp>
#include <openengine/ogre/renderer.hpp>
#include "../mwworld/actiontalk.hpp"
#include <components/nif/node.hpp>
#include <openengine/bullet/physic.hpp>




namespace MWRender {

class Animation {
    struct Group {
        NifOgre::TextKeyMap::const_iterator mBase;

        NifOgre::TextKeyMap::const_iterator mStart;
        NifOgre::TextKeyMap::const_iterator mStop;
        NifOgre::TextKeyMap::const_iterator mLoopStart;
        NifOgre::TextKeyMap::const_iterator mLoopStop;

        NifOgre::TextKeyMap::const_iterator mNext;

        size_t mLoops;

        Ogre::AnimationState *mState;
        Ogre::Vector3 mVelocity;

        Group() : mLoops(0), mState(0), mVelocity(0.0f)
        { }

        void readyAnimation(Ogre::Entity *ent, const std::string &groupname);
    };

    void processGroup(Group &group, float time);

protected:
    Ogre::SceneNode* mInsert;

    float mTime;
    Group mCurGroup;
    Group mNextGroup;

    bool mSkipFrame;

    NifOgre::EntityList mEntityList;
    NifOgre::TextKeyMap mTextKeys;

    bool findGroupInfo(const std::string &groupname,
                       const std::string &begin, const std::string &beginloop,
                       const std::string &endloop, const std::string &end,
                       Animation::Group *group);

    void createEntityList(Ogre::SceneNode *node, const std::string model);

public:
    Animation();
    virtual ~Animation();

    void playAnim(const std::string &groupname, const std::string &begin, const std::string &end);
    void loopAnim(const std::string &groupname,
                  const std::string &begin, const std::string &beginloop,
                  const std::string &endloop, const std::string &end, int loops);

    void playGroup(std::string groupname, int mode, int loops);
    void skipAnim();
    virtual void runAnimation(float timepassed);
};

}
#endif
