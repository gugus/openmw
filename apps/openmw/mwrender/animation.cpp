#include "animation.hpp"

#include <OgreHardwarePixelBuffer.h>
#include <OgreSkeletonInstance.h>
#include <OgreEntity.h>
#include <OgreBone.h>
#include <OgreSubMesh.h>
#include <OgreSceneManager.h>

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"
#include "../mwbase/mechanicsmanager.hpp"

namespace MWRender
{

void Animation::Group::readyAnimation(Ogre::Entity *ent, const std::string &groupname)
{
    mState = ent->getAnimationState(groupname);

    Ogre::SkeletonInstance *skel = ent->getSkeleton();
    Ogre::Animation *anim = skel->getAnimation(groupname);
    /* FIXME: Not all skeletons have a Bip01 node, creatures for instance use a different name. We
     * should probably be using the skeleton root bone, but the current Ogre NIF loader will create
     * bones before the first animated bone. */
    if(anim && skel->hasBone("Bip01"))
    {
        unsigned short handle = skel->getBone("Bip01")->getHandle();
        if(anim->hasNodeTrack(handle))
        {
            Ogre::NodeAnimationTrack *track = anim->getNodeTrack(handle);
            int numkfs = track->getNumKeyFrames();
            if(numkfs > 1)
            {
                Ogre::TransformKeyFrame *startframe = track->getNodeKeyFrame(0);
                Ogre::TransformKeyFrame *endframe = track->getNodeKeyFrame(numkfs-1);

                mVelocity = (endframe->getTranslate() - startframe->getTranslate()) /
                            (endframe->getTime() - startframe->getTime());
            }
        }
    }
}


Animation::Animation()
    : mInsert(NULL)
    , mTime(0.0f)
    , mSkipFrame(false)
{
}

Animation::~Animation()
{
    if(mInsert)
    {
        Ogre::SceneManager *sceneMgr = mInsert->getCreator();
        for(size_t i = 0;i < mEntityList.mEntities.size();i++)
            sceneMgr->destroyEntity(mEntityList.mEntities[i]);
    }
    mEntityList.mEntities.clear();
    mEntityList.mSkelBase = 0;
}


struct checklow {
    bool operator()(const char &a, const char &b) const
    {
        return ::tolower(a) == ::tolower(b);
    }
};


void Animation::createEntityList(Ogre::SceneNode *node, const std::string model)
{
    mInsert = node->createChildSceneNode();
    assert(mInsert);

    mEntityList = NifOgre::NIFLoader::createEntities(mInsert, &mTextKeys, model);
    if(mEntityList.mSkelBase)
    {
        Ogre::AnimationStateSet *aset = mEntityList.mSkelBase->getAllAnimationStates();
        Ogre::AnimationStateIterator as = aset->getAnimationStateIterator();
        while(as.hasMoreElements())
        {
            Ogre::AnimationState *state = as.getNext();
            state->setEnabled(false);
            state->setLoop(false);
        }
    }
}


bool Animation::findGroupInfo(const std::string &groupname, const std::string &begin, const std::string &beginloop, const std::string &endloop, const std::string &end, Animation::Group *group)
{
    group->mBase = mTextKeys.end();
    group->mStart = mTextKeys.end();
    group->mLoopStart = mTextKeys.end();
    group->mLoopStop = mTextKeys.end();
    group->mStop = mTextKeys.end();

    const std::string &start = groupname+": "+begin;
    const std::string &startloop = groupname+": "+beginloop;
    const std::string &stoploop = groupname+": "+endloop;
    const std::string &stop = groupname+": "+end;

    /*const std::string &start2 = groupname+" start";
    const std::string &startloop2 = groupname+" loop start";
    const std::string &stop2 = groupname+" stop";
    const std::string &stoploop2 = groupname+" loop stop";*/

    NifOgre::TextKeyMap::const_iterator iter;
    for(iter = mTextKeys.begin();iter != mTextKeys.end();iter++)
    {
        std::string::const_iterator strpos = iter->second.begin();
        std::string::const_iterator strend = iter->second.end();
        size_t strlen = strend-strpos;

        if(iter->second.find(':') != groupname.length() ||
           std::mismatch(strpos, strend, groupname.begin(), checklow()).second != groupname.end())
            continue;

        if(group->mBase == mTextKeys.end())
            group->mBase = iter;

        if(start.size() <= strlen && std::mismatch(strpos, strend, start.begin(), checklow()).first == strend)
        {
            group->mStart = iter;
            group->mLoopStart = iter;
        }
        else if(stop.size() <= strlen && std::mismatch(strpos, strend, stop.begin(), checklow()).first == strend)
        {
            group->mStop = iter;
            if(group->mLoopStop == mTextKeys.end())
                group->mLoopStop = iter;
        }
        if(startloop.size() <= strlen && std::mismatch(strpos, strend, startloop.begin(), checklow()).first == strend)
        {
            group->mLoopStart = iter;
        }
        else if(stoploop.size() <= strlen && std::mismatch(strpos, strend, stoploop.begin(), checklow()).first == strend)
        {
            group->mLoopStop = iter;
        }
        if(group->mStart != mTextKeys.end() && group->mLoopStart != mTextKeys.end() &&
           group->mLoopStop != mTextKeys.end() && group->mStop != mTextKeys.end())
        {
            group->mNext = group->mStart;
            return true;
        }

        /*if(start2.size() <= strlen && std::mismatch(strpos, strend, start2.begin(), checklow()).first == strend)
        {
            times->mStart = iter->first;
            times->mLoopStart = iter->first;
        }
        else if(startloop2.size() <= strlen && std::mismatch(strpos, strend, startloop2.begin(), checklow()).first == strend)
        {
            times->mLoopStart = iter->first;
        }
        else if(stoploop2.size() <= strlen && std::mismatch(strpos, strend, stoploop2.begin(), checklow()).first == strend)
        {
            times->mLoopStop = iter->first;
        }
        else if(stop2.size() <= strlen && std::mismatch(strpos, strend, stop2.begin(), checklow()).first == strend)
        {
            times->mStop = iter->first;
            if(times->mLoopStop < 0.0f)
                times->mLoopStop = iter->first;
            break;
        }*/
    }

    return false;
}


void Animation::processGroup(Group &group, float time)
{
    while(group.mNext != mTextKeys.end() && time >= group.mNext->first)
    {
        // TODO: Process group.mNext->second
        MWWorld::Ptr ptr = MWBase::Environment::get().getWorld()->searchPtrViaHandle(mInsert->getParent()->getName());
        MWBase::Environment::get().getMechanicsManager()->animationNotify(ptr,group.mNext->second);
        onKeyReached();
        group.mNext++;
    }
}


void Animation::playAnim(const std::string &groupname, const std::string &begin, const std::string &end)
{
    loopAnim(groupname, begin, begin, end, end, 1);
}

void Animation::loopAnim(const std::string &groupname,
                const std::string &begin, const std::string &beginloop,
                const std::string &endloop, const std::string &end, int loops)
{
    if(!mEntityList.mSkelBase)
        throw std::runtime_error("No skeleton on actor");

    Group group;
    group.mLoops = loops;
    group.readyAnimation(mEntityList.mSkelBase, groupname);

    if(!findGroupInfo(groupname, begin, beginloop, endloop, end, &group))
        throw std::runtime_error("Failed to find info for animation group "+groupname);

    if(mCurGroup.mState)
        mCurGroup.mState->setEnabled(false);
    mCurGroup = group;
    mCurGroup.mState->setEnabled(true);
    mNextGroup.mState = 0;
    mTime = mCurGroup.mStart->first;
}


bool Animation::findCustomGroupNote(const std::string &groupname,const std::string &note,float time)
{
    time = -1;
    NifOgre::TextKeyMap::const_iterator iter;
    for(iter = mTextKeys.begin();iter != mTextKeys.end();iter++)
    {
        std::string::const_iterator strpos = iter->second.begin();
        std::string::const_iterator strend = iter->second.end();
        size_t strlen = strend-strpos;

        if(note.size() <= strlen && std::mismatch(strpos, strend, note.begin(), checklow()).first == strend)
        {
            time = iter->first;
            return true;
        }
    }
    return false;
}

void Animation::playGroup(std::string groupname, int mode, int loops)
{
    std::cout << "playanim" << groupname;
    if(!mEntityList.mSkelBase)
        throw std::runtime_error("No skeleton on actor");
    std::transform(groupname.begin(), groupname.end(), groupname.begin(), ::tolower);

    Group group;
    group.mLoops = loops;
    group.readyAnimation(mEntityList.mSkelBase, groupname);

    if(groupname == "all")
    {
        group.mBase = group.mStart = group.mLoopStart = group.mLoopStop = group.mStop = mTextKeys.end();

        NifOgre::TextKeyMap::const_iterator iter = mTextKeys.begin();
        if(iter != mTextKeys.end())
        {
            group.mBase = group.mStart = group.mLoopStart = group.mNext = iter;
            iter = mTextKeys.end();
            group.mLoopStop = group.mStop = --iter;
        }
    }
    else if(!findGroupInfo(groupname, "start", "loop start", "loop stop", "stop", &group))
        throw std::runtime_error("Failed to find info for animation group "+groupname);

    if(mode == 0 && mCurGroup.mState)
        mNextGroup = group;
    else
    {
        if(mCurGroup.mState)
            mCurGroup.mState->setEnabled(false);
        mCurGroup = group;
        mCurGroup.mState->setEnabled(true);
        mNextGroup.mState = 0;
        if(mode == 2)
            mCurGroup.mNext = mCurGroup.mLoopStart;
        mTime = mCurGroup.mNext->first;
    }
}

void Animation::skipAnim()
{
    mSkipFrame = true;
}

void Animation::runAnimation(float timepassed)
{
    if(mCurGroup.mState && !mSkipFrame)
    {
        mTime += timepassed;
    do_more:
        while(mCurGroup.mLoops > 1 && mTime >= mCurGroup.mLoopStop->first)
        {
            processGroup(mCurGroup, mCurGroup.mLoopStop->first);
            mCurGroup.mNext = mCurGroup.mLoopStart;

            mCurGroup.mLoops--;
            mTime = mTime - mCurGroup.mLoopStop->first + mCurGroup.mLoopStart->first;
        }
        if(mCurGroup.mLoops <= 1 && mTime >= mCurGroup.mStop->first)
        {
            processGroup(mCurGroup, mCurGroup.mStop->first);
            if(mNextGroup.mState)
            {
                mTime = mTime - mCurGroup.mStop->first + mNextGroup.mStart->first;
                mCurGroup.mState->setEnabled(false);
                mCurGroup = mNextGroup;
                mCurGroup.mState->setEnabled(true);
                mNextGroup.mState = 0;
                goto do_more;
            }
            mTime = mCurGroup.mStop->first;
        }
        else
            processGroup(mCurGroup, mTime);

        mCurGroup.mState->setTimePosition(mTime);
        mInsert->setPosition(mCurGroup.mVelocity * -(mTime - mCurGroup.mBase->first));
    }
    mSkipFrame = false;
}

}
