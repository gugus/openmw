#ifndef _GAME_RENDER_ANIMATIONLISTENER_H
#define _GAME_RENDER_ANIMATIONLISTENER_H
#include <components/nifogre/ogrenifloader.hpp>

namespace MWRender
{

class AnimationListener
{
public:
    virtual void handleTextKey(const std::string &groupname, const NifOgre::TextKeyMap::const_iterator &key) = 0;
};

}
#endif
