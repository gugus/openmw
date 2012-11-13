#include "creatureanimation.hpp"

#include <OgreEntity.h>
#include <OgreSceneManager.h>
#include <OgreSubEntity.h>

#include "renderconst.hpp"

#include "../mwbase/world.hpp"

using namespace Ogre;
using namespace NifOgre;
namespace MWRender{

CreatureAnimation::~CreatureAnimation()
{
}

CreatureAnimation::CreatureAnimation(const MWWorld::Ptr& ptr): Animation()
{
    MWWorld::LiveCellRef<ESM::Creature> *ref = ptr.get<ESM::Creature>();  
    assert (ref->mBase != NULL);
    if(!ref->mBase->mModel.empty())

    {
        std::string mesh = "meshes\\" + ref->mBase->mModel;

        createEntityList(ptr.getRefData().getBaseNode(), mesh);
        for(size_t i = 0;i < mEntityList.mEntities.size();i++)
        {
            Ogre::Entity *ent = mEntityList.mEntities[i];
            ent->setVisibilityFlags(RV_Actors);

            bool transparent = false;
            for (unsigned int j=0;j < ent->getNumSubEntities() && !transparent; ++j)
            {
                Ogre::MaterialPtr mat = ent->getSubEntity(j)->getMaterial();
                Ogre::Material::TechniqueIterator techIt = mat->getTechniqueIterator();
                while (techIt.hasMoreElements() && !transparent)
                {
                    Ogre::Technique* tech = techIt.getNext();
                    Ogre::Technique::PassIterator passIt = tech->getPassIterator();
                    while (passIt.hasMoreElements() && !transparent)
                    {
                        Ogre::Pass* pass = passIt.getNext();

                        if (pass->getDepthWriteEnabled() == false)
                            transparent = true;
                    }
                }
            }
            ent->setRenderQueueGroup(transparent ? RQG_Alpha : RQG_Main);
        }
    }
}

void CreatureAnimation::runAnimation(float timepassed)
{
    // Placeholder

    Animation::runAnimation(timepassed);
}

}
