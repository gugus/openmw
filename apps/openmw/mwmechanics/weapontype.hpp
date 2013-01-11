#ifndef GAME_MWMECHANICS_WEAPONTYPE_H
#define GAME_MWMECHANICS_WEAPONTYPE_H


#include "../mwworld/ptr.hpp"
#include "../mwworld/containerstore.hpp"
#include "../mwworld/class.hpp"
#include "../mwworld/inventorystore.hpp"


/**
*Return the weapon type that an NPC (or the player) is carying. It's the name used by the animation system.
*/
std::string getWeaponType(MWWorld::Ptr mPtr);

#endif