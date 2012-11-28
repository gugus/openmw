#include "actionequip.hpp"

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"

#include "inventorystore.hpp"
#include "player.hpp"
#include "class.hpp"
#include "../mwmechanics/npcstats.hpp"

namespace MWWorld
{
    ActionEquip::ActionEquip (const MWWorld::Ptr& object) : Action (false, object)
    {
    }

    void ActionEquip::executeImp (const Ptr& actor)
    {
        MWWorld::Ptr player = MWBase::Environment::get().getWorld()->getPlayer().getPlayer();
        MWWorld::InventoryStore& invStore = MWWorld::Class::get(player).getInventoryStore(player);

        // slots that this item can be equipped in
        std::pair<std::vector<int>, bool> slots = MWWorld::Class::get(getTarget()).getEquipmentSlots(getTarget());

        // retrieve ContainerStoreIterator to the item
        MWWorld::ContainerStoreIterator it = invStore.begin();
        for (; it != invStore.end(); ++it)
        {
            if (*it == getTarget())
            {
                break;
            }
        }
        if(getTarget().getTypeName() == typeid(ESM::Weapon).name())
        {
            if(MWWorld::Class::get(player).getNpcStats(player).getDrawState() != MWMechanics::DrawState_Nothing)
            {
                std::cout << "cannot change weapon while one is already in use";
                return;
            }
        }

        assert(it != invStore.end());

        // equip the item in the first free slot
        for (std::vector<int>::const_iterator slot=slots.first.begin();
            slot!=slots.first.end(); ++slot)
        {
            // if all slots are occupied, replace the last slot
            if (slot == --slots.first.end())
            {
                invStore.equip(*slot, it);
                break;
            }

            if (invStore.getSlot(*slot) == invStore.end())
            {
                // slot is not occupied
                invStore.equip(*slot, it);
                break;
            }
        }
    }
}
