#include "weapontype.hpp"

std::string getWeaponType(MWWorld::Ptr mPtr)
{
    std::string weaponType = "";

    MWWorld::ContainerStoreIterator iter = MWWorld::Class::get(mPtr).getInventoryStore(mPtr).
        getSlot(MWWorld::InventoryStore::Slot_CarriedRight);

    if (iter == MWWorld::Class::get(mPtr).getInventoryStore(mPtr).end())
    {
        weaponType = "handtohand";
        return weaponType;
    }

    int type = iter->get<ESM::Weapon>()->mBase->mData.mType;

    if((type == ESM::Weapon::LongBladeOneHand) || (type == ESM::Weapon::AxeOneHand) || (type == ESM::Weapon::BluntOneHand) ||
        (type == ESM::Weapon::ShortBladeOneHand))
    {
        weaponType = "weapononehand";
    }
    else if ((type == ESM::Weapon::AxeTwoHand) || (type == ESM::Weapon::BluntTwoClose) || (type == ESM::Weapon::BluntTwoClose) ||
        (type == ESM::Weapon::LongBladeTwoHand))
    {
        weaponType = "weapontwohand";
    }
    else if ((type == ESM::Weapon::BluntTwoWide) || (type == ESM::Weapon::SpearTwoWide))
    {
        weaponType = "weapontwowide";
    }
    else if (type == ESM::Weapon::MarksmanBow)
    {
        weaponType  = "bowandarrow";
    }
    else if (type == ESM::Weapon::MarksmanCrossbow)
    {
        weaponType = "crossbow";
    }
    else if (type == ESM::Weapon::MarksmanThrown)
    {
        weaponType = "throwweapon";
    }

    return weaponType;
}