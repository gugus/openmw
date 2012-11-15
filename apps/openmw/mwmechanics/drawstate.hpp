#ifndef GAME_MWMECHANICS_DRAWSTATE_H
#define GAME_MWMECHANICS_DRAWSTATE_H

namespace MWMechanics
{
    /// \note The _ suffix is required to avoid a collision with a Windoze macro. Die, Microsoft! Die!
    enum DrawState_
    {
        DrawState_Weapon = 0,
        DrawState_Spell = 1,
        DrawState_Nothing = 2,
        DrawState_Drawing_Spell = 3,
        DrawState_UnDrawing_Spell = 4,
        DrawState_Drawing_Weapon = 5,
        DrawState_Drawing_Weapon_Attached = 6,
        DrawState_UnDrawing_Weapon = 7,
        DrawState_UnDrawing_Weapon_Attached = 8
    };
}

#endif
