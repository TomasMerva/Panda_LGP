#pragma once

#include <vector>
#include <string>
#include <panda_lgp/constraints/constraints.h>


/*
1. problem -> musim ulozit presne parametre pre akcie zo skeletonu (MoveF nemusi byt hned grey region)
2. problem -> Struktury na akcie som absolutne nepouzil v podstate, lebo vyplnujem iba SkeletonEntry()
*/


namespace logic
{

enum SkeletonAction
{
    MoveF,
    Pick,
    MoveH,
    Place
};



// class SkeletonEntry
// {
//     public:
//         SkeletonEntry(SkeletonAction action_name, std::vector<std::string> frames)
//         {
//             switch (action_name)
//             {
//                 case SkeletonAction::MoveF:
//                 {
//                     /* code */
//                     break;
//                 }
//                 case SkeletonAction::MoveH:
//                 {
//                     break;
//                 }
//                 default:
//                     break;
//             }
//         }
//     private:
//         /* data */
// };





} // namespace