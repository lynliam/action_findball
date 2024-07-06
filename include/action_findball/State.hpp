#ifndef STATE_HPP
#define STATE_HPP
#include <cstdint>

namespace action_findball {
    enum class APPROACHINGBALL {
        IDLE = 0,
        LOOKING = 1,
        TOFORWARD = 2,
        APPROACHING1 = 3,
        APPROACHING2 = 4,
        CATCHING = 5,
        CATCHING_2 = 6,
        CATCHING_3 = 7,
        LOST = 8,
        FINDING = 9,
        BLOCK = 10,
        CATCHING_PURPLE = 11,
        BUMP_AWAY = 12,
        NO_BALL = 13,
        BUCKLE = 14,
        BACKWARD = 15,
        SUCCEED = 16,
        FAIL = 17,
        CRASH = 18
    };

    enum class Status : int8_t
    {
        SUCCEEDED = 1,
        FAILED = 2,
        RUNNING = 3,
        CRASHED = 4,
        IDLE = 5
    };

    enum class Situation : int8_t
    {
        Direct = 0,
        Purple_block = 1
    };

    enum class CATCHBALL {
        IDLE = 0,
        BOUNCING = 1,
        CATCHING = 2,
        BACKING = 3,
        LOST = 4,
        timeout = 5,
        SUCCEED = 6
    };

    enum class TOWARD {
        FRONT = 0,
        LEFT = 1,
        RIGHT = 2,
    };
} //    namespace action_findball

#endif // STATE_HPP