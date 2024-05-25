#ifndef STATE_HPP
#define STATE_HPP
#include <cstdint>

namespace action_findball {
    enum class APPROACHINGBALL {
        IDLE = 0,
        LOOKING = 1,
        TOFORWARD = 2,
        APPROACHING = 3,
        CATCHING = 4,
        LOST = 5,
        FINDING = 6,
        SUCCEED = 7
    };

    enum class Status : int8_t
    {
        SUCCEEDED = 1,
        FAILED = 2,
        RUNNING = 3,
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
} //    namespace action_findball

#endif // STATE_HPP