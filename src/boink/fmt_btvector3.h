#pragma once

#include <fmt/format.h>
#include <btBulletDynamicsCommon.h>

template<>
struct fmt::formatter<btVector3>
{
    // No custom format options, just parse normally
    constexpr auto parse(fmt::format_parse_context& ctx)
    {
        return ctx.begin();
    }

    template<typename FormatContext>
    auto format(const btVector3& v, FormatContext& ctx) const
    {
        return fmt::format_to(
            ctx.out(),
            "[{:.3f}, {:.3f}, {:.3f}]",
            v.x(), v.y(), v.z()
        );
    }
};
