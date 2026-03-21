#pragma once

#include <fmt/format.h>
#include "boink/boink_c_api.h"

// Formatter for BoinkVec3
template<>
struct fmt::formatter<BoinkVec3>
{
    constexpr auto parse(fmt::format_parse_context& ctx)
    {
        return ctx.begin();
    }

    template<typename FormatContext>
    auto format(const BoinkVec3& v, FormatContext& ctx) const
    {
        return fmt::format_to(
            ctx.out(),
            "[{:.3f}, {:.3f}, {:.3f}]",
            v.x, v.y, v.z
        );
    }
};

// Formatter for BoinkQuaternion
template<>
struct fmt::formatter<BoinkQuaternion>
{
    constexpr auto parse(fmt::format_parse_context& ctx)
    {
        return ctx.begin();
    }

    template<typename FormatContext>
    auto format(const BoinkQuaternion& q, FormatContext& ctx) const
    {
        return fmt::format_to(
            ctx.out(),
            "[{:.3f}, {:.3f}, {:.3f}, {:.3f}]",
            q.x, q.y, q.z, q.w
        );
    }
};

// Formatter for BoinkGroundType enum
template<>
struct fmt::formatter<BoinkGroundType>
{
    constexpr auto parse(fmt::format_parse_context& ctx)
    {
        return ctx.begin();
    }

    template<typename FormatContext>
    auto format(const BoinkGroundType& type, FormatContext& ctx) const
    {
        const char* names[] = {
            "Asphalt",
            "Grass",
            "Sand",
            "Gravel",
            "Wall",
            "Kerb"
        };
        
        if(static_cast<int>(type) >= 0 && static_cast<int>(type) < 6) {
            return fmt::format_to(ctx.out(), "{}", names[static_cast<int>(type)]);
        }
        return fmt::format_to(ctx.out(), "Unknown({})", static_cast<int>(type));
    }
};
