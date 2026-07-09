/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

namespace senseAD {
namespace perception {

enum class EnvLightStatus {
    NIGHT_DARK = 0,
    NIGHT_MOON = 1,
    NIGHT_BRIGHT_LIGHT = 2,
    CLOUDY_OUTDOOR = 3,
    SUNNY_DAYTIME = 4,
    EXTREM_LIGHT = 5
};

}  // namespace perception
}  // namespace senseAD
