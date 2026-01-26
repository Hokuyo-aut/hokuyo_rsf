/*
 * hokuyo_spel_master
 * 
 * Copyright (c) 2025 LOCT Co., Ltd.
 * All rights reserved.
 *
 * This software is the property of LOCT Co., Ltd.
 * It is provided solely for evaluation and joint development purposes
 * under prior agreement with LOCT Co., Ltd.
 *
 * Redistribution or use outside the agreed scope is prohibited
 * without written permission from LOCT Co., Ltd.
 */

 #pragma once

namespace hsp {

enum class LIOStatus : uint8_t {
  SLEEP    = 1,
  START    = 2,
  RESTART  = 3,
  RESET    = 4,
  SHUTDOWN = 5,
};

} // namespace hsp