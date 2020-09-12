/*******************************************************************************
* Copyright (C) AnKobot Smart Tech Co., Ltd.  All Rights Reserved.
********************************************************************************
* File Name   : navigation_planner_input.h
* Author      : evan.zhang
* Version     : v0.01
* Date        : 2020-07-10
* Description : 覆盖全局规划器输入
// HEADER_H
*******************************************************************************/

#ifndef PLANNER_NAVIGATION_PLANNER_INPUT_H
#define PLANNER_NAVIGATION_PLANNER_INPUT_H

#include "planner/global_planner_input.h"
#include "misc/akplanning_typedefs.h"

namespace ak_planning_planner
{
    DEFINE_GLOBAL_PLANNER_INPUT(Navigation)
        AKAreaPtr area;                     // 目标清扫区域
        float clean_dir         = -1.0f;    // 清扫方向（0 - 2PI中的某个角度)，-1使用默认值
        float clean_width       = -1.0f;    // 弓字形清扫间距，-1使用默认值
    END_GLOBAL_PLANNER_INPUT(Navigation)
}

#endif // PLANNER_NAVIGATION_PLANNER_INPUT_H