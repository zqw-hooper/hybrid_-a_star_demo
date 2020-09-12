/*******************************************************************************
* Copyright (C) AnKobot Smart Tech Co., Ltd.  All Rights Reserved.
********************************************************************************
* File Name   : navigtion_planner_log.h
* Author      : lucianzhong
* Version     : v0.01
* Date        : 2020-05-12
* Description : navigtion_planner log定义
*******************************************************************************/

#ifndef CONTROLLER_NAVIGATION_PLANNER_LOG_H
#define CONTROLLER_NAVIGATION_PLANNER_LOG_H

#include "log/log_manager.h"

#define LOG_NAVIGATION_PLANNER_FLAG navigtion_planner
#define LOG_NAVIGATION_PLANNER "navigtion_planner"
#define NAVIGATION_DEBUG_LOG(fmt, ...)                        \
    LOG(LOG_NAME(LOG_NAVIGATION_PLANNER), __LINE__, __FILE__, \
        ak_planning_utils::LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define navigtion_INFO_LOG(fmt, ...)                          \
    LOG(LOG_NAME(LOG_NAVIGATION_PLANNER), __LINE__, __FILE__, \
        ak_planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define ROTATE_WARN_LOG(fmt, ...)                             \
    LOG(LOG_NAME(LOG_NAVIGATION_PLANNER), __LINE__, __FILE__, \
        ak_planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define NAVIGATION_ERROR_LOG(fmt, ...)                        \
    LOG(LOG_NAME(LOG_NAVIGATION_PLANNER), __LINE__, __FILE__, \
        ak_planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)

#endif // CONTROLLER_NAVIGATION_PLANNER_LOG_H