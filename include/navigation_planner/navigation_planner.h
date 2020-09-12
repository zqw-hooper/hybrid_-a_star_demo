/*******************************************************************************
* Copyright (C) AnKobot Smart Tech Co., Ltd.  All Rights Reserved.
********************************************************************************
* File Name   : navigtion_planner.h
* Author      : evan.zhang
* Version     : v0.01
* Date        : 2020-07-10
* Description : 覆盖全局规划器定义
*******************************************************************************/

#ifndef PLANNER_NAVIGATION_PLANNER_H
#define PLANNER_NAVIGATION_PLANNER_H

#include "planner/global_planner.h"
#include "data/slam_data.h"
#include "data/local_map_data.h"

namespace ak_planning_planner
{

    DEFINE_GLOBAL_PLANNER(Navigtion)

public:
    std::vector<AKPose> get_global_path()
    {
        return navigation_global_path;
    }

private:
    float loadConfig(ConfigManager &cfg_mgr);
    void initRunSM();
    void reset();
    void getData();
    void finishPlanning();
    bool handleInput(const GlobalPlannerInputPtr input);
    ContextPtr saveContext();
    Transition restoreContext(ContextPtr ctx);

    void stopRobot();

    // log相关函数
    void saveCleanStatusLog();
    void saveEndPtLog();

    bool Hybrid_A_star();

    // 参数

    // 状态机相关

    // 恢复相关（暂停恢复，异常恢复）

    // log数据的路径

    DataSlam m_slam_data;
    // 全局地图
    DataLocalMap m_simple_global_map;
    // Hybrid A* 需要起点和终点, 全局地图
    int persist_map_hybrid_A_star[800][800];

    // vector<vector<int>> persist_map_hybrid_A_star;

    // 起点
    // vector<double> START = {415, 600, 0.0};
    std::vector<double> START = {400, 400, 0.0};
    // 终点
    std::vector<double> GOAL = {500, 400, 0};

    // 输出的全局的路径点
    std::vector<AKPose> navigation_global_path;

    friend struct navigtionPlannerStates;

    END_GLOBAL_PLANNER(Navigtion)

} // namespace ak_planning_planner

extern ak_planning_planner::NavigtionPlanner &g_navigtion_planner;

#endif // PLANNER_Navigtion_PLANNER_H