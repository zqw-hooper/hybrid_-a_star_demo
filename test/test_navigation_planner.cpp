/*******************************************************************************
* Copyright (C) AnKobot Smart Tech Co., Ltd.  All Rights Reserved.
********************************************************************************
* File Name   : test_coverage_planner.cc
* Author      : evan.zhang
* Version     : v0.01
* Date        : 2020-07-21
* Description : 测试coverage_planner.cc
*******************************************************************************/


#include "navigtion_planner.h"
#include "navigation_planner_input.h"
#include "hybrid_breadth_first.h"

#include "fdfs_planner/fdfs_planner.h"
#include "pure_pursuit_planner/pure_pursuit_planner.h"
#include "path_follow_planner.h"

#include "mpc_task.h"
#include "mpc_planner.h"

#include "misc/akplanning_common_config.h"
#include "misc/robot_config.h"
#include "misc/akplanning_defs.h"

#include "rotate_planner/rotate_task.h"
#include "rotate_planner/rotate_planner.h"
#include "basic_planner/basic_task.h"
#include "basic_planner/basic_planner.h"

#include "task/task_manager.h"
// #include "speed_controller/speed_controller.h"

#include "geometry/geometry_func.h"
#include "log/log_manager.h"
#include "timer/sleep_timer.h"
#include "data_center/data_center.h"
#include "event_center/event_center.h"

#include "data/slam_data.h"
#include "data/chassis_data.h"
#include "event/global_common_event_def.h"

using namespace ak_planning_controller;
using namespace ak_planning_data;
using namespace ak_planning_utils;
// using namespace ak_planning_map;
using namespace ak_planning_planner;

DEFINE_CONFIG_TYPE(CONFIG_AKPLANNING, AKPlanning);
DEFINE_CONFIG_TYPE(CONFIG_ROBOT, Robot);

ConfigRobot *g_robot_cfg;

int main()
{
    ThreadPool pool(6);

    ConfigManager cfg_mgr;
    cfg_mgr.LoadConfig("/home/zy/Desktop/B800_2020_08_24/pms/config/robot.yaml");
    cfg_mgr.LoadConfig("/home/zy/Desktop/B800_2020_08_24/akplanning/config/planning.yaml");
    // cfg_mgr.LoadConfig("/usr/akb/config/planning.yaml");
    // cfg_mgr.LoadConfig("/usr/akb/config/pms.yaml");
    // cfg_mgr.LoadConfig("/usr/akb/config/robot.yaml");

    g_robot_cfg = dynamic_cast<ConfigRobot *>(cfg_mgr.GetSubConfig(CONFIG_ROBOT));
    g_robot_cfg->setFoortprint();

    // local_planner
    // BasicPlannerPtr &basic_planner = BasicPlanner::getInstance(&pool);
    fdfsPlannerPtr &fdfsPlanner = fdfsPlanner::getInstance(&pool);
    PurepursuitPlannerPtr &purepursuit_planner = PurepursuitPlanner::getInstance(&pool);
    MPCPlannerPtr &mpc_planner = MPCPlanner::getInstance(&pool);

    g_dc.init(cfg_mgr);
    g_ec.init(cfg_mgr);
    // g_speed_controller.init(cfg_mgr,
    //                         [&](float vl, float vr) {});
    g_tm.init(cfg_mgr);

    //
    // basic_planner->init(cfg_mgr);
    fdfsPlanner->init(cfg_mgr);
    purepursuit_planner->init(cfg_mgr);
    mpc_planner->init(cfg_mgr);

    // global_planner init   // g_表示全局
    g_navigtion_planner.init(cfg_mgr, &pool);
    // g_path_follow_planner.init(cfg_mgr, &pool);

    OPEN_LOG_EXCEPT();

    // 在task_manager中注册local_planner
    // g_tm.registerPlanner<TaskBasic>(basic_planner);
    g_tm.registerPlanner<Taskfdfs>(fdfsPlanner);
    g_tm.registerPlanner<TaskPurepursuit>(purepursuit_planner);
    g_tm.registerPlanner<TaskMPC>(mpc_planner);

    AKPoint cur_pt{0.0f, 0.0f};
    std::vector<Grid> grids;
    float range = 1.0f;

    AKGrid point_1{10, 10};
    AKGrid point_2{-10, 10};
    AKGrid point_3{-10, -10};
    AKGrid point_4{10, -10};

    grids.emplace_back(point_1);
    grids.emplace_back(point_2);
    grids.emplace_back(point_3);
    grids.emplace_back(point_4);

    CREATE_GLOBAL_PLANNER_INPUT(navigationPlannerInput, navigation_input);
    navigation_input->clean_dir = 0.0f;
    navigation_input->clean_width = 0.15f;
    AKAreaPtr area = std::make_shared<AKArea>(grids);
    navigation_input->area = area;

    g_navigtion_planner.startPlanner(navigation_input);

    // CREATE_GLOBAL_PLANNER_INPUT(CoveragePlannerInput, coverage_input);
    // coverage_input->clean_dir = M_PI / 3;
    // coverage_input->clean_width = 0.15f;

    // cv::Mat input = cv::imread("/home/evan/Pictures/test.bmp");
    // std::vector<AKAreaContour> area_contours;
    // generateContourFromMat(input, [](uint8_t val) { return val < 10; },
    //     GridRect{{200, 200},  { 200 + input.rows - 1, 200 + input.cols - 1}},
    //     area_contours);

    // AKAreaPtr area = std::make_shared<AKArea>(area_contours[1]);
    // coverage_input->area = area;

    // g_coverage_planner.startPlanner(coverage_input);

    SleepTimer t(0.1f);
    while (true)
    {
        t.sleep();
    }

    return 0;
}