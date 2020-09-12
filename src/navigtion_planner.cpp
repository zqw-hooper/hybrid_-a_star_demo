/*******************************************************************************
* Copyright (C) AnKobot Smart Tech Co., Ltd.  All Rights Reserved.
********************************************************************************
* File Name   : navigtion_planner.cc
* Author      : evan.zhang
* Version     : v0.01
* Date        : 2020-07-10
* Description : Hybrid A* (全局路径规划)
*******************************************************************************/

#include "navigation_planner.h"
#include "navigation_planner_input.h"
#include "hybrid_breadth_first.h"
#include "navigation_planner_log.h"

#include "path_follow_planner.h"

#include "planner/global_planner_events.h"
#include "geometry/geometry_func.h"
#include "timer/timer.h"
#include "data_center/data_center.h"
#include "event_center/event_center.h"
#include "task/task_manager.h"

#include "local_planner/local_planner_events.h"

#include "rotate_planner/rotate_task.h"
#include "basic_planner/basic_task.h"

#include "misc/akplanning_common_config.h"
#include "misc/robot_config.h"
#include "event/global_common_event_def.h"

// #include "mpc_planner_log.h"
#include "mpc_task.h"
#include "mpc_planner.h"

using namespace ak_planning_planner;
using namespace ak_planning_utils;
using namespace ak_planning_data;
using namespace ak_planning_controller;

// namespace ak_planning_planner
// {
//     DEFINE_GLOBAL_PLANNER_INPUT(path_follow)
//     AKAreaPtr area;            // 目标清扫区域
//     float clean_dir = -1.0f;   // 清扫方向（0 - 2PI中的某个角度)，-1使用默认值
//     float clean_width = -1.0f; // 弓字形清扫间距，-1使用默认值

//     std::vector<AKPose> global_path;
//     END_GLOBAL_PLANNER_INPUT(path_follow)
// } // namespace ak_planning_planner

namespace ak_planning_planner
{
    struct navigtionPlannerStates
    { // StateWithOwner
        // Class that clients can use instead of deriving directly from State that provides convenient
        // typed access to the Owner. This class can also be chained via the StateBaseType parameter,
        // which is useful when inheriting state machines.
        struct BaseState : StateWithOwner<NavigtionPlanner>
        {
        };

        struct Disable : BaseState
        {
            virtual void OnEnter() // OnEnter is invoked when a State is created
            {
            }

            virtual void OnExit()
            {
                navigtion_DEBUG_LOG("Exit Disable...");
                std::cout << "OnEnter   " << std::endl;
            }

            virtual Transition GetTransition()
            {
                std::cout << "OnEnter navigation disable  " << std::endl;
                if (Owner().m_run_loop_enabled)
                {
                    return SiblingTransition<Enable>();
                }
                else
                {
                    return NoTransition();
                }
            }

            virtual void Update()
            {
                navigtion_DEBUG_LOG("Disable...");
            }
        };

        struct Enable : BaseState
        {
            virtual void OnEnter()
            {
                navigtion_DEBUG_LOG("Enter Enable...");
                Owner().dispatchThread();
                // navigtion_DEBUG_LOG("Enter Disable...");
                std::cout << "OnEnter  navigtion_ Enable : BaseState" << std::endl;
                bool find_path = g_navigtion_planner.Hybrid_A_star();
                std::vector<AKPose> m_global_path = g_navigtion_planner.get_global_path();
                if (find_path == true && m_global_path.size() > 2)
                {

                    std::cout << "find global path" << std::endl;

                    { //debug only  // 应该在ak_plan_service里面已经初始化好了
                        // 调用 path_follow_planner
                        CREATE_GLOBAL_PLANNER_INPUT(path_followPlannerInput, path_follow_input);
                        path_follow_input->global_path = m_global_path;
                        std::cout << "path_follow_input->global_path.size()   " << path_follow_input->global_path.size() << std::endl;
                        m_path_follow_planner_input_id = g_path_follow_planner.startPlanner(path_follow_input);
                    }
                }
                else
                {
                    std::cout << "fail to find global path, stop the robot and calling for help" << std::endl;
                    g_navigtion_planner.stopRobot();
                }
            }

            virtual void OnExit()
            {
                navigtion_DEBUG_LOG("Exit Enable...");
                // g_navigtion_planner.stopPlanner();
                // g_path_follow_planner.stopPlanner();
                Owner().stopThread();
            }

            virtual Transition GetTransition()
            {

                // // 判断pure_pursuit的event
                // EvtList &evt_list = Owner().m_evt_list;
                // for (auto &evt : evt_list)
                // {
                //     std::type_index evt_type = evt->getType();
                //     if (TYPE_EQUALS(evt_type, EvTaskFinished))
                //     {
                //         EvTaskFinishedPtr finish_evt =
                //             std::reinterpret_pointer_cast<EvTaskFinished>(evt);
                //         if (finish_evt->task_id == m_path_follow_planner_input_id)
                //         {

                //             std::cout << "receive path_follower finished_event " << std::endl;
                //             return SiblingTransition<Finished>();
                //         }
                //     }
                // }

                return NoTransition();
            }

            virtual void Update()
            {
                navigtion_DEBUG_LOG("Enable...");
            }
            int64_t m_path_follow_planner_input_id;
        };

        struct Finished : BaseState
        {
            virtual void OnEnter()
            {
                // navigtion_DEBUG_LOG("Enter Finished, Rotate task finished");
                Owner().finishPlanning();

                navigtion_DEBUG_LOG("set stop flag, change to Idle");
                Owner().m_stop_flag = true;
            }

            virtual void OnExit()
            {
                navigtion_DEBUG_LOG("Exit Finished");
            }

            virtual Transition GetTransition()
            {
                return NoTransition();
            }
        };
    };
} // namespace ak_planning_planner

NavigtionPlanner &g_navigtion_planner = NavigtionPlanner::getInstance();

float NavigtionPlanner::loadConfig(ConfigManager &cfg_mgr)
{
}

void NavigtionPlanner::initRunSM()
{
    m_run_sm.Initialize<navigtionPlannerStates::Disable>(this);
    m_run_sm.SetDebugInfo("NavigtionPlanner", TraceLevel::None);
    m_run_sm.ProcessStateTransitions();
    m_run_sm.UpdateStates();
}

void NavigtionPlanner::reset()
{
}

void NavigtionPlanner::stopRobot()
{
}

ContextPtr NavigtionPlanner::saveContext()
{
}

Transition NavigtionPlanner::restoreContext(ContextPtr ctx)
{
}

void NavigtionPlanner::getData()
{ // need to get global_map
    g_dc.getData<DataLocalMap>(m_simple_global_map);
    g_dc.getData<DataSlam>(m_slam_data);
}

void NavigtionPlanner::finishPlanning()
{
    stopRobot();

    // navigtion_INFO_LOG("finish navigtion planner, send planner finished event, input id = %ld",
    // m_input_id);

    std::cout << "navigation_planner navigtionPlanner::finishPlanning()   put_event" << std::endl;
    CREATE_EVENT(EvPlannerFinished, ev_navigtion_finished);
    ev_navigtion_finished->id = m_input_id;
    g_ec.pushEvent(ev_navigtion_finished);
}

bool NavigtionPlanner::handleInput(const GlobalPlannerInputPtr input)
{
    // Hybrid_A_star();
    return true;
}

bool NavigtionPlanner::Hybrid_A_star()
{
    // Hybrid A* searching in cost map
    {

        // need to get map from persist_map_hybrid_A_star
        for (int x = 0; x < 800; ++x)
        {
            for (int y = 0; y < 800; ++y)
            {
                persist_map_hybrid_A_star[x][y] = 255;
            }
        }

        int X = 1;
        int _ = 0;

        double SPEED = 1.45;
        double LENGTH = 0.5;

        // 栅格图赋值

        float cell_resolution_for_hybrid_A_star = 0.25;
        // >1
        int mutiple = cell_resolution_for_hybrid_A_star / 0.05;
        int row_colum_number = 800 / mutiple;
        // printf("row_colum_number  %d  \n", row_colum_number);

        row_colum_number = 800;
        vector<vector<int>> hybrid_A_start_cost_map(row_colum_number, vector<int>(row_colum_number));
        // 全部可行驶 // 0 表示空
        for (int m = 0; m < row_colum_number; m++)
        {
            for (int n = 0; n < row_colum_number; n++)
            {
                hybrid_A_start_cost_map[m][n] = 0;
            }
        }

        for (int p = 0; p < 800; p++)
        {
            for (int q = 0; q < 800; q++)
            {
                // need to get map from persist_map_hybrid_A_star
                if (persist_map_hybrid_A_star[p][q] != 255)
                {
                    // int x_hybrid = p/mutiple;
                    // int y_hybrid = q/mutiple;
                    hybrid_A_start_cost_map[p][q] = 0; // 0 表示空, 1 表示有障碍物
                }
            }
        }

        // cost_map
        vector<vector<int>> GRID = hybrid_A_start_cost_map;

        cout << "Finding path through grid ..." << endl;

        HBF hbf = HBF();

        HBF::maze_path get_path = hbf.search(GRID, START, GOAL);

        // 最终的路径点
        vector<HBF::maze_s> show_path = hbf.reconstruct_path(get_path.came_from, START, get_path.final);
        for (int p = show_path.size() - 1; p >= 0; p--)
        {
            AKPose tmp_pose;
            // to_do
            tmp_pose.theta = show_path[p].theta;           // 弧度
            tmp_pose.pt.x = (show_path[p].x - 400) * 0.05; // grid_to_pose
            tmp_pose.pt.y = (show_path[p].y - 400) * 0.05;
            navigation_global_path.push_back(tmp_pose);
        }

        // cout << "show path from start to finish" << endl;
        cout << "navigation_global_path.size()   " << navigation_global_path.size() << endl;
        for (int p = 0; p < navigation_global_path.size(); p++)
        {
            // cout << " index  " << p << " x   " << navigation_global_path[p].pt.x << " y  "
            //      << navigation_global_path[p].pt.y << " theta " << navigation_global_path[p].theta << endl;
        }

        if (show_path.size() > 2)
        {
            return true;
        }
        else
        {
            return false;
        }

        // //可视化, 保存图片
        // // JPS结果以点的形式加入图片
        // for (int i = 0; i < show_path.size(); i++)
        // {
        //     DataGrid current_point;
        //     current_point.x = show_path[i].x;
        //     current_point.y = show_path[i].y;
        //     persist_map_hybrid_A_star[current_point.x][current_point.y] = UNCOVERED;
        // }
        // my_gridmap.SaveMap("Hybrid_A_star_result.bmp", persist_map_hybrid_A_star);
    }
    // End  Hybrid A* searching in cost map
}