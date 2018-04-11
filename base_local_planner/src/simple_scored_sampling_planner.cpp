/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: TKruse
 *********************************************************************/

#include <base_local_planner/simple_scored_sampling_planner.h>

#include <ros/console.h>

namespace base_local_planner {
  
  SimpleScoredSamplingPlanner::SimpleScoredSamplingPlanner(std::vector<TrajectorySampleGenerator*> gen_list, std::vector<TrajectoryCostFunction*>& critics, int max_samples)
  {
    max_samples_ = max_samples;
    gen_list_ = gen_list;
    critics_ = critics;
  }

  double SimpleScoredSamplingPlanner::scoreTrajectory(Trajectory& traj, double best_traj_cost) {
    double traj_cost = 0;
    int gen_id = 0;
    //多个评价函数
    for(std::vector<TrajectoryCostFunction*>::iterator score_function = critics_.begin(); score_function != critics_.end(); ++score_function) {
      TrajectoryCostFunction* score_function_p = *score_function;
      if (score_function_p->getScale() == 0) {
        continue;
      }
      //真正的评价在这里
      double cost = score_function_p->scoreTrajectory(traj);
      if (cost < 0) {
        ROS_DEBUG("Velocity %.3lf, %.3lf, %.3lf discarded by cost function  %d with cost: %f", traj.xv_, traj.yv_, traj.thetav_, gen_id, cost);
        traj_cost = cost;
        break;
      }
      if (cost != 0) {
        cost *= score_function_p->getScale();
      }
      traj_cost += cost;
      //best_traj_cost 初始值 -1
      if (best_traj_cost > 0) {
        // since we keep adding positives, once we are worse than the best, we will stay worse
        if (traj_cost > best_traj_cost) {
          break;
        }
      }
      gen_id ++;
    }


    return traj_cost;
  }

  bool SimpleScoredSamplingPlanner::findBestTrajectory(Trajectory& traj, std::vector<Trajectory>* all_explored)
  {
    Trajectory loop_traj;
    Trajectory best_traj;
    double loop_traj_cost, best_traj_cost = -1;
    bool gen_success;
    int count, count_valid;
    //准备评价函数
    for (std::vector<TrajectoryCostFunction*>::iterator loop_critic = critics_.begin(); loop_critic != critics_.end(); ++loop_critic)
    {
      TrajectoryCostFunction* loop_critic_p = *loop_critic;
      //通过prepare函数实现的储存计算path_dist，goal_dist在map cell里面
      //所有的map cell 在prepare 函数里面计算更新
      if (loop_critic_p->prepare() == false) {
        ROS_WARN("A scoring function failed to prepare");
        return false;
      }
    }
    //开始评价gen里面的样本
    for (std::vector<TrajectorySampleGenerator*>::iterator loop_gen = gen_list_.begin(); loop_gen != gen_list_.end(); ++loop_gen)
    {
      count = 0;
      count_valid = 0;
      TrajectorySampleGenerator* gen_ = *loop_gen;
     //检查所有的样本
      while (gen_->hasMoreTrajectories())
      {
        gen_success = gen_->nextTrajectory(loop_traj);
        if (gen_success == false)
        {
          // TODO use this for debugging
          continue;
        }
        //开始一次添加3个cost，开始评价
        loop_traj_cost = scoreTrajectory(loop_traj, best_traj_cost);

        if (all_explored != NULL)
        {
          loop_traj.cost_ = loop_traj_cost;
          all_explored->push_back(loop_traj);
        }
        //正的评分被保留
        if (loop_traj_cost >= 0)
        {
          count_valid++;
          //初始值小于零，通过比较留下最好的
          if (best_traj_cost < 0 || loop_traj_cost < best_traj_cost) {
            best_traj_cost = loop_traj_cost;
            best_traj = loop_traj;
          }
        }
        count++;
        //是否到了最大的样本数
        if (max_samples_ > 0 && count >= max_samples_)
        {
          break;
        }        
      }
      //正的评分被保留
      //traj就是速度的产生
      //为什么只传出一个的速度，但是多个点？？？？？
      //一个trajectory可以有多个点，但是只有一组速度
      if (best_traj_cost >= 0) {
        traj.xv_ = best_traj.xv_;
        traj.yv_ = best_traj.yv_;
        traj.thetav_ = best_traj.thetav_;
        traj.cost_ = best_traj_cost;
        traj.resetPoints();
        double px, py, pth;
        //多个点
        for (unsigned int i = 0; i < best_traj.getPointsSize(); i++)
        {

          best_traj.getPoint(i, px, py, pth);
          traj.addPoint(px, py, pth);//传出路径
        }
      }
      ROS_DEBUG("Evaluated %d trajectories, found %d valid", count, count_valid);
      //正的评分被保留
      if (best_traj_cost >= 0) {
        // do not try fallback generators
        break;
      }
    }
    return best_traj_cost >= 0;
  }

  
}// namespace
